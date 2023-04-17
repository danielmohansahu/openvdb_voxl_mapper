/* Simple ground plane extraction operation.
 *
 * Ground plane extraction operations.
 */

// STL
#include <concepts>
#include <optional>

// OpenVDB
#include <openvdb/points/PointAttribute.h>

// NanoVDB
#include <nanovdb/util/OpenToNanoVDB.h>
#include <nanovdb/util/CudaDeviceBuffer.h>
#include <nanovdb/util/GridHandle.h>

// OVM
#include <openvdb_voxel_mapper/operations/ground_plane_extraction.h>


// TEMPORARY
#include "Image.hpp"

namespace ovm::ops
{

std::optional<Map> ground_plane_extraction_geometric(const openvdb::points::PointDataGrid::Ptr& grid)
{
  using namespace openvdb::points;
  
  // sanity check inputs
  if (!grid || grid->empty())
    return std::nullopt;

  // initialize output map dimensions and pose from the grid's bounding box
  const auto bbox = grid->evalActiveVoxelBoundingBox();
  ovm::Map result {bbox, grid->transform()};

  // Iterate over all the leaf nodes in the grid.
  for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf)
  {
    // Create a read-only AttributeHandle. Position always uses Vec3f.
    const AttributeHandle<openvdb::Vec3f> handle(leaf->constAttributeArray("P"));

    // Iterate over the point indices in the leaf.
    for (auto idx = leaf->beginIndexOn(); idx; ++idx)
    {
      // Extract the voxel-space position of the point.
      const openvdb::Vec3f voxelPosition = handle.get(*idx);

      // Extract the index-space position of the voxel.
      const openvdb::Vec3d xyz = idx.getCoord().asVec3d();

      // Compute the world-space position of the point.
      const openvdb::Vec3f worldPosition = grid->transform().indexToWorld(voxelPosition + xyz);
      
      // update 2D map value with the lowest Z value found so far
      const auto coords = xyz - bbox.min();
      auto& val = result.map.coeffRef(coords.y(), coords.x());
      val = (std::isnan(val)) ? worldPosition.z() : std::min(val, worldPosition.z());
    }
  }

  return result;
}

extern "C" void launch_ground_plane_kernel(const nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& gridHandle,
                                           const openvdb::CoordBBox& bbox,
                                           nanovdb::ImageHandle<nanovdb::CudaDeviceBuffer>& imgHandle,
                                           cudaStream_t stream);

std::optional<Map> ground_plane_extraction_geometric_cuda(const openvdb::points::PointDataGrid::Ptr& grid)
{
  // sanity check inputs
  if (!grid || grid->empty())
    return std::nullopt;

  // initialize output map dimensions and pose from the grid's bounding box
  const auto bbox = grid->evalActiveVoxelBoundingBox();
  ovm::Map result {bbox, grid->transform()};

  // convert grid from OpenVDB to NanoVDB grid
  auto gridHandle = nanovdb::openToNanoVDB<nanovdb::CudaDeviceBuffer>(*grid);

  // construct a nanoVDB image (proxy for MAP result)
  nanovdb::ImageHandle<nanovdb::CudaDeviceBuffer> imgHandle(result.map.cols(), result.map.rows());

  // Create a CUDA stream to allow for asynchronous copy of pinned CUDA memory.
  cudaStream_t stream;
  cudaStreamCreate(&stream);

  // Copy the NanoVDB grid and Image to the GPU asynchronously
  gridHandle.deviceUpload(stream, false);
  imgHandle.deviceUpload(stream, false);

  // execute core method on the GPU
  launch_ground_plane_kernel(gridHandle, bbox, imgHandle, stream);


  // @TODO, convert from NanoVDB image to Eigen




  // Destroy the CUDA stream
  cudaStreamDestroy(stream);
  
  // @TODO!
  return std::nullopt;
}

} // namespace ovm::ops