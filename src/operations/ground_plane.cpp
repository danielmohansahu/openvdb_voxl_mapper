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
#include <openvdb_voxel_mapper/operations/ground_plane.h>

namespace ovm::ops
{

std::optional<Eigen::MatrixXf> min_z_ground_plane(const openvdb::points::PointDataGrid::Ptr& grid)
{
  using namespace openvdb::points;
  
  // sanity check inputs
  if (!grid || grid->empty())
    return std::nullopt;

  // initialize output map dimensions and pose from the grid's bounding box
  const auto bbox = grid->evalActiveVoxelBoundingBox();
  Eigen::MatrixXf result = Eigen::MatrixXf::Constant(bbox.dim().y(), bbox.dim().x(), NAN);
  
  // Iterate over all the leaf nodes in the grid.
  for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf)
  {
    // Create a read-only AttributeHandle. Position always uses Vec3f.
    const AttributeHandle<AttPositionT> handle(leaf->constAttributeArray(ATT_POSITION));

    // Iterate over the point indices in the leaf.
    for (auto idx = leaf->beginIndexOn(); idx; ++idx)
    {
      // Extract the voxel-space position of the point.
      const AttPositionT voxelPosition = handle.get(*idx);

      // Extract the index-space position of the voxel.
      const openvdb::Vec3d xyz = idx.getCoord().asVec3d();

      // Compute the world-space position of the point.
      const AttPositionT worldPosition = grid->transform().indexToWorld(voxelPosition + xyz);
      
      // update 2D map value with the lowest Z value found so far
      const auto [row, col] = idx_to_rc(xyz.x(), xyz.y(), bbox.min().x(), bbox.max().y());
      auto& val = result.coeffRef(row, col);
      val = (std::isnan(val)) ? worldPosition.z() : std::min(val, worldPosition.z());
    }
  }

  return result;
}

extern "C" void launch_ground_plane_kernel(const nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& gridHandle,
                                           const openvdb::CoordBBox& bbox,
                                           float* deviceMap,
                                           cudaStream_t stream);

std::optional<Eigen::MatrixXf> min_z_ground_plane_cuda(const openvdb::points::PointDataGrid::Ptr& grid)
{
  using namespace openvdb::points;
  
  // sanity check inputs
  if (!grid || grid->empty())
    return std::nullopt;

  // initialize output map dimensions and pose from the grid's bounding box
  const auto bbox = grid->evalActiveVoxelBoundingBox();
  Eigen::MatrixXf result = Eigen::MatrixXf::Constant(bbox.dim().y(), bbox.dim().x(), NAN);

  // more sanity checks (everything needs to have the same size)
  assert (bbox.dim().x() == (bbox.max().x() - bbox.min().x()) + 1);
  assert (bbox.dim().y() == (bbox.max().y() - bbox.min().y()) + 1);
  assert (bbox.dim().x() == result.cols());
  assert (bbox.dim().y() == result.rows());

  // convert grid from OpenVDB to NanoVDB grid
  // N.B.: We first create a copy of our grid and drop unnecessary attributes to:
  //  1) Reduce the move time from CPU -> GPU
  //  2) Avoid trying to port unsupported types to NanoVDB (like double...)
  // This is ok for _this_ operation, but other operations will need to proceed differently.
  auto temporary_grid = grid->deepCopy();
  if (temporary_grid->tree().cbeginLeaf()->attributeSet().descriptor().find(ATT_CONFIDENCE) != AttributeSet::INVALID_POS)
    dropAttributes(temporary_grid->tree(), std::vector<std::string>{ATT_STAMP, ATT_CONFIDENCE});
  else
    dropAttribute(temporary_grid->tree(), ATT_STAMP);
  auto gridHandle = nanovdb::openToNanoVDB<nanovdb::CudaDeviceBuffer>(temporary_grid);

  // construct a buffer to support host <-> GPU conversion
  nanovdb::CudaDeviceBuffer mapBuffer;
  mapBuffer.init(bbox.dim().x() * bbox.dim().y() * sizeof(float));

  // Create a CUDA stream to allow for asynchronous copy of pinned CUDA memory.
  cudaStream_t stream;
  cudaStreamCreate(&stream);

  // Copy the NanoVDB grid and matrix to the GPU asynchronously
  gridHandle.deviceUpload(stream, false);
  mapBuffer.deviceUpload();

  // execute core method on the GPU
  float* device_map = reinterpret_cast<float*>(mapBuffer.deviceData());
  launch_ground_plane_kernel(gridHandle, bbox, device_map, stream);

  // copy modified map back to host
  mapBuffer.deviceDownload();

  // convert from matHandle to eigen result
  float* host_map = reinterpret_cast<float*>(mapBuffer.data());
  const auto xmin = bbox.min().x(), xmax = bbox.max().x();
  const auto ymin = bbox.min().y(), ymax = bbox.max().y();
  for (auto i = 0; i != (xmax - xmin + 1); ++i)
    for (auto j = 0; j != (ymax - ymin + 1); ++j)
    {
      // @TODO this indexing / frame nonsense really needs to be abstracted.
      const auto [row, col] = idx_to_rc(i + xmin, j + ymin, xmin, ymax);
      result.coeffRef(row, col) = host_map[j + i * (ymax - ymin + 1)];
    }

  // Destroy the CUDA stream
  cudaStreamDestroy(stream);

  // return updated map
  return result;
}

} // namespace ovm::ops