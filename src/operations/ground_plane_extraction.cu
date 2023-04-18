/* Simple ground plane extraction operation.
 *
 * Ground plane extraction operations.
 */

// STL
#include <iostream>

// OpenVDB
#include <openvdb/math/Coord.h>

// NanoVDB
#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/CudaDeviceBuffer.h>
#include <nanovdb/util/GridHandle.h>

// OVM
#include <openvdb_voxel_mapper/types.h>

namespace ovm::ops
{

// kernel to iterate through a single column of voxels and return the lowest point height
__global__ void min_z_kernel(const nanovdb::NanoGrid<uint32_t>& grid,
                             float* deviceMap,
                             const int xmin, const int xmax,
                             const int ymin, const int ymax,
                             const int zmin, const int zmax)
{
  // get the indices of this voxel (which correspond to the 2D grid, and the CUDA thread)
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  const int j = blockIdx.y * blockDim.y + threadIdx.y;

  // ignore out of bounds threads
  if (i > xmax - xmin || j > ymax - ymin)
    return;

  // construct coordinate accessor for openvdb voxel in index space ([0,inf) -> (-inf, inf))
  const nanovdb::Coord ijk_begin {i + xmin, j + ymin, 0};

  // construct accessor and intra-voxel point iterators
  const nanovdb::Vec3f *begin = nullptr, *end = nullptr;

  // get device grid accessor
  auto acc = grid.getAccessor();

  // initialize minimum Z height
  float min_z = NAN;

  // starting from the first voxel, iterate upwards
  for (int k = zmin; k != zmax; ++k)
  {
    // get coordinate of this voxel
    const auto ijk = ijk_begin.offsetBy(0,0,k);
    const uint32_t offset = nanovdb::NanoLeaf<uint32_t>::CoordToOffset(ijk);

    // get leaf containing this voxel (and skip if inactive)
    auto* leaf = acc.probeLeaf(ijk);
    if (leaf == nullptr || !leaf->isActive(offset))
      continue;
      
    // iterate through points in the voxel
    auto* p = reinterpret_cast<const nanovdb::Vec3f*>(grid.blindData(0)) + leaf->minimum();
    begin = p + (offset == 0 ? 0 : leaf->getValue(offset - 1));
    end = p + leaf->getValue(offset);
    while (begin != end)
    {
      // convert from various internal voxel coordinate frames to world frame
      const nanovdb::Vec3f idx = nanovdb::Vec3f(ijk) + *begin++;
      const nanovdb::Vec3f pt = grid.indexToWorld(idx);

      // update minimum Z tracked so far
      min_z = std::isnan(min_z) ? pt[2] : fminf(min_z, pt[2]);
      // printf("Point (%f,%f,%f)", pt[0], pt[1], pt[2]);
    }
    
    // if we've found a point then break early; no points above this voxel will be smaller, axiomatically
    if (!std::isnan(min_z))
      break;
  }

  // update array element representing the minimum Z value in a column
  if (!std::isnan(min_z))
    deviceMap[j + i * (xmax - xmin + 1)] = min_z;
}

extern "C" void launch_ground_plane_kernel(const nanovdb::GridHandle<nanovdb::CudaDeviceBuffer>& gridHandle,
                                           const openvdb::CoordBBox& bbox,
                                           float* deviceMap,
                                           cudaStream_t stream)
{
  // get a (raw) pointer to a NanoVDB grid of value type float on the GPU (uint32_t for PointDataGrid)
  auto* deviceGrid = gridHandle.deviceGrid<uint32_t>();

  // sanity check
  if (!deviceGrid || !deviceMap)
    throw std::runtime_error("Failed to load grid and / or mat to the GPU.");

  // set up GPU block / thread configuration
  auto round = [](int a, int b) { return (a + b - 1) / b; };
  constexpr dim3 threadsPerBlock(8, 8);
  const dim3 numBlocks(round(bbox.dim().x(), threadsPerBlock.x), round(bbox.dim().y(), threadsPerBlock.y));

  // kernel syntax:  <<<blocks per grid, threads per block, dynamic shared memory per block, stream >>>
  min_z_kernel<<<numBlocks, threadsPerBlock, 0, stream>>>(
    *deviceGrid, deviceMap,
    bbox.min().x(), bbox.max().x(),
    bbox.min().y(), bbox.max().y(),
    bbox.min().z(), bbox.max().z()
  );
}

} // namespace ovm::ops