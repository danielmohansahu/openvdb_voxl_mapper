/* Simple ground plane extraction operation.
 *
 * Ground plane extraction operations.
 */

// STL
#include <iostream>

// NanoVDB
#include <nanovdb/NanoVDB.h>

// OVM
#include <openvdb_voxel_mapper/types.h>

namespace ovm::ops
{

extern "C" void launch_ground_plane_kernel(const nanovdb::NanoGrid<uint32_t>*,
                                           const nanovdb::NanoGrid<uint32_t>*,
                                           cudaStream_t stream,
                                           ovm::Map::MapT& result)
{
  // @TODO!
}

} // namespace ovm::ops