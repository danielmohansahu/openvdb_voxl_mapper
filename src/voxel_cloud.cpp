/* VoxelCloud.hpp
 *
 * Template implementation of VoxelCloud
 */

// STL
#include <assert.h>
#include <limits>

// OpenVDB
#include <openvdb/points/PointCount.h>
#include <openvdb/points/PointStatistics.h>

// OVM
#include <openvdb_voxel_mapper/voxel_cloud.h>
#include <openvdb_voxel_mapper/operations/openvdb_operations.h>

namespace ovm
{

bool VoxelCloud::empty() const
{
  return (_grid) ? _grid->empty() : true;
}
  
size_t VoxelCloud::size() const
{
  return (_grid) ? openvdb::points::pointCount(_grid->tree()) : 0;
}

void VoxelCloud::reset()
{
  _grid.reset();
}

void VoxelCloud::write(const std::string& filename) const
{
  if (!_grid)
    return;

  std::cout << "Writing current grid to " << filename << " ..." << std::endl;
  try
  {
    openvdb::io::File(filename).write({_grid});
  }
  catch (...)
  {
    std::cerr << "Failed to write grid to " << filename << std::endl;
  }
}

void VoxelCloud::merge(const VoxelCloud& other)
{
  // handle edge cases
  if (other.empty())
    // not super useful, but thanks? I guess?
    return;
  else if (this->empty())
    // we're empty; just use the other directly
    _grid = other.grid()->deepCopy();
  else
    // both VoxelClouds have contents - merge 'other' into ourselves
    openvdb::points::mergePoints(*_grid, *(other.grid()));
}

void VoxelCloud::remove(const AttStampT stamp)
{
  // handle edge cases
  if (this->empty())
    return;

  // construct evaluation criteria (TRUE == keep the associated point)
  auto valid = [stamp] (const AttStampT& val) -> bool { return val != stamp; };

  // filter grid
  ops::drop_by_attribute_criterion<AttStampT>(_grid->tree(), ATT_STAMP, valid);
}

void VoxelCloud::remove_before(const AttStampT stamp)
{
  // handle edge cases
  if (this->empty())
    return;

  // construct evaluation criteria (TRUE == keep the associated point)
  auto valid = [stamp] (const AttStampT& val) -> bool { return val > stamp; };

  // filter grid
  ops::drop_by_attribute_criterion<AttStampT>(_grid->tree(), ATT_STAMP, valid);
}

std::pair<AttStampT,AttStampT> VoxelCloud::time_bounds() const
{
  // handle edge cases
  if (this->empty())
    return {0,0};

  // initialize results
  AttStampT lower {std::numeric_limits<AttStampT>::max()};
  AttStampT upper {std::numeric_limits<AttStampT>::min()};

  // get min / max values
  if (openvdb::points::evalMinMax(_grid->tree(), ATT_STAMP, lower, upper))
    return {lower, upper};

  // failed to process for some reason
  throw std::runtime_error("Failed to extract time bounds. Unexpected error.");
}

} // namespace ovm
