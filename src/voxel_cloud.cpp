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
#include <openvdb_voxel_mapper/operations/openvdb.h>

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

  // debugging
  const size_t size_initial = (_opts->verbose) ? this->size() : 0;

  // filter grid
  ops::drop_by_attribute_criterion<AttStampT>(_grid->tree(), ATT_STAMP, valid);
  
  if (_opts->verbose)
  {
    const float delta = size_initial - this->size();
    const auto [lower,upper] = this->time_bounds();
    printf("VoxelCloud::remove_before dropped %f points (%.2f %%)\n", delta, 100.0 * delta / size_initial);
    printf(" stamps (lower | threshold | upper): %f | %f | %f\n", lower, stamp, upper);
  }
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
