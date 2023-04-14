/* VoxelCloud.hpp
 *
 * Template implementation of VoxelCloud
 */

// STL
#include <assert.h>

// OpenVDB
#include <openvdb/points/PointAttribute.h>
#include <openvdb/points/PointConversion.h>

// OVM
#include <openvdb_voxel_mapper/openvdb/PointMerge.h>

namespace ovm
{

VoxelCloud::VoxelCloud(const pcl::PointCloud<PointT>& pcl_cloud)
{
  // perform OpenVDB initialization
  openvdb::initialize();
  
  // construct grid from PCL cloud
  _grid = ovm::conversions::from_pcl(pcl_cloud);
  assert(_grid);
  _grid->setName("OVM Grid");
}

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

void VoxelCloud::merge(VoxelCloud&& other)
{
  // handle edge cases
  if (other.empty())
  {
    // not super useful, but thanks? I guess?
    return;
  }
  if (this->empty())
  {
    // we're empty; just use the other directly
    *this = std::move(other);
    return;
  }

  // both VoxelClouds have contents - merge 'other' into ourselves
  openvdb::points::mergePoints(*_grid, *(other.grid()));
  
  // reset other to make sure nobody uses it
  other.reset();  
}

} // namespace ovm
