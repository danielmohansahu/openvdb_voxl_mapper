/* VoxelCloud.hpp
 *
 * Template implementation of VoxelCloud
 */

// OpenVDB
#include <openvdb/points/PointAttribute.h>
#include <openvdb/points/PointConversion.h>

// OVM
#include <openvdb_voxel_mapper/openvdb/PointMerge.h>

namespace ovm
{

VoxelCloud::VoxelCloud(const pcl::PointCloud<PointT>& pcl_cloud)
{
  using namespace openvdb::points;
  using namespace openvdb::math;
  using namespace openvdb::tools;
  // @TODO move this to a separate utility function and add conditional
  //  attribute support

  // convert from cloud object to a series of vectors (XYZ, Attributes)
  std::vector<openvdb::Vec3H> positions;
  positions.reserve(pcl_cloud.size());
  for (const auto& pt : pcl_cloud)
    positions.emplace_back(pt.x, pt.y, pt.z);

  PointAttributeVector<openvdb::Vec3H> positionsWrapper(positions);
  // @TODO breakout voxel size to be configurable
  Transform::Ptr transform = Transform::createLinearTransform(0.5);
  PointIndexGrid::Ptr pointIndexGrid = createPointIndexGrid<PointIndexGrid>(positionsWrapper, *transform);
  _grid = createPointDataGrid<NullCodec, PointDataGrid>(*pointIndexGrid, positionsWrapper, *transform);
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
