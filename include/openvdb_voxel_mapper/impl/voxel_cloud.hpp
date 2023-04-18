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

template <typename PointT>
VoxelCloud::VoxelCloud(const pcl::PointCloud<PointT>& pcl_cloud, const Options& options)
 : _opts(options)
{
  // perform OpenVDB initialization
  openvdb::initialize();
  
  // construct grid from PCL cloud
  _grid = ovm::conversions::from_pcl(pcl_cloud, _opts);
  assert(_grid);
  _grid->setName("OVM Grid");
}

} // namespace ovm
