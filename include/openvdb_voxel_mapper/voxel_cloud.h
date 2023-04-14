/* VoxelCloud.h
 *
 * Class definition for VoxelCloud, a wrapper around an OpenVDB
 * PointDataGrid which abstracts some common operations like
 * merging.
 */

#pragma once

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// OpenVDB
#include <openvdb/openvdb.h>

namespace ovm
{

class VoxelCloud
{
 public:
  // convenience typedefs
  using GridT   = openvdb::points::PointDataGrid;
  using PointT  = pcl::PointXYZ;

 public:
  // empty constructor
  VoxelCloud() = default;

  // construct from a PCL cloud
  explicit VoxelCloud(const pcl::PointCloud<PointT>& pcl_cloud);

  // return whether or not we have data
  bool empty() const;
  
  // return number of available points - this can be expensive.
  size_t size() const;

  // clear and reset to a blank slate
  void reset();

  // write current cloud to file
  void write(const std::string& filename) const;

  // merge in another point data grid, consuming it in the process
  void merge(VoxelCloud&& other);

  // accessor for core grid - API for operations
  GridT::Ptr grid() { return _grid; }

  // setter for core grid - API for operations
  void swap(GridT::Ptr& other) { _grid.swap(other); }

 private:
  // core underlying PointDataGrid
  GridT::Ptr _grid;
}; // class VoxelCloud

} // namespace ovm

// implementation
#include "impl/voxel_cloud.hpp"
