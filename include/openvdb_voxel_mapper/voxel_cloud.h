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

// OVM
#include "types.h"
#include "conversions/pcl.h"

namespace ovm
{

class VoxelCloud
{
 public:
  // convenience typedefs
  using GridT = openvdb::points::PointDataGrid;

 public:
  // no empty constructor allowed
  explicit VoxelCloud(const Options& options = Options()) : _opts(options) {};

  // construct from a PCL cloud
  template <typename PointT>
  explicit VoxelCloud(const pcl::PointCloud<PointT>& pcl_cloud, const Options& options = Options());

  // return whether or not we have data
  bool empty() const;
  
  // return number of available points - this can be expensive.
  size_t size() const;

  // clear and reset to a blank slate
  void reset();

  // write current cloud to file
  void write(const std::string& filename) const;

  // merge in another point data grid, consuming it in the process
  void merge(const VoxelCloud& other);

  // remove a single timestamp from the cloud
  void remove(const float stamp);
  
  // remove all stamps prior to the given, inclusive
  void remove_before(const float stamp);

  // get min and max timestamps tracked by the cloud
  //  note : this operation requires a full traversal of the cloud
  std::pair<float,float> time_bounds() const;

  // accessor for core grid - API for operations
  GridT::Ptr grid() { return _grid; }

  // const accessor for core grid - API for operations
  const GridT::Ptr grid() const { return _grid; }

  // setter for core grid - API for operations
  void swap(GridT::Ptr& other) { _grid.swap(other); }

 private:
  // core underlying PointDataGrid
  GridT::Ptr _grid;

  // configuration options
  Options _opts {};

}; // class VoxelCloud

} // namespace ovm

// implementation
#include "impl/voxel_cloud.hpp"
