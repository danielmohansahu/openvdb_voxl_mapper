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
  explicit VoxelCloud(const std::shared_ptr<Options>& options = std::make_shared<Options>())
   : _opts(options)
  {
    initialize();
  };

  // construct from a PCL cloud
  template <typename PointT>
  VoxelCloud(const pcl::PointCloud<PointT>& pcl_cloud,
             const std::shared_ptr<Options>& options = std::make_shared<Options>())
   : _opts(options)
  {
    initialize();

    // construct grid from PCL cloud
    _opts->frame = pcl_cloud.header.frame_id;
    _grid = ovm::conversions::from_pcl(pcl_cloud, _opts);
    _grid->setName(_opts->name);
  }

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
  void remove(const AttStampT stamp);
  
  // remove all stamps prior to the given, inclusive
  void remove_before(const AttStampT stamp);

  // get min and max timestamps tracked by the cloud
  //  note : this operation requires a full traversal of the cloud
  std::pair<AttStampT,AttStampT> time_bounds() const;

  // accessor for core grid - API for operations
  GridT::Ptr grid() { return _grid; }

  // const accessor for core grid - API for operations
  const GridT::Ptr grid() const { return _grid; }

  // setter for core grid - API for operations
  void swap(GridT::Ptr& other) { _grid.swap(other); }

 private:
  // openvdb initialization; can be called multiple times
  void initialize() const
  {
    // perform OpenVDB initialization
    openvdb::initialize();

    // @TODO use different codecs for various attributes. We don't need 16
    //  bits to store the confidence [0,1.0].
  }

  // core underlying PointDataGrid
  GridT::Ptr _grid;

  // configuration options
  std::shared_ptr<Options> _opts;

}; // class VoxelCloud

} // namespace ovm
