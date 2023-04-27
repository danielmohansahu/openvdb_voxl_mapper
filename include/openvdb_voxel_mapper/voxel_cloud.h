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
  VoxelCloud(const std::shared_ptr<const Options>& options = std::make_shared<Options>())
   : _opts(options)
  {
    initialize();
  };

  // construct from a PCL cloud
  template <typename PointT>
  VoxelCloud(const pcl::PointCloud<PointT>& pcl_cloud,
             const std::shared_ptr<const Options>& options = std::make_shared<Options>())
   : _opts(options)
  {
    initialize();
    merge(pcl_cloud);
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
  template <typename PointT>
  void merge(const pcl::PointCloud<PointT>& pcl_cloud)
  {
    if (this->empty())
    {
      // initialize from this cloud
      _grid = ovm::conversions::from_pcl(pcl_cloud, _opts);

      // set some metadata; this is mostly just for debugging.
      _grid->setName(_opts->name);
      _grid->insertMeta("pcl_frame", openvdb::StringMetadata(pcl_cloud.header.frame_id));
    }
    else
    {
      // sanity checks
      assert(_grid);
      assert(pcl_cloud.header.frame_id == _opts->frame);

      // merge
      openvdb::points::mergePoints(*_grid, *ovm::conversions::from_pcl(pcl_cloud, _opts));
    }
  }

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
  
  // const accessor for options
  const std::shared_ptr<const Options> options() const { return _opts; }

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
  const std::shared_ptr<const Options> _opts;

}; // class VoxelCloud

} // namespace ovm
