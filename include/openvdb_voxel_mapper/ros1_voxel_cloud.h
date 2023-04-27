/* ros1_voxel_cloud.h
 *
 * ROS1 wrapper for OVM::VoxelCloud.
 * 
 * This class provides API methods for VoxelCloud
 * allowing for direct interaction to / from ROS
 * message types.
 */

// ROS1
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>

// OVM
#include "types.h"
#include "voxel_cloud.h"
#include "conversions/ros1.h"
#include "operations/ground_plane_extraction.h"

namespace ovm::ros
{

class ROS1VoxelCloud
{
 public:
  // expected constructor
  explicit ROS1VoxelCloud(const std::shared_ptr<Options>& options = std::make_shared<Options>())
   : _opts(options), _cloud(options) {}

  // construct from a sensor_msgs::PointCloud2
  template <typename PointT = pcl::PointXYZ>
  ROS1VoxelCloud(const sensor_msgs::PointCloud2& msg, const std::shared_ptr<Options>& options = std::make_shared<Options>())
   : _opts(options), _cloud(options)
  {
    merge(msg);
  }
  
  // merge an incoming sensor_msgs::PointCloud2 cloud
  template <typename PointT = pcl::PointXYZ>
  void merge(const sensor_msgs::PointCloud2& msg)
  {
    // convert to PCL
    pcl::PointCloud<PointT> pcl_cloud;
    pcl::fromROSMsg(msg, pcl_cloud);

    // perform merge
    _cloud.merge(pcl_cloud);
  }
  
  // convert current full cloud to a sensor_msgs::PointCloud2
  std::optional<sensor_msgs::PointCloud2> cloud(const std::string& frame) const
  {
    return conversions::to_ros(_cloud, _opts);
  }

  // extract the ground plane of the current cloud
  std::optional<grid_map_msgs::GridMap> ground_plane(const bool gpu = false,
                                                     const std::string& layer_name = "elevation") const
  {
    // get timestamp of result
    // perform ground plane extraction via specified method
    if (auto map = (gpu) ? ops::ground_plane_extraction_geometric_cuda(_cloud.grid())
                         : ops::ground_plane_extraction_geometric(_cloud.grid())
        ; map)
    {
      // extraction succeeded - convert to grid map
      const auto [lower, stamp] = time_bounds();
      const auto center = _cloud.grid()->transform().indexToWorld(_cloud.grid()->evalActiveVoxelBoundingBox().getCenter());
      return conversions::to_ros(*map, _opts, center, layer_name, stamp);
    }
    // extraction failed
    return std::nullopt;
  }

 public:
  // pass-through API

  // PASSTHROUGH API: return whether or not we have data
  bool empty() const { return _cloud.empty(); };
  
  // PASSTHROUGH API: return number of available points - this can be expensive.
  size_t size() const { return _cloud.size(); };

  // PASSTHROUGH API: clear and reset to a blank slate
  void reset() { _cloud.reset(); };

  // PASSTHROUGH API: write current cloud to file
  void write(const std::string& filename) const { _cloud.write(filename); };

  // PASSTHROUGH API: remove a single timestamp from the cloud
  void remove(const AttStampT stamp) { _cloud.remove(stamp); };
  
  // PASSTHROUGH API: remove all stamps prior to the given, inclusive
  void remove_before(const AttStampT stamp) { _cloud.remove_before(stamp); };

  // PASSTHROUGH API: return the (lower,upper) timestamp bounds of the grid
  std::pair<AttStampT,AttStampT> time_bounds() const { return _cloud.time_bounds(); };

 protected:
  // configuration options
  std::shared_ptr<Options> _opts;
 
  // core underlying voxel cloud (encapsulation > inheritance)
  VoxelCloud _cloud;
}; // class ROS1VoxelCloud

} // namespace ovm::ros