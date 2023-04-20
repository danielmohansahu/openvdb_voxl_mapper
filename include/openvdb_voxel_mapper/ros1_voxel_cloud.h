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
  explicit ROS1VoxelCloud(const Options& options = Options())
   : _opts(options), _cloud(options) {}

  // merge an incoming sensor_msgs::PointCloud2 cloud
  template <typename PointT = pcl::PointXYZ>
  void merge(const sensor_msgs::PointCloud2& msg) { _cloud.merge(conversions::from_ros<PointT>(msg, _opts)); }
  
  // convert current full cloud to a sensor_msgs::PointCloud2
  std::optional<sensor_msgs::PointCloud2> cloud(const std::string& frame) const { return conversions::to_ros(_cloud, frame); }

  // extract the ground plane of the current cloud
  std::optional<grid_map_msgs::GridMap> ground_plane(const bool gpu = true) const
  {
    // perform ground plane extraction via specified method
    if (auto map = (gpu) ? ops::ground_plane_extraction_geometric_cuda(_cloud.grid())
                         : ops::ground_plane_extraction_geometric(_cloud.grid())
        ; map)
      // conversion succeeded - convert to grid map
      return conversions::to_ros(*map, "ground");
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
  void remove(const float stamp) { _cloud.remove(stamp); };
  
  // PASSTHROUGH API: remove all stamps prior to the given, inclusive
  void remove_before(const float stamp) { _cloud.remove_before(stamp); };

 private:
  // configuration options
  // @TODO make shared with underlying cloud
  Options _opts;
 
  // core underlying voxel cloud (encapsulation > inheritance)
  VoxelCloud _cloud;
}; // class ROS1VoxelCloud

} // namespace ovm::ros