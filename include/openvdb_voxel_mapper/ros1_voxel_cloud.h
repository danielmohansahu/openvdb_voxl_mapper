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
#include "operations/ground_plane.h"
#include "operations/semantics.h"
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
  std::optional<grid_map_msgs::GridMap> maps(const std::string& ground_layer = "elevation",
                                             const std::string& label_layer  = "label",
                                             const std::vector<std::string>& labels = std::vector<std::string>(),
                                             const bool gpu = false) const
  {
    // sanity check inputs
    if (labels.size() != 0 && labels.size() != _opts->labels.size())
      throw std::runtime_error("Invalid number of labels received.");

    // construct resulting map
    const auto [lower, stamp] = time_bounds();
    const auto bbox = _cloud.grid()->transform().indexToWorld(_cloud.grid()->evalActiveVoxelBoundingBox());
    auto grid = conversions::to_ros(_opts, bbox.getCenter(), bbox.extents().x(), bbox.extents().y(), stamp);
    
    // check if conversion succeeded
    if (!grid)
      return std::nullopt;

    // attempt to add ground plane
    if (auto ground = (gpu) ? ops::min_z_ground_plane_cuda(_cloud.grid()) : ops::min_z_ground_plane(_cloud.grid()) ; ground)
      conversions::add_layer(*grid, *ground, ground_layer);

    // attempt to add label layer
    if (auto label = ops::semantic_projection_argmax(_cloud); label)
      conversions::add_layer(*grid, label->cast<float>(), label_layer);

    //  attempt to add per-label confidences
    // @TODO!

    // convert to a grid_map_msg and return
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(*grid, msg);
    return msg;
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
  const std::shared_ptr<Options> _opts;
 
  // core underlying voxel cloud (encapsulation > inheritance)
  VoxelCloud _cloud;
}; // class ROS1VoxelCloud

} // namespace ovm::ros