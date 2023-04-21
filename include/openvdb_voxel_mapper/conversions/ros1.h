/* Conversions to and from OpenVDB types for ROS1.
 *
 */

#pragma once

// STL
#include <optional>
#include <type_traits>

// ROS1
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// GridMap
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>

// OVM
#include "../types.h"
#include "../voxel_cloud.h"
#include "pcl.h"

namespace ovm::conversions
{

// construct a ROS PointCloud2 into a VoxelCloud
template <typename PointT>
VoxelCloud from_ros(const sensor_msgs::PointCloud2& msg, const std::shared_ptr<Options>& opts)
{  
  // convert to PCL
  pcl::PointCloud<PointT> pcl_cloud;
  pcl::fromROSMsg(msg, pcl_cloud);
  
  // construct and return a new VoxelCloud
  return VoxelCloud(pcl_cloud, opts);
}

// convert a VoxelCloud into a ROS PointCloud2
std::optional<sensor_msgs::PointCloud2> to_ros(const VoxelCloud& cloud,
                                               const std::shared_ptr<Options>& opts)
{
  // convert to PCL and then to a PointCloud2
  if (auto pcl_cloud = to_pcl(cloud.grid(), opts); pcl_cloud)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*pcl_cloud, msg);
    return msg;
  }

  // unable to convert
  return std::nullopt;
}

// convert a ovm::Map to a ROS grid map
std::optional<grid_map_msgs::GridMap> to_ros(const ovm::Map& map,
                                             const std::string& layer,
                                             const std::shared_ptr<Options>& opts)
{
  // compile time sanity checks
  static_assert(std::is_same_v<grid_map::GridMap::Matrix, ovm::Map::MapT>, "Internal Map type mismatch.");

  // handle edge cases
  if (map.map.rows() == 0 || map.map.cols() == 0)
    return std::nullopt;

  // initialize a grid_map::GridMap object
  grid_map::GridMap grid;
  grid.setPosition({map.pose.x(), map.pose.y()});
  grid.add(layer, map.map);
  grid.setFrameId(opts->frame);
  // @TODO 
  // grid.setTimestamp !!!

  // convert to a grid_map_msg and return
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(grid, msg);
  return msg;
}

} // namespace ovm::conversions