/* Conversions to and from OpenVDB types for ROS1.
 *
 */

#pragma once

// STL
#include <optional>

// ROS1
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

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
std::optional<grid_map_msgs::GridMap> to_ros(const ovm::Map& map, const std::string& layer)
{
  // I am a stub.
  return std::nullopt;
}

} // namespace ovm::conversions