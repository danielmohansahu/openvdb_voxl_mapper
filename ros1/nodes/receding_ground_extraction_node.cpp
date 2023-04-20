/* receding_ground_extraction_node.cpp
 *
 * ROS1 Node which subscribes to a PC topic and extracts / 
 * publishes the ground plane for the last N seconds of data.
 * 
 * Ego pose correction is done via TF2 - this thus requires
 * the bag to have a functional TF tree.
 */

// STL
#include <optional>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_msgs/GridMap.h>

// TF
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>

// OVM
#include "../../include/openvdb_voxel_mapper/ros1_voxel_cloud.h"

int main(int argc, char ** argv)
{
  // ROS initialization
  ros::init(argc, argv, "receding_ground_extraction");
  ros::NodeHandle nh, pnh("~");

  // configuration - required params
  std::string cloud_topic, fixed_frame;
  if (!pnh.getParam("cloud_topic", cloud_topic))
    throw std::runtime_error("Missing required ros param 'cloud_topic'.");
  if (!pnh.getParam("fixed_frame", fixed_frame))
    throw std::runtime_error("Missing required ros param 'fixed_frame'.");

  // configuration - optional params
  ovm::Options opts;
  const std::string map_topic = pnh.param("map_topic", std::string("map"));
  const double horizon = pnh.param("horizon", 10.0);
  opts.voxel_size = pnh.param("voxel_size", 0.5);

  // initialize tf listener / buffer
  std::shared_ptr<tf2_ros::Buffer> tfb;
  tf2_ros::TransformListener tfl {*tfb};

  // initialize mapper
  ovm::ros::ROS1VoxelCloud mapper {opts};
  
  // set up ros publisher
  ros::Publisher pub = nh.advertise<grid_map_msgs::GridMap>(map_topic, 1);

  // set up subscriber and callback
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
    cloud_topic, 1,
    [&, tfb, pub, mapper, horizon, fixed_frame] (const auto& msg) mutable -> void
    {
      // core ROS callback
      ROS_DEBUG("Processing incoming PointCloud...");

      // prune current cloud to remove too-old timestamps
      // "premature optimization" vs. "avoiding premature pessimization"
      ROS_DEBUG("Removing old data...");
      mapper.remove_before(msg->header.stamp.toSec() - horizon);

      // try to transform cloud into fixed frame and merge
      if (sensor_msgs::PointCloud2 cloud;
          pcl_ros::transformPointCloud(fixed_frame, *msg, cloud, *tfb))
      {
        // merge in latest cloud
        ROS_DEBUG("Merging transformed cloud...");
        mapper.merge(cloud);
      }

      // extract ground plane
      // note: we still do this even if we didn't merge incoming data
      ROS_DEBUG("Extracting ground plane...");
      if (auto map = mapper.ground_plane(); map)
        pub.publish(*map);
      else
        ROS_ERROR("Failed to extract ground plane.");

      ROS_DEBUG("Finished processing PointCloud.");
    }
  );

  // spin until shutdown
  ros::spin();
}