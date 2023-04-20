/* aggregate_bag.cpp
 *
 * Utility script / demonstration of OVM.
 * 
 * This program stitches together sensor_msgs::PointCloud2
 * messages from the given ROSBAG into a single monolithic
 * cloud and saves it to a file.
 * 
 * Ego pose correction is done via TF2 - this thus requires
 * the bag to have a functional TF tree.
 * 
 * @TODO batch inputs to show speedup.
 * @TODO support .pcd file saving instead of / in addition to .vdb
 */

// STL
#include <optional>

// Boost
#include <boost/program_options.hpp>

// ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

// TF
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>

// OVM
#include <openvdb_voxel_mapper/ros1_voxel_cloud.h>

// convenience typedefs
namespace po = boost::program_options;

// user settable command line arguments
struct CommandLineOptions
{
  std::vector<std::string> filenames;   // bag filenames
  std::string cloud_topic {""};         // point cloud topic to assemble; if unset we'll try them all
  std::string fixed_frame {"odom"};     // frame_id in which to assemble clouds
  std::string output {"combined.vdb"};  // output filename; @TODO support .pcd instead
}; // struct CommandLineOptions

std::optional<CommandLineOptions> parse_args(int argc, char** argv)
{
  
  // set up result
  CommandLineOptions opts;

  // set up input argument options
  po::options_description description("Command line arguments:");
  description.add_options()
    ("help", "Produce help message")
    ("bag-files", po::value<std::vector<std::string>>(&opts.filenames), "All ROS Bag files associated with this run.")
    ("cloud-topic", po::value<std::string>(&opts.cloud_topic), "sensor_msgs::PointCloud2 topic to aggregate - defaults to assembling all.")
    ("fixed-frame", po::value<std::string>(&opts.fixed_frame), "Fixed frame to assemble clouds in.")
    ("output", po::value<std::string>(&opts.output), "Output filename for the combined cloud - currently in .vdb format.")
  ;

  // make bags a positional argument (last)
  po::positional_options_description positional_options;
  positional_options.add("bag-files", -1);

  // parse options and store in variable map
  po::variables_map variables_map;
  po::store(po::command_line_parser(argc, argv).options(description).positional(positional_options).run(), variables_map);
  po::notify(variables_map);
  
  // if help request, print and return
  if (variables_map.count("help"))
  {
    std::cout << description << std::endl;
    return std::nullopt;
  }
  return opts;
}

int main(int argc, char ** argv)
{
  // parse program options
  const auto opts = parse_args(argc, argv);
  if (!opts)
    return 0;

  // initialize core view object
  rosbag::View view;
  
  // collection of bag handles
  std::vector<std::unique_ptr<rosbag::Bag>> bags;
  bags.reserve(opts->filenames.size());

  // load all bags into a view
  for(const auto& bagfile : opts->filenames)
  {
    // construct bag object
    bags.emplace_back(std::make_unique<rosbag::Bag>(bagfile));
    
    // store and add to view
    view.addQuery(*(bags.back()));
  }

  // get point cloud topics and perform sanity checks
  std::vector<std::string> cloud_topics;
  for (const auto& connection : view.getConnections())
  {
    // if a specific topic is set, check for it
    if (connection->topic == opts->cloud_topic)
    {
      if (connection->datatype != "sensor_msgs/PointCloud2")
        throw std::runtime_error("Requested topic '" + opts->cloud_topic + "' is not a sensor_msgs::PointCloud2 (it's a '" + connection->datatype + "')");
      else
        cloud_topics.push_back(opts->cloud_topic);
    }
    else
    {
      // otherwise, assimilate all sensor_msgs::PointCloud2
      if (connection->datatype == "sensor_msgs/PointCloud2")
        cloud_topics.push_back(opts->cloud_topic);
    }
  }
  if (cloud_topics.size() == 0)
    throw std::runtime_error("No point cloud topics found...");

  // construct and populate a tf buffer
  // note: constructed in advance so we can interpolate odometry information as needed
  std::cout << "Collecting TF information..." << std::endl;
  tf2_ros::Buffer tfb(ros::DURATION_MAX);
  tfb.setUsingDedicatedThread(true);
  for (const auto& m : view)
    if (m.getTopic() == "/tf" || m.getTopic() == "/tf_static")
      if (const auto msg = m.instantiate<tf2_msgs::TFMessage>(); msg)
        for (const auto& transform : msg->transforms)
          tfb.setTransform(transform, "rosbag", /*is_static=*/m.getTopic() == "/tf_static");

  // construct a cloud object
  ovm::ros::ROS1VoxelCloud cloud;

  // iterate through the bag, accumulating PointClouds
  std::cout << "Merging clouds..." << std::endl;
  size_t count {0};
  for (const auto& m : view)
    if (std::find(cloud_topics.begin(), cloud_topics.end(), m.getTopic()) != cloud_topics.end())
      if (auto msg = m.instantiate<sensor_msgs::PointCloud2>(); msg)
      {
        // transform cloud
        sensor_msgs::PointCloud2 transformed_cloud;
        if (!pcl_ros::transformPointCloud(opts->fixed_frame, *msg, transformed_cloud, tfb))
        {
          std::cerr << "Failed to transform cloud from '" << msg->header.frame_id << "' to '"
                    << opts->fixed_frame << "; is that fixed frame reasonable?" << std::endl;
          continue;
        }

        // merge, if successful
        cloud.merge<pcl::PointXYZ>(transformed_cloud);
        ++count;
      }

  // save cloud to bag
  std::cout << "Writing combined cloud to " << opts->output << std::endl;
  cloud.write(opts->output);
}