/* TestOVMROS1.cpp
 *
 * Unit tests for ROS1 bindings.
 */

// GTest
#include <gtest/gtest.h>

// GridMap
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

// ROS Messages
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>

// OVM
#include <openvdb_voxel_mapper/ros1_voxel_cloud.h>
#include <openvdb_voxel_mapper/conversions/ros1.h>

// helper class to initialize common structures
class TestOVMROS1: public ::testing::Test
{
public:
  void SetUp() override { openvdb::initialize(); }
  void TearDown() override { openvdb::uninitialize(); }
}; // class TestOVMROS1

// test conversion to / from ROS message types
TEST_F(TestOVMROS1, testMapConversions)
{
  // N.B. Resolutions at discrete multiples of the map length will result in funny sizes,
  //  like extra rows of NAN or missing columns. We're avoiding this for now by just
  //  _not_ testing those cases. I'm not sure if this is a real issue or not.

  // constants
  const float resolution = 0.35;

  // first, populate a grid_map with known values in XY
  grid_map::GridMap gt_map({"elevation"});
  gt_map.setFrameId("silly");
  gt_map.setGeometry(grid_map::Length(1.25, 2.05), resolution, grid_map::Position(-75.0, 1.0));
  for (grid_map::GridMapIterator it(gt_map); !it.isPastEnd(); ++it)
  {
    grid_map::Position position;
    gt_map.getPosition(*it, position);
    gt_map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 + 5.0 * position.y()) * position.x();
  }
  
  // convert to a PointCloud
  sensor_msgs::PointCloud2 gt_cloud;
  grid_map::GridMapRosConverter::toPointCloud(gt_map, "elevation", gt_cloud);

  // perform some intermediate sanity checks (we expect one point per map cell)
  ASSERT_EQ(gt_cloud.height * gt_cloud.width, gt_map.getSize()(0) * gt_map.getSize()(1));

  // construct an OVM cloud from this Voxel Cloud
  auto opts = std::make_shared<ovm::Options>();
  opts->voxel_size = resolution;
  opts->frame = "silly";
  ovm::ros::ROS1VoxelCloud cloud(gt_cloud, opts);

  // check some basic details
  EXPECT_EQ(cloud.size(), gt_cloud.height * gt_cloud.width);

  // extract ground plane and convert to another grid_map::GridMap
  const auto map_msg = cloud.maps("elevation");
  ASSERT_TRUE(map_msg);
  grid_map::GridMap map;
  ASSERT_TRUE(grid_map::GridMapRosConverter::fromMessage(*map_msg, map));

  // print out the maps
  std::cout << "GT Map: \n" << gt_map.getPosition() << std::endl;
  std::cout << "GT Map: \n" << gt_map.get("elevation") << std::endl;
  std::cout << "OVM Map: \n" << map.getPosition() << std::endl;
  std::cout << "OVM Map: \n" << map.get("elevation") << std::endl;

  // perform validation of map metadata
  EXPECT_TRUE(map.hasSameLayers(gt_map));
  EXPECT_EQ(map.getFrameId(), gt_map.getFrameId());
  EXPECT_EQ(map.getTimestamp(), gt_map.getTimestamp());
  EXPECT_EQ(map.getResolution(), gt_map.getResolution());

  // check bounds; no point continuing if these are wrong
  ASSERT_TRUE(map.getSize().isApprox(gt_map.getSize()));

  // the "position" of a cell can be off, but it shouldn't be off my more than 1/2 the resolution
  const auto offset = map.getPosition() - gt_map.getPosition();
  ASSERT_LT(abs(offset(0)), resolution / 2.0);
  ASSERT_LT(abs(offset(1)), resolution / 2.0);

  // perform a cell-by-cell check
  for (grid_map::GridMapIterator it(gt_map); !it.isPastEnd(); ++it)
  {
    // get the position of this iterator - it should be the same
    grid_map::Position gt_pos, pos;
    gt_map.getPosition(*it, gt_pos);
    map.getPosition(*it, pos);
    EXPECT_FLOAT_EQ(gt_pos(0), pos(0) - offset(0));
    EXPECT_FLOAT_EQ(gt_pos(1), pos(1) - offset(1));

    // get the value of this cell
    EXPECT_FLOAT_EQ(gt_map.at("elevation", *it), map.at("elevation", *it));
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}