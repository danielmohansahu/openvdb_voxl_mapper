/* TestOVMROS1.cpp
 *
 * Unit tests for ROS1 bindings.
 */

// GTest
#include <gtest/gtest.h>

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

// test ROS1 wrapper class 
TEST_F(TestOVMROS1, testROSVoxelCloud)
{
  // I am a stub
  EXPECT_TRUE(false);
}

// test conversion to / from ROS message types
TEST_F(TestOVMROS1, testConversions)
{
  // procedure: hardcoded PCL cloud with points in columns
  //  verify resulting map size (?) position, and all cells

  // I am a stub
  EXPECT_TRUE(false);
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}