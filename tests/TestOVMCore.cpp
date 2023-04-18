/* TestOVMCore.cpp
 *
 * Unit tests for the VoxelCloud class, which handles
 * common operations on an underlying OpenVDB grid.
 */

// GTest
#include <gtest/gtest.h>

// OpenVDB
#include <openvdb/openvdb.h>

// OVM
#include <openvdb_voxel_mapper/voxel_cloud.h>

// OVM Test
#include "test_utilities.h"

class TestVoxelCloud: public ::testing::Test
{
public:
  void SetUp() override { openvdb::initialize(); }
  void TearDown() override { openvdb::uninitialize(); }
}; // class TestVoxelCloud

// test basic construction of VoxelCloud
TEST_F(TestVoxelCloud, testConstruction)
{
  // default (empty) constructor should result in an empty cloud
  ovm::VoxelCloud empty_cloud {};
  EXPECT_TRUE(empty_cloud.empty());

  // construction from a point cloud
  const size_t num_points {100};
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  for (size_t i = 0; i != num_points; ++i)
    pcl_cloud.emplace_back(i, 2.0 * i, 3.0 * i);
  ovm::VoxelCloud normal_cloud(pcl_cloud);

  // sanity check cloud status
  ASSERT_FALSE(pcl_cloud.empty());
  EXPECT_EQ(pcl_cloud.size(), num_points);

  // verify reset functionality
  normal_cloud.reset();
  EXPECT_TRUE(normal_cloud.empty());
}

// test merge of multiple point clouds
TEST_F(TestVoxelCloud, testMerge)
{
  // construct an empty cloud
  ovm::VoxelCloud cloud {};

  // generate and add random clouds
  const size_t num_points = 1000;
  const size_t num_clouds = 10;
  for (size_t i = 0; i != num_clouds; ++i)
  {
    auto subcloud = ovm::test::make_random_cloud("", num_points);
    cloud.merge(subcloud);
  }

  // verify full cloud has expected number of points
  EXPECT_EQ(cloud.size(), num_clouds * num_points);
}

// test subset deletion
TEST_F(TestVoxelCloud, testDeletion)
{
  // first thing to do : make a map of {stamp : num_points}

  // construct a cloud and merge in several different timestamps
  
  
  // delete a single timestamp

  // verify that timestamp is no longer present


  // delete all timestamps before a fixed time

  // verify remaining timestamps all exist 

  EXPECT_TRUE(false);
}