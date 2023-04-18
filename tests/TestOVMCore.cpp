/* TestOVMCore.cpp
 *
 * Unit tests for the VoxelCloud class, which handles
 * common operations on an underlying OpenVDB grid.
 */

// STL
#include <unordered_map>
#include <limits>

// GTest
#include <gtest/gtest.h>

// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/points/PointCount.h>

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
  // construct empty cloud
  ovm::VoxelCloud cloud;
  ASSERT_EQ(cloud.size(), 0);
  
  // generate random clouds with varying timestamps
  std::unordered_map<float,size_t> num_points;
  for (size_t i = 0; i != 10; ++i)
  {
    // make a random cloud
    auto pclcloud = ovm::test::make_random_pcl_cloud();
    pclcloud.header.stamp = i * 1e3;   // cast to milliseconds per PCL convention

    // convert to voxelcloud
    ovm::VoxelCloud subcloud(pclcloud);
    const size_t N = openvdb::points::pointCount(subcloud.grid()->tree());
    ASSERT_EQ(N, pclcloud.size());

    // save this stamp's metadata
    num_points.insert({i, N});

    // merge into the main grid
    cloud.merge(subcloud);
  }

  // get starting total of points
  const size_t total_points = openvdb::points::pointCount(cloud.grid()->tree());

  // sanity check total points (really a merge test)
  {
    size_t total_points_check {0};
    for (auto& kv : num_points)
      total_points_check += kv.second;
    ASSERT_EQ(total_points, total_points_check);
  }

  // verify that all expected timestamps are in the correct range
  {
    const auto [min,max] = cloud.time_bounds();
    EXPECT_EQ(min, 0);
    EXPECT_EQ(max, 10);
    for (auto kv : num_points)
      EXPECT_TRUE(kv.second >= min && kv.second <= max);
  }

  // attempt to delete a single timestamp
  cloud.remove(1);

  // verify that point totals make sense
  {
    const size_t N = openvdb::points::pointCount(cloud.grid()->tree());   
    EXPECT_EQ(N, total_points - num_points.at(1));
  }

  // delete all timestamps before a fixed time
  cloud.remove_before(5.5);

  // verify remaining timestamps are all strictly after the threshold
  {
    const auto [min,max] = cloud.time_bounds();
    EXPECT_EQ(min, 6);
    EXPECT_EQ(max, 10);

    const size_t N = openvdb::points::pointCount(cloud.grid()->tree());   
    EXPECT_EQ(N, num_points.at(6) + num_points.at(7) + num_points.at(8) + num_points.at(9) + num_points.at(10));
  }

  // remove all points
  cloud.remove_before(std::numeric_limits<float>::max());
  EXPECT_TRUE(cloud.empty());
}