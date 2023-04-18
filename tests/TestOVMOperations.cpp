/* TestOVMOperations.cpp
 *
 * Unit tests for OVM operations.
 */

// GTest
#include <gtest/gtest.h>

// OpenVDB
#include <openvdb/openvdb.h>

// OVM
#include <openvdb_voxel_mapper/voxel_cloud.h>
#include <openvdb_voxel_mapper/operations/ground_plane_extraction.h>

// helper class to initialize common structures
class TestOVMOperations: public ::testing::Test
{
public:
  void SetUp() override { openvdb::initialize(); }
  void TearDown() override { openvdb::uninitialize(); }
}; // class TestOVMOperations

// test ground plane extraction
TEST_F(TestOVMOperations, testGroundPlane)
{
  // construct a PCL cloud with hardcoded points
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  for (int i = -5; i != 5; ++i)
  {
    float v = static_cast<float>(i);
    pcl_cloud.push_back({0, 0, v});   // center column of points
    pcl_cloud.push_back({v, v, 0});   // everywhere else has Z = 0
  }

  // set voxel size to 1.0 meter
  ovm::Options opts {};
  opts.voxel_size = 1.0;

  // construct a cloud
  ovm::VoxelCloud cloud(pcl_cloud, opts);

  // perform (CPU) ground plane extraction
  const auto cpu_map = ovm::ops::ground_plane_extraction_geometric(cloud.grid());
  ASSERT_TRUE(cpu_map);

  // perform (GPU) ground plane extraction
  const auto gpu_map = ovm::ops::ground_plane_extraction_geometric_cuda(cloud.grid());
  ASSERT_TRUE(gpu_map);

  // compare results
  EXPECT_TRUE(*cpu_map == *gpu_map);
  
  // compare each to ground truth result expectation
  ovm::Map::PoseT gt_pose {0,0};
  ovm::Map::MapT gt_map {0,0};

  EXPECT_TRUE(cpu_map->pose == gt_pose);
  EXPECT_TRUE(gpu_map->pose == gt_pose);
  EXPECT_TRUE(cpu_map->map == gt_map);
  EXPECT_TRUE(gpu_map->map == gt_map);
}

// test label extraction operation
TEST_F(TestOVMOperations, testLabelArgmax)
{
  // procedure: hardcoded PCL cloud with points in columns
  //  verify resulting map size (?) position, and all cells

  // I am a stub
  EXPECT_TRUE(false);
}

// test label confidence projection via logodds
TEST_F(TestOVMOperations, testLabelConfidenceLogodds)
{
  // procedure: hardcoded PCL cloud with points in columns
  //  verify resulting map size (?) position, and all cells

  // I am a stub
  EXPECT_TRUE(false);
}