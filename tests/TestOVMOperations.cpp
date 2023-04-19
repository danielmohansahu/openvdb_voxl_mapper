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
  for (int i = -5; i != 6; ++i)
  {
    float v = static_cast<float>(i);
    pcl_cloud.push_back({0, 0, v});   // center column of points
    pcl_cloud.push_back({v, v, v});   // diagonals have cascading Z
    pcl_cloud.push_back({v, -v, v});  // diagonals have cascading Z
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
  EXPECT_TRUE(cpu_map->pose == gpu_map->pose);
  EXPECT_TRUE(cpu_map->map == gpu_map->map);
  
  // compare each to ground truth result expectation
  ovm::Map gt_map;
  gt_map.pose = ovm::Map::PoseT {-5,-5};
  gt_map.map = ovm::Map::MapT {0,0};

  EXPECT_TRUE(cpu_map->pose == gt_map.pose);
  EXPECT_TRUE(gpu_map->pose == gt_map.pose);
  EXPECT_TRUE(cpu_map->map == gt_map.map);
  EXPECT_TRUE(gpu_map->map == gt_map.map);

  std::cout << "CPU_pose: " << cpu_map->pose << std::endl;
  std::cout << "CPU_map: \n" << cpu_map->map << std::endl;
  std::cout << "GPU_pose: " << gpu_map->pose << std::endl;
  std::cout << "GPU_map: \n" << gpu_map->map << std::endl;
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