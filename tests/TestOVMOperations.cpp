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
#include <openvdb_voxel_mapper/operations/ground_plane.h>

// OVM Test
#include "test_utilities.h"

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
  for (int x = -5; x != 6; ++x)
    for (int y = -3; y != 4; ++y)
      for (int z = 1; z != 10; ++z)
      {
        // the value of the point is (x + y) * z
        float v = static_cast<float>((x + y) * z);
        pcl_cloud.push_back({static_cast<float>(x), static_cast<float>(y), v});
      }

  // set up ground truth results we expect
  Eigen::MatrixXf gt_map ;
  gt_map = Eigen::MatrixXf::Zero(13, 21);
  gt_map << -18, NAN, -9, NAN, 0, NAN, 1, NAN, 2, NAN, 3, NAN, 4, NAN, 5, NAN, 6, NAN, 7, NAN, 8,
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN,
            -27, NAN, -18, NAN, -9, NAN, 0, NAN, 1, NAN, 2, NAN, 3, NAN, 4, NAN, 5, NAN, 6, NAN, 7,
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN,
            -36, NAN, -27, NAN, -18, NAN, -9, NAN, 0, NAN, 1, NAN, 2, NAN, 3, NAN, 4, NAN, 5, NAN, 6,
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN,
            -45, NAN, -36, NAN, -27, NAN, -18, NAN, -9, NAN, 0, NAN, 1, NAN, 2, NAN, 3, NAN, 4, NAN, 5,
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN,
            -54, NAN, -45, NAN, -36, NAN, -27, NAN, -18, NAN, -9, NAN, 0, NAN, 1, NAN, 2, NAN, 3, NAN, 4,
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN,
            -63, NAN, -54, NAN, -45, NAN, -36, NAN, -27, NAN, -18, NAN, -9, NAN, 0, NAN, 1, NAN, 2, NAN, 3,
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN,
            -72, NAN, -63, NAN, -54, NAN, -45, NAN, -36, NAN, -27, NAN, -18, NAN, -9, NAN, 0, NAN, 1, NAN, 2;

  // set voxel size to 1.0 meter
  auto opts = std::make_shared<ovm::Options>();
  opts->voxel_size = 0.5;

  // construct a cloud
  ovm::VoxelCloud cloud(pcl_cloud, opts);

  // perform (CPU) ground plane extraction
  const auto cpu_map = ovm::ops::min_z_ground_plane(cloud.grid());
  ASSERT_TRUE(cpu_map);

  // perform (GPU) ground plane extraction
  const auto gpu_map = ovm::ops::min_z_ground_plane_cuda(cloud.grid());
  ASSERT_TRUE(gpu_map);

  // compare results between maps
  EXPECT_TRUE(ovm::test::equal(*cpu_map, *gpu_map));
  
  // compare with ground truth
  EXPECT_TRUE(ovm::test::equal(*cpu_map, gt_map));
  EXPECT_TRUE(ovm::test::equal(*gpu_map, gt_map));
  
  std::cout << "GT_map: \n" << gt_map << std::endl;
  std::cout << "CPU_map: \n" << *cpu_map << std::endl;
  std::cout << "GPU_map: \n" << *gpu_map << std::endl;
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