/* TestOVMConversions.cpp
 *
 * Unit tests for OVM conversions.
 */

// GTest
#include <gtest/gtest.h>

// OpenVDB
#include <openvdb/openvdb.h>

// OVM
#include <openvdb_voxel_mapper/voxel_cloud.h>
#include <openvdb_voxel_mapper/conversions.h>

// helper class to initialize common structures
class TestOVMConversions: public ::testing::Test
{
public:
  void SetUp() override { openvdb::initialize(); }
  void TearDown() override { openvdb::uninitialize(); }
}; // class TestOVMConversions

// test conversion to / from a pcl::PointCloud<pcl::PointXYZ>
TEST_F(TestOVMConversions, testPCLPointXYZ)
{
  // I am a stub.
  EXPECT_TRUE(false);
}

// test conversion to / from a pcl::PointCloud<pcl::PointXYZL>
TEST_F(TestOVMConversions, testPCLPointXYZL)
{
  // I am a stub.
  EXPECT_TRUE(false);
}

// test conversion to / from a pcl::PointCloud<pcl::PointXYZRGBL>
TEST_F(TestOVMConversions, testPCLPointXYZRGBL)
{
  // I am a stub.
  EXPECT_TRUE(false);
}