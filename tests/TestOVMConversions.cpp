/* TestOVMConversions.cpp
 *
 * Unit tests for OVM conversions.
 * 
 * Future improvements:
 *  - Implement a proper test for distance between PCL PointClouds (like ICP)
 *  - Improve random generation of arbitrary attributes
 *  - Test expected attributes pre / post conversion
 */

// GTest
#include <gtest/gtest.h>

// OpenVDB
#include <openvdb/openvdb.h>

// OVM
#include <openvdb_voxel_mapper/voxel_cloud.h>
#include <openvdb_voxel_mapper/conversions.h>

// OVM Test
#include "test_utilities.h"

// helper class to initialize common structures
class TestOVMConversions: public ::testing::Test
{
 public:
  void SetUp() override { openvdb::initialize(); }
  void TearDown() override { openvdb::uninitialize(); }

  // test conversion to / from a PCL point  
  template <typename PointT>
  void TestPCL()
  {
    using namespace ovm::test;
    using namespace ovm::conversions;
    
    // create a random PCL cloud
    const auto original_cloud = make_random_pcl_cloud<PointT>();

    // convert to OpenVDB grid
    const auto grid = from_pcl(original_cloud);

    // convert back to pcl cloud
    const auto new_cloud = to_pcl(grid);
    ASSERT_TRUE(new_cloud);

    // verify all points are the same
    ASSERT_EQ(original_cloud.size(), new_cloud->size());

    // @TODO check each point? Use ICP and measure difference?
  }
}; // class TestOVMConversions

// test conversion to / from a pcl::PointCloud<pcl::PointXYZ>
TEST_F(TestOVMConversions, testPCLPointXYZ)
{
  TestPCL<pcl::PointXYZ>();
}

// test conversion to / from a pcl::PointCloud<pcl::PointXYZL>
TEST_F(TestOVMConversions, testPCLPointXYZL)
{
  TestPCL<pcl::PointXYZL>();
}

// test conversion to / from a pcl::PointCloud<pcl::PointXYZRGBL>
TEST_F(TestOVMConversions, testPCLPointXYZRGBL)
{
  TestPCL<pcl::PointXYZRGBL>();
}

// semi-arbitrary custom point type
struct MyPointWithXYZLC
{
  // data types
  float x {0};
  float y {0};
  float z {0};
  size_t label {0};
  float confidence {0};

  // constructors
  MyPointWithXYZLC() = default;
  MyPointWithXYZLC(float x_, float y_, float z_, size_t l = 0, float c = 0)
   : x(x_), y(y_), z(z_), label(l), confidence(c) {}
}; // struct MyPointWithXYZLC

// test arbitrary point structures
TEST_F(TestOVMConversions, testCustomPoint)
{
  TestPCL<MyPointWithXYZLC>();
}