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
#include <openvdb_voxel_mapper/conversions/pcl.h>

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
    const auto opts = std::make_shared<ovm::Options>();
    const auto grid = from_pcl(original_cloud, opts);

    // convert back to pcl cloud
    const auto new_cloud = to_pcl(grid, opts);
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

  // convenience equality operators
  bool operator==(const MyPointWithXYZLC& other) const
  {
    const float eps = std::numeric_limits<float>::epsilon();
    return (fabs(x - other.x) < eps) &&
           (fabs(y - other.y) < eps) &&
           (fabs(z - other.z) < eps) &&
           (fabs(confidence - other.confidence) < eps) &&
           (label == other.label);
  }

  bool operator==(const pcl::PointXYZ& other) const
  {
    const float eps = std::numeric_limits<float>::epsilon();
    return (fabs(x - other.x) < eps) &&
           (fabs(y - other.y) < eps) &&
           (fabs(z - other.z) < eps);
  }
}; // struct MyPointWithXYZLC

// test arbitrary point structures
TEST_F(TestOVMConversions, testCustomPoint)
{
  TestPCL<MyPointWithXYZLC>();
}

// expected failures for certain cloud / label / confidence configuration
TEST_F(TestOVMConversions, testLabelFailures)
{
  // construct a ground truth PCL cloud with known labels
  pcl::PointCloud<MyPointWithXYZLC> gt_cloud;
  gt_cloud.emplace_back(0.0f, 0.0f, 0.0f, -1, 0.0);
  gt_cloud.emplace_back(1.0f, 1.0f, 1.0f, 0, 0.35);
  gt_cloud.emplace_back(2.0f, 2.0f, 2.0f, 1, 0.55);
  gt_cloud.emplace_back(3.0f, 3.0f, 3.0f, 195, 1.0);

  // construct default options
  auto gt_opts = std::make_shared<ovm::Options>();
  gt_opts->labels = std::vector<int>{-1, 0, 1, 195};

  {
    // we should fail to construct a cloud with missing labels
  }

  {
    // we should fail to construct a cloud with confidences outside [0,1.0]
  }

  {
    // if we drop label support we can't convert back to a full resolution cloud
  }

  {
    // if our cloud doesn't have confidences we should end up with all default confidences
  }

  EXPECT_TRUE(false);
}

// test constructing clouds with semantic information and verify that information gets retained
TEST_F(TestOVMConversions, testSemantics)
{
  // construct a ground truth PCL cloud with known labels
  pcl::PointCloud<MyPointWithXYZLC> gt_cloud;
  gt_cloud.emplace_back(0.0f, 0.0f, 0.0f, -1, 0.0);
  gt_cloud.emplace_back(1.0f, 1.0f, 1.0f, 0, 0.35);
  gt_cloud.emplace_back(2.0f, 2.0f, 2.0f, 1, 0.55);
  gt_cloud.emplace_back(3.0f, 3.0f, 3.0f, 195, 1.0);

  // set up options with valid labels
  auto opts = std::make_shared<ovm::Options>();
  opts->labels = std::vector<int>{-1, 0, 1, 195};

  // convert to voxel cloud
  ovm::VoxelCloud ovm_cloud(gt_cloud, opts);

  // convert back to the same desired point type
  const auto cloud = ovm::conversions::to_pcl<MyPointWithXYZLC>(ovm_cloud.grid(), ovm_cloud.options());
  ASSERT_TRUE(cloud);

  // iterate through the cloud, making sure we find an equivalent point
  EXPECT_EQ(cloud->size(), gt_cloud.size());
  for (const auto pt : *cloud)
  {
    bool found = false;
    for (const auto gt_pt : gt_cloud)
      if (pt == gt_pt)
        found = true;

    // if we get here and  this point isn't found we have a problem.
    EXPECT_TRUE(found);
  }
  
  // there should also be no problem converting to a simpler type
  const auto xyz_cloud = ovm::conversions::to_pcl<pcl::PointXYZ>(ovm_cloud.grid(), ovm_cloud.options());
  ASSERT_TRUE(xyz_cloud);

  // iterate through the cloud, making sure we find an equivalent point
  EXPECT_EQ(xyz_cloud->size(), gt_cloud.size());
  for (const auto pt : *xyz_cloud)
  {
    bool found = false;
    for (const auto gt_pt : gt_cloud)
      if (gt_pt == pt)
        found = true;

    // if we get here and  this point isn't found we have a problem.
    EXPECT_TRUE(found);
  }
}