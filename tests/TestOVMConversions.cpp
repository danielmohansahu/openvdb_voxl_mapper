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

// test arbitrary point structures
TEST_F(TestOVMConversions, testCustomPoint)
{
  TestPCL<ovm::test::MyPointWithXYZLC>();
}

// expected failures for certain cloud / label / confidence configuration
TEST_F(TestOVMConversions, testLabelFailures)
{
  // construct a ground truth PCL cloud with known labels
  pcl::PointCloud<ovm::test::MyPointWithXYZLC> gt_cloud;
  gt_cloud.emplace_back(0.0f, 0.0f, 0.0f, -1, 0.0);
  gt_cloud.emplace_back(1.0f, 1.0f, 1.0f, 0, 0.35);
  gt_cloud.emplace_back(2.0f, 2.0f, 2.0f, 1, 0.55);
  gt_cloud.emplace_back(3.0f, 3.0f, 3.0f, 195, 1.0);

  // construct default options
  auto gt_opts = std::make_shared<ovm::Options>();
  gt_opts->labels = std::vector<int>{-1, 0, 1, 195};

#ifndef NDEBUG
  {
    // we should fail to construct a cloud with missing labels
    auto bad_opts = std::make_shared<ovm::Options>();
    bad_opts->labels = std::vector<int>{0, 1, 195};
    EXPECT_DEATH(ovm::VoxelCloud(gt_cloud, bad_opts), "Assertion");
  }
#endif // NDEBUG

#ifndef NDEBUG
  {
    // we should fail to construct a cloud with confidences outside [0,1.0]
    pcl::PointCloud<ovm::test::MyPointWithXYZLC> bad_cloud;
    pcl::copyPointCloud(gt_cloud, bad_cloud);
    bad_cloud.emplace_back(4.0f, 4.0f, 4.0f, 195, 1.5);
    EXPECT_DEATH(ovm::VoxelCloud(bad_cloud, gt_opts), "Assertion");
  }
#endif // NDEBUG

  {
    // if we drop label support we can't convert back to a full resolution cloud
    auto simple_opts = std::make_shared<ovm::Options>();
    simple_opts->labels.clear();
    ovm::VoxelCloud simple_cloud(gt_cloud, simple_opts);

    // we should fail to convert this back to the original type, because we dropped the labels / confidences
    EXPECT_THROW(ovm::conversions::to_pcl<ovm::test::MyPointWithXYZLC>(simple_cloud.grid(), simple_cloud.options()), std::runtime_error);

    // but converting to a simpler point should succeed
    ovm::conversions::to_pcl<pcl::PointXYZ>(simple_cloud.grid(), simple_cloud.options());
  }
}

// test constructing clouds with semantic information and verify that information gets retained
TEST_F(TestOVMConversions, testSemantics)
{
  // set up options with valid labels
  auto opts = std::make_shared<ovm::Options>();
  opts->labels = std::vector<int>{-1, 0, 1, 195};

  // construct a ground truth PCL cloud with known labels
  pcl::PointCloud<ovm::test::MyPointWithXYZLC> gt_cloud;
  gt_cloud.emplace_back(0.0f, 0.0f, 0.0f, opts->unknown, 0.0);  // anything with 0 confidence will end up as unknown
  gt_cloud.emplace_back(1.0f, 1.0f, 1.0f, 0, 0.35);
  gt_cloud.emplace_back(1.0f, 1.0f, 1.0f, 1, 0.75);
  gt_cloud.emplace_back(2.0f, 2.0f, 2.0f, 1, 0.55);
  gt_cloud.emplace_back(3.0f, 3.0f, 3.0f, 195, 1.0);

  // convert to voxel cloud
  ovm::VoxelCloud ovm_cloud(gt_cloud, opts);

  // convert back to the same desired point type
  const auto cloud = ovm::conversions::to_pcl<ovm::test::MyPointWithXYZLC>(ovm_cloud.grid(), ovm_cloud.options());
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