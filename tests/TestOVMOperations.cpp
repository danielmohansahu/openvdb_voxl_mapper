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
#include <openvdb_voxel_mapper/operations/semantics.h>

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

// test label confidence projection via logodds
TEST_F(TestOVMOperations, testLogoddsConfidence)
{
  // set up options with valid labels
  auto opts = std::make_shared<ovm::Options>();
  opts->voxel_size = 1.0;
  opts->labels = std::vector<int>{opts->unknown, 0, 1, 195};

  // construct a ground truth PCL cloud with known labels and confidences
  pcl::PointCloud<ovm::test::MyPointWithXYZLC> gt_cloud;
  // a column with empty confidences
  gt_cloud.emplace_back(0.0f, 0.0f, -1.0f, opts->unknown, 0.1);
  gt_cloud.emplace_back(0.0f, 0.0f,  0.3f, 0, 0.0);
  gt_cloud.emplace_back(0.0f, 0.0f, 10.0f, 1, 0.0);
  gt_cloud.emplace_back(0.0f, 0.0f, 11.0f, 195, 0.0);

  // a column with great confidences
  gt_cloud.emplace_back(1.0f, 1.0f, 1.0f, 0, 1.0);
  gt_cloud.emplace_back(1.0f, 1.0f, 2.0f, 1, 1.0);
  gt_cloud.emplace_back(1.0f, 1.0f, 4.0f, 195, 1.0);

  // a column with varying confidences of the same type
  gt_cloud.emplace_back(2.0f, 2.0f, -1.0f, 195, 0.8);
  gt_cloud.emplace_back(2.0f, 2.0f, -2.0f, 195, 0.3);
  gt_cloud.emplace_back(2.0f, 2.0f, -4.0f, 195, 0.9);

  // define ground truth maps
  std::vector<Eigen::MatrixXf> gt_confidences;
  {
    // label opts->unknown
    Eigen::MatrixXf first(3,3);
    first << NAN, NAN, NAN,
             NAN, NAN, NAN,
             0.1, NAN, NAN;
    gt_confidences.emplace_back(first);

    // label 0
    Eigen::MatrixXf second(3,3);
    second << NAN, NAN, NAN,
              NAN, 1.0, NAN,
              0.0, NAN, NAN;
    gt_confidences.emplace_back(second);

    // label 1
    Eigen::MatrixXf third(3,3);
    third << NAN, NAN, NAN,
             NAN, 1.0, NAN,
             0.0, NAN, NAN;
    gt_confidences.emplace_back(third);

    // label 195
    Eigen::MatrixXf fourth(3,3);
    fourth << NAN, NAN, 0.9391,
              NAN, 1.0, NAN,
              0.0, NAN, NAN;
    gt_confidences.emplace_back(fourth);
  }

  // our ground truth label map is just 1D
  //  note that there's no guarantee for an argmax which will get picked
  Eigen::MatrixXi gt_labels(3,3);
  gt_labels << opts->unknown, opts->unknown, 195,
               opts->unknown, 0            , opts->unknown,
               opts->unknown, opts->unknown, opts->unknown;

  // convert to voxel cloud
  ovm::VoxelCloud cloud(gt_cloud, opts);

  // perform logodds aggregation
  const auto confidence_maps = ovm::ops::semantic_projection_logodds(cloud);
  ASSERT_TRUE(confidence_maps);
  ASSERT_EQ(confidence_maps->size(), gt_confidences.size());
  
  // compare maps to our expected ground truth
  for (size_t i = 0; i != confidence_maps->size(); ++i)
  {
    // compare this label's confidence map to our expected ground truth
    const auto gt = gt_confidences[i];
    const auto map = (*confidence_maps)[i];

    std::cout << "comparing (ground truth): \n";
    std::cout << gt << std::endl;
    std::cout << "to resulting map: \n";
    std::cout << map << std::endl;

    ASSERT_EQ(gt.rows(), map.rows());
    ASSERT_EQ(gt.cols(), map.cols());
    EXPECT_TRUE(ovm::test::equal(gt, map, 1e-4f));
  }

  // perform end-to-end argmax operation, which just returns the top label for each cell
  const auto label_map = ovm::ops::semantic_projection_argmax(cloud);
  ASSERT_TRUE(label_map);

  // compare maps to our expected ground truth
  std::cout << "comparing labels (ground truth): \n";
  std::cout << gt_labels << std::endl;
  std::cout << "to resulting label map: \n";
  std::cout << *label_map << std::endl;
  ASSERT_EQ(label_map->cols(), gt_labels.cols());
  ASSERT_EQ(label_map->rows(), gt_labels.rows());
  EXPECT_TRUE(label_map->isApprox(gt_labels));
}