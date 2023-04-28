/* Utility functions used throughout unit tests.
 *
 */

#pragma once

// STL
#include <string>

// PCL
#include <pcl/common/generate.h>
#include <pcl/common/io.h>

// OVM
#include <openvdb_voxel_mapper/voxel_cloud.h>

namespace ovm::test
{

// default number of points to generate for a cloud
#define DEFAULT_NUM_POINTS 10000

// semi-arbitrary custom point type for testing
struct MyPointWithXYZLC
{
  // data types
  float x {0};
  float y {0};
  float z {0};
  int label {0};
  float confidence {0};

  // constructors
  MyPointWithXYZLC() = default;
  MyPointWithXYZLC(float x_, float y_, float z_, int l = 0, float c = 0)
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

// construct a PCL Uniform Distribution random point generator
template <typename PointT>
auto uniform_generator(const float xmin = -100.0, const float xmax = 100.0,
                       const float ymin = -100.0, const float ymax = 100.0,
                       const float zmin = -100.0, const float zmax = 100.0)
{
  auto seed = static_cast<std::uint32_t> (time (nullptr));
  pcl::common::CloudGenerator<PointT, pcl::common::UniformGenerator<float>> gen;
  gen.setParametersForX ({xmin, xmax, seed++});
  gen.setParametersForY ({ymin, ymax, seed++});
  gen.setParametersForZ ({zmin, zmax, seed++});
  return gen;
}

// construct a PCL Normal Distribution random point generator
template <typename PointT>
auto normal_generator(const float xmean = 0.0, const float xstddev = 25.0,
                      const float ymean = 0.0, const float ystddev = 25.0,
                      const float zmean = 0.0, const float zstddev = 25.0)
{
  auto seed = static_cast<std::uint32_t> (time (nullptr));
  pcl::common::CloudGenerator<PointT, pcl::common::NormalGenerator<float>> gen;
  gen.setParametersForX ({xmean, xstddev, seed++});
  gen.setParametersForY ({ymean, ystddev, seed++});
  gen.setParametersForZ ({zmean, zstddev, seed++});
  return gen;
}

// construct a random PCL cloud
template <typename PointT = pcl::PointXYZ>
pcl::PointCloud<PointT> make_random_pcl_cloud(const size_t N = DEFAULT_NUM_POINTS,
                                              const float xmean = 0.0, const float xstddev = 25.0,
                                              const float ymean = 0.0, const float ystddev = 25.0,
                                              const float zmean = 0.0, const float zstddev = 25.0)
{
  // generate a random PCL cloud
  pcl::PointCloud<PointT> cloud;
  auto generator = normal_generator<PointT>(xmean, xstddev, ymean, ystddev, zmean, zstddev);
  for (size_t i = 0; i != N; ++i)
    cloud.emplace_back(generator.get());
  
  return cloud;
}

// construct a random VoxelCloud
template <typename PointT = pcl::PointXYZ>
ovm::VoxelCloud make_random_cloud(const std::string filename = "",
                                  const size_t N = DEFAULT_NUM_POINTS,
                                  const float xmean = 0.0, const float xstddev = 25.0,
                                  const float ymean = 0.0, const float ystddev = 25.0,
                                  const float zmean = 0.0, const float zstddev = 25.0)
{
  // construct random pcl cloud
  const auto cloud = make_random_pcl_cloud<PointT>(N, xmean, xstddev, ymean, ystddev, zmean, zstddev);
  
  // convert from a PCL cloud to a VoxelCloud
  ovm::VoxelCloud result {cloud};

  // save, if given a filename
  if (filename != "")
    openvdb::io::File(filename).write({result.grid()});

  // return full cloud
  return result;
}

// compare eigen matrices with NANs
inline bool equal(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b, const float eps = std::numeric_limits<float>::epsilon())
{
  // set all NANS to Zero
  return a.array().isNaN().select(0,a).isApprox(b.array().isNaN().select(0,b), eps);
}

} // namespace ovm::test