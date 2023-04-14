/* Utility functions used throughout unit tests.
 *
 */

// STL
#include <string>

// PCL
#include <pcl/common/generate.h>

// OVM
#include <openvdb_voxel_mapper/voxel_cloud.h>

namespace ovm_test
{

// default number of points to generate for a cloud
#define DEFAULT_NUM_POINTS 10000

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

// construct a random cloud, evenly sampled across a sphere
ovm::VoxelCloud make_random_cloud(const std::string filename = "",
                                  const size_t N = DEFAULT_NUM_POINTS,
                                  const float xmean = 0.0, const float xstddev = 25.0,
                                  const float ymean = 0.0, const float ystddev = 25.0,
                                  const float zmean = 0.0, const float zstddev = 25.0)
{
  // generate a random PCL cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  auto generator = normal_generator<pcl::PointXYZ>(xmean, xstddev, ymean, ystddev, zmean, zstddev);
  for (size_t i = 0; i != N; ++i)
    cloud.emplace_back(generator.get());

  // convert from a PCL cloud to a VoxelCloud
  ovm::VoxelCloud result {cloud};

  // save, if given a filename
  if (filename != "")
    openvdb::io::File(filename).write({result.grid()});

  // return full cloud
  return result;
}

} // namespace ovm_test