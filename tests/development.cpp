// continually changing development test - not a real automated test

// STL
#include <iostream>

// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/util/CpuTimer.h>

// OVM
#include <openvdb_voxel_mapper/voxel_cloud.h>
#include <openvdb_voxel_mapper/operations/ground_plane.h>

// OVM Test
#include "test_utilities.h"

void print_middle_of_map(const Eigen::MatrixXf& map)
{
  std::cout << "Map's 'center' points: " << std::endl;
  std::cout << map.block<10,10>(map.rows() / 2 - 5, map.cols() / 2 - 5) << std::endl;
}

int main(int argc, char** argv)
{
  // simple timer
  openvdb::util::CpuTimer timer;

  // generate random clouds outside of the merge timing loop
  timer.start("Generating random clouds");
  std::vector<pcl::PointCloud<pcl::PointXYZ>> random_clouds;
  for (size_t i = 0; i != 100; ++i)
    random_clouds.emplace_back(ovm::test::make_random_pcl_cloud(200000, 1.0 * i, 40.0, 0.5 * i, 40.0, 0.0, 10.0));

  // construct an empty cloud
  ovm::VoxelCloud cloud;

  // add random clouds. each iteration _roughly_ correspond to 1 frame of a 128X1024 beam
  //  LIDAR with 120m max range traveling at ~10m/s, for a total of 10 seconds of data
  timer.restart("Merging clouds into grid");
  for (auto& subcloud : random_clouds)
    cloud.merge(subcloud);

  // extract ground plane from grid (naive single threaded)
  timer.restart("Extracting ground plane via CPU");
  const auto cpu_map = ovm::ops::min_z_ground_plane(cloud.grid());
  timer.stop();
  if (cpu_map)
    print_middle_of_map(*cpu_map);
  else
    std::cerr << "FAILED TO CONSTRUCT CPU MAP!!!" << std::endl;

  // extract ground plane from grid (GPU)
  timer.start("Extracting ground plane via GPU");
  const auto gpu_map = ovm::ops::min_z_ground_plane_cuda(cloud.grid());
  timer.stop();
  if (gpu_map)
    print_middle_of_map(*gpu_map);
  else
    std::cerr << "FAILED TO CONSTRUCT MAP!!!" << std::endl;

  // dump cloud to file
  cloud.write("development.vdb");
  std::cout << "Wrote aggregate cloud to development.vdb." << std::endl;
}
