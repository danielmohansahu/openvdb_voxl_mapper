// continually changing development test - not a real automated test

// STL
#include <iostream>

// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/util/CpuTimer.h>

// OVM
#include <openvdb_voxel_mapper/voxel_cloud.h>
#include <openvdb_voxel_mapper/operations/ground_plane_extraction.h>

// OVM Test
#include "test_utilities.h"

void print_middle_of_map(const ovm::Map::MapT& map)
{
  std::cout << "Map's 'center' points: " << std::endl;
  std::cout << map.block<10,10>(map.rows() / 2 - 5, map.cols() / 2 - 5) << std::endl;
}

int main(int argc, char** argv)
{
  // simple timer
  openvdb::util::CpuTimer timer;

  // construct an empty cloud
  ovm::VoxelCloud cloud;

  // generate and add random clouds
  // each iteration _roughly_ correspond to 1 frame of a 128X1024 beam
  //  LIDAR with 120m max range traveling at ~10m/s, for a total of 10 seconds of data
  timer.start("Merging clouds into grid");
  for (size_t i = 0; i != 100; ++i)
  {
    const std::string filename = "raw_" + std::to_string(i) + ".vdb";
    cloud.merge(ovm::test::make_random_cloud(filename, 200000, 1.0 * i, 40.0, 0.5 * i, 40.0, 0.0, 10.0));
  }

  // extract ground plane from grid (naive single threaded)
  timer.restart("Extracting ground plane via CPU");
  const auto cpu_map = ovm::ops::ground_plane_extraction_geometric(cloud.grid());
  timer.stop();
  if (cpu_map)
    print_middle_of_map(cpu_map->map);
  else
    std::cerr << "FAILED TO CONSTRUCT CPU MAP!!!" << std::endl;

  // extract ground plane from grid (GPU)
  timer.start("Extracting ground plane via GPU");
  const auto gpu_map = ovm::ops::ground_plane_extraction_geometric_cuda(cloud.grid());
  timer.stop();
  if (gpu_map)
    print_middle_of_map(gpu_map->map);
  else
    std::cerr << "FAILED TO CONSTRUCT MAP!!!" << std::endl;

  // dump cloud to file
  cloud.write("development.vdb");
  std::cout << "Wrote aggregate cloud to development.vdb." << std::endl;
}
