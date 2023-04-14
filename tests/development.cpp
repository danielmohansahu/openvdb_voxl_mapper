// continually changing development test - not a real automated test

// OpenVDB
#include <openvdb/openvdb.h>

// OVM
#include <openvdb_voxel_mapper/voxel_cloud.h>
#include <openvdb_voxel_mapper/operations/ground_plane_extraction.hpp>

// OVM Test
#include "test_utilities.h"

int main(int argc, char** argv)
{
  // construct an empty cloud
  ovm::VoxelCloud cloud;

  // generate and add random clouds
  // each iteration _roughly_ correspond to 1 frame of a 128X1024 beam
  //  LIDAR with 120m max range traveling at ~10m/s, for a total of 10 seconds of data
  std::cout << "Merging clouds into grid..." << std::endl;
  for (size_t i = 0; i != 100; ++i)
  {
    const std::string filename = "raw_" + std::to_string(i) + ".vdb";
    cloud.merge(ovm_test::make_random_cloud(filename, 200000, 1.0 * i, 40.0, 0.5 * i, 40.0, 0.0, 10.0));
  }
  std::cout << "Generated " << cloud.size() << " points." << std::endl;

  // extract ground plane from grid
  std::cout << "Extracting ground plane..." << std::endl;
  const auto map = ovm::ops::ground_plane_extraction_geometric(cloud.grid());
  assert(map);

  // print out the middle of the map
  std::cout << "Ground plane of 'center' points: " << std::endl;
  std::cout << map->map.block<10,10>(map->map.rows() / 2 - 5, map->map.cols() / 2 - 5) << std::endl;

  // dump cloud to file
  cloud.write("development.vdb");
  std::cout << "Wrote aggregate cloud to development.vdb." << std::endl;
}