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
  std::cout << "Merging clouds into grid..." << std::endl;
  for (size_t i = 0; i != 100; ++i)
  {
    const std::string filename = "raw_" + std::to_string(i) + ".vdb";
    cloud.merge(ovm_test::make_random_cloud(filename, 10000, 0.0, 2 * i, 0.0, 2 * i, 10.0 * i, 5.0));
  }

  // extract ground plane from grid
  std::cout << "Extracting ground plane..." << std::endl;
  const auto map = ovm::ops::ground_plane_extraction_geometric(cloud.grid());
  assert(map);

  // print out the middle of the map
  std::cout << "Ground plane of center points: " << std::endl;
  std::cout << map->map.block<20,20>(map->map.rows() / 2 - 10, map->map.cols() / 2 - 10) << std::endl;

  // dump cloud to file
  cloud.write("development.vdb");
  std::cout << "Wrote aggregate cloud to development.vdb." << std::endl;
}