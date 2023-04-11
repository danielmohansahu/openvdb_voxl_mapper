// continually changing development test - not a real automated test

// OpenVDB
#include <openvdb/openvdb.h>

// OVM
#include <openvdb_voxel_mapper/aggregator.h>
#include "test_utilities.h"

int main(int argc, char** argv)
{
  openvdb::initialize();

  // construct an aggregator
  ovm::AggregatorOptions opts;
  ovm::Aggregator agg {opts};

  // generate and add random clouds
  for (size_t i = 0; i != 10; ++i)
    // note that we save each raw cloud, for verification
    agg.insert(ovm_test::make_random_cloud("raw_" + std::to_string(i) + ".vdb"));

  // dump cloud to file
  agg.write("development.vdb");
}