/* aggregator.cpp
 *
 * Implementation of the Aggregator class.
 */

// STL
#include <iostream>
#include <assert.h>

// OpenVDB
#include <openvdb/points/PointConversion.h>

// OVM
#include <openvdb_voxel_mapper/aggregator.h>

namespace ovm
{

Aggregator::Aggregator(const AggregatorOptions options)
 : _options(options)
{
  // initialize OpenVDB
  openvdb::initialize();

  // reset to a clean state
  reset();
}

void Aggregator::insert(const Cloud& cloud)
{
  // @TODO
  //  - sanity check input cloud
  //  - do this more efficiently; currently just Proof of Concept

  // convenience namespace access
  using namespace openvdb::math;
  using namespace openvdb::tools;
  using namespace openvdb::points;

  // concatenate clouds
  _cloud.concatenate(cloud);

  // update class variables
  std::cout << "Performing lazy initialization of Aggregator." << std::endl;
  
  // calculate the (very rough) voxel size
  const auto voxel_size = computeVoxelSize(_cloud, _options.points_per_voxel);
  std::cout << "  voxel size: " << voxel_size << std::endl;
  
  // construct transform (mapping from world space (xyz) to index space (ijk))
  _transform = Transform::createLinearTransform(voxel_size);

  // instantiate index grid (supports determining index of point associated with real-world xyz location)
  _index_grid = createPointIndexGrid<PointIndexGrid>(_cloud, *_transform);
  _index_grid->setName("AggregatePoints");
}

void Aggregator::reset()
{
  // reset class variables to a clean state
  _transform = nullptr;
  _index_grid = nullptr;
  _cloud = Cloud();
}

void Aggregator::write(const std::string& filename) const
{
  using namespace openvdb::tools;
  using namespace openvdb::points;

  // Create a VDB file object and write out the grid.
  if (_index_grid)
  {
    // although our internal representation is a mapping from VOXEL -> index,
    //  we convert to a full PointDataGrid when saving.
    PointDataGrid::Ptr grid = createPointDataGrid<NullCodec, PointDataGrid>(
      *_index_grid, _cloud, _index_grid->transform());

    // Set the name of the grid and save
    grid->setName("AggregatePoints");
    openvdb::io::File(filename).write({grid});
  }
}

} // namespace ovm