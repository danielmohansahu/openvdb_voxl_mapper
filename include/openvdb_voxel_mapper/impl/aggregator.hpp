/* aggregator.cpp
 *
 * Templatee implementation of the Aggregator class.
 */

// STL
#include <iostream>
#include <assert.h>

// OpenVDB
#include <openvdb/points/PointConversion.h>

// OVM
#include <openvdb_voxel_mapper/operations/ground_plane_extraction.hpp>

namespace ovm
{

template <typename PointT>
Aggregator<PointT>::Aggregator(const AggregatorOptions options)
 : _options(options)
{
  // initialize OpenVDB
  openvdb::initialize();

  // reset to a clean state
  reset();
}

template <typename PointT>
void Aggregator<PointT>::insert(const Cloud<PointT>& cloud)
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
  
  // construct transform (mapping from world space (xyz) to index space (ijk))
  _transform = Transform::createLinearTransform(_options.voxel_size);

  // instantiate index grid (supports determining index of point associated with real-world xyz location)
  _index_grid = createPointIndexGrid<PointIndexGrid>(_cloud, *_transform);
  _index_grid->setName("AggregatePoints");
}

template <typename PointT>
void Aggregator<PointT>::reset()
{
  // reset class variables to a clean state
  _transform.reset();
  _index_grid.reset();
  _cloud = Cloud<PointT>();
}

template <typename PointT>
void Aggregator<PointT>::write(const std::string& filename) const
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

    // test 
    operations::ground_plane_extraction_geometric(_index_grid, _cloud);
  }
}

} // namespace ovm