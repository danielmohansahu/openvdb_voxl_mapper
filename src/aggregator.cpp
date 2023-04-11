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
  const PointAttributeVector position_wrapper(_cloud.xyz);
  const auto voxel_size = computeVoxelSize(position_wrapper, _options.points_per_voxel);
  std::cout << "  voxel size: " << voxel_size << std::endl;
  
  // construct transform (mapping from world space (xyz) to index space (ijk))
  _transform = Transform::createLinearTransform(voxel_size);

  // instantiate index grid (supports determining index of point associated with real-world xyz location)
  _index_grid = createPointIndexGrid<PointIndexGrid>(position_wrapper, *_transform);
  _index_grid->setName("AggregatePoints");
}

void Aggregator::reset()
{
  // reset class variables to a clean state
  _transform = nullptr;
  _index_grid = nullptr;
  _cloud = Cloud();
}

void Aggregator::write(const std::string& filename)
{
  using namespace openvdb::tools;
  using namespace openvdb::points;

  // Create a VDB file object and write out the grid.
  if (_index_grid)
  {
    // Create a PointDataGrid containing these four points and using the point index grid.
    PointDataGrid::Ptr grid = createPointDataGrid<NullCodec, PointDataGrid>(
      *_index_grid, PointAttributeVector(_cloud.xyz), _index_grid->transform());

    // Set the name of the grid
    grid->setName("AggregatePoints");

    // Append a "confidence" attribute to the grid to hold the confidence.
    // This attribute storage uses a unit range codec to reduce the memory
    // storage requirements down from 4-bytes to just 1-byte per value. This is
    // only possible because accuracy of the radius is not that important to us
    // and the values are always within unit range (0.0 => 1.0).
    appendAttribute(grid->tree(), "confidence", TypedAttributeArray<float>::attributeType());

    // Populate the "confidence" attribute on the points
    populateAttribute<PointDataTree, PointIndexTree, PointAttributeVector<float>>(
      grid->tree(), _index_grid->tree(), "confidence", PointAttributeVector<float>(_cloud.confidences));

    // Append a "label" attribute to the grid to hold the label.
    appendAttribute(grid->tree(), "label", TypedAttributeArray<int>::attributeType());

    // Populate the "confidence" attribute on the points
    populateAttribute<PointDataTree, PointIndexTree, PointAttributeVector<int>>(
      grid->tree(), _index_grid->tree(), "label", PointAttributeVector<int>(_cloud.labels));

    // although our internal representation is a mapping from VOXEL -> index,
    //  we convert to a full PointDataGrid when saving.
    openvdb::io::File(filename).write({grid});
  }
}

} // namespace ovm