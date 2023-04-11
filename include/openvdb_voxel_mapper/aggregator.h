/* aggregator.h
 *
 * Class definition of the Aggregator class which encapsulates
 * the current cache of clouds, insertion methods, and rendering
 * options.
 */

// STL
#include <memory>

// OpenVDB
#include <openvdb/openvdb.h>

// OVM
#include "types.h"

namespace ovm
{

struct AggregatorOptions
{
  size_t points_per_voxel {8};

}; // struct AggregatorOptions

class Aggregator
{
 public:
  // expected constructor
  explicit Aggregator(const AggregatorOptions options = AggregatorOptions());

  // insert a new point cloud into the cloud
  void insert(const Cloud& cloud);

  // reset the cloud to a clean slate
  void reset();
  
  // write current cloud to file
  void write(const std::string& filename);

 private:
  // core cloud object - contains the full cloud data as one-dimensional vectors
  Cloud _cloud;

  // openvdb Transform (maps real world space (xyz) to index space (i,j,k))
  openvdb::math::Transform::Ptr _transform;

  // PointIndexGrid - core 
  openvdb::tools::PointIndexGrid::Ptr _index_grid;

  // configuration options
  const AggregatorOptions _options;

}; // class Aggregator

} // namespace ovm