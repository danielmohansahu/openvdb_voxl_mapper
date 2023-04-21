/* types.h
 *
 * Commonly used type definitions.
 */

#pragma once

// Eigen
#include <Eigen/Core>

// OpenVDB
#include <openvdb/math/Coord.h>
#include <openvdb/math/Transform.h>

namespace ovm
{

// supported attribute strings and typedefs
typedef openvdb::Vec3f  AttPositionT;     // type of position information
typedef int             AttLabelT;        // type of label information
typedef float           AttConfidenceT;   // type of confidence information
typedef double          AttStampT;        // type of stamp information
#define ATT_POSITION    "P"               // name of position information
#define ATT_LABEL       "label"           // name of label information
#define ATT_CONFIDENCE  "confidence"      // name of confidence information
#define ATT_STAMP       "stamp"           // name of stamp information

// project configuration options
struct Options
{
  float voxel_size {0.5};                   // voxel size, in meters
  AttConfidenceT default_confidence {1.0};  // confidence, if unsupplied
  AttLabelT default_label {-1};             // label, if unsupplied
  AttLabelT free_label {0};                 // label for free space, e.g. ray trace cleared
  std::vector<AttLabelT> ground_labels {};  // labels considered 'ground' for operations
  std::string name {"OVM Grid"};            // string identifier for the grid
  std::string frame {""};                   // coordinate frame in which we're assembling
  bool verbose {false};                     // operate in verbose logging mode
}; // struct Options

// Axis-aligned map structure used as the final output of Mapping.
struct Map
{
  // typedefs
  typedef Eigen::MatrixXf MapT;
  typedef Eigen::Vector2f PoseT;
  
  MapT map;   // map of arbitrary float values
  PoseT pose; // XY origin of the map

  // convenience equality operator
  friend inline bool operator==(const Map& lhs, const Map& rhs)
  {
    return (lhs.pose == rhs.pose) && (lhs.map == rhs.map);
  }

  // convenience constructor from an openvdb bounding box
  Map(const openvdb::CoordBBox& bbox, const openvdb::math::Transform& tf)
  {
    // get dimensions and origin of grid bounding box
    const auto dimensions = bbox.dim();
    const auto origin = tf.indexToWorld(bbox.min());
    map = MapT::Constant(dimensions.y(), dimensions.x(), NAN);
    pose = PoseT {origin.x(), origin.y()};
  }

  // maintain default constructor
  Map() = default;
}; // struct Map

} // namespace ovm
