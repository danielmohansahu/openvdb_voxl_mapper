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

} // namespace ovm
