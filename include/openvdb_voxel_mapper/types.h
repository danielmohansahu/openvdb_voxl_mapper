/* types.h
 *
 * Commonly used type definitions.
 */

#pragma once

#include <Eigen/Core>

namespace ovm
{

// Axis-aligned map structure used as the final output of Mapping.
struct Map
{
  // typedefs
  typedef Eigen::MatrixXf MapT;
  typedef Eigen::Vector2f PoseT;
  
  MapT map;   // map of arbitrary float values
  PoseT pose; // XY origin of the map
}; // struct Map

} // namespace ovm