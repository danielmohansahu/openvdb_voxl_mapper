/* Operations to extract label and confidence information from the grid.
 */

#pragma once

// STL
#include <optional>
#include <array>

// OVM
#include "../types.h"
#include "../voxel_cloud.h"
#include "utils.h"

// OpenVDB
#include <openvdb/points/PointAttribute.h>

namespace ovm::ops
{

// aggregate columns of voxels in the grid via logodds and return an array with class confidence per cell
std::optional<std::vector<Eigen::MatrixXf>>
semantic_projection_logodds(const ovm::VoxelCloud& cloud)
{
  // @TODO
  //  - iterate through voxels
  //  - iterate through classes in voxel
  //  - logodds aggregate class confidence of each point in the voxel to the cell at that XY index

  // I am a stub.
  return std::nullopt;
}

// return the argmax label for each cell based on their aggregate confidence
std::optional<Eigen::MatrixXi>
semantic_projection_argmax(const ovm::VoxelCloud& cloud)
{
  // @TODO:
  //  - get array of confidences per cell
  //  - get array index of top confidence per cell
  //  - map from array index to label

  if (auto confidences = semantic_projection_logodds(cloud); confidences)
  {
    // I am a stub.
    return std::nullopt;
  }

  // extraction failed
  return std::nullopt;
}

} // namespace ovm::ops