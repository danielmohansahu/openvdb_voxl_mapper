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
  // @TODO make this more efficent

  using namespace openvdb::points;

  // handle edge cases
  if (cloud.empty())                          // no data
    return std::nullopt;
  if (cloud.options()->labels.size() == 0)     // no labels
    return std::nullopt;

  // @TODO check if confidence attribute exists!

  // access grid and options
  const auto& grid = cloud.grid();
  const auto& opts = cloud.options();

  // useful named constants
  const size_t num_labels = opts->labels.size();

  // initialize output map dimensions and pose from the grid's bounding box
  std::vector<Eigen::MatrixXf> result; result.reserve(num_labels);
  const auto bbox = grid->evalActiveVoxelBoundingBox();
  for (size_t i = 0; i != num_labels; ++i)
    result.emplace_back(Eigen::MatrixXf::Constant(bbox.dim().y(), bbox.dim().x(), NAN));

  // Iterate over all the leaf nodes in the grid.
  for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf)
  {
    // Create read-only AttributeHandles to access the XYZ position and confidence data per point
    const AttributeHandle<AttConfidenceT> handle(leaf->constAttributeArray(ATT_CONFIDENCE));

    // Iterate over the point indices in the leaf.
    for (auto idx = leaf->beginIndexOn(); idx; ++idx)
    {
      // get the index of this voxel in the corresponding grid
      const auto [row, col] = idx_to_rc(idx.getCoord().x(), idx.getCoord().y(), bbox.min().x(), bbox.max().y());

      // iterate through all the label confidences, updating the underlying grid
      for (size_t i = 0; i != num_labels; ++i)
        if (const AttConfidenceT confidence = handle.get(*idx, i); !std::isnan(confidence))
        {
          // @TODO logodds!
          // get current and new confidence value
          auto& val = result[i].coeffRef(row, col);
          val = std::isnan(val) ? confidence : std::max(val, confidence);
        }
    }
  }

  // return suite of updated maps
  return result;
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