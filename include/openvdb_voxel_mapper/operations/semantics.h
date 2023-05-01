/* Operations to extract label and confidence information from the grid.
 *
 * @TODO there are a lot of opportunities to speed this implementation up.
 */

#pragma once

// STL
#include <optional>
#include <array>
#include <algorithm>

// OVM
#include "../types.h"
#include "../voxel_cloud.h"
#include "utils.h"

// OpenVDB
#include <openvdb/points/PointAttribute.h>

namespace ovm::ops
{

// log odds conversion from confidence (which we're treating as probability...)
static inline float logodds(float p)
{
  // clamp inputs to avoid numerical problems
  p = std::clamp(p, 1e-5f, 1.0f - 1e-5f);
  return std::log(p / (1 - p));
}

// conversion from log odds to probability (which we're treating as confidence...)
static inline float invlogodds(const float lo)
{
  const auto odds = std::exp(lo);
  return odds / (1 + odds);
}

// aggregate columns of voxels in the grid via logodds and return an array with class confidence per cell
std::optional<std::vector<Eigen::MatrixXf>>
semantic_projection_logodds(const ovm::VoxelCloud& cloud)
{
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
          // get current and new confidence value in log-odds
          const auto conf = logodds(confidence);
          auto& val = result[i].coeffRef(row, col);
          val = std::isnan(val) ? conf : (conf + val);
        }
    }
  }

  // aggregation and projection was done in log-odds; reverse before returning
  for (auto& mat : result)
    mat = mat.array().isNaN().select(mat, mat.unaryExpr(&invlogodds));

  // return suite of updated maps
  return result;
}

// return the argmax label for each cell based on their aggregate confidence
std::optional<Eigen::MatrixXi>
semantic_projection_argmax(const ovm::VoxelCloud& cloud)
{
  // get a vector of confidences associated with each class for each 2D cell of the Voxel Grid
  if (auto confidences = semantic_projection_logodds(cloud); (confidences && confidences->size() != 0))
  {
    // supported labels and dimensions
    const auto& opts = cloud.options();
    const size_t rows = (*confidences)[0].rows();
    const size_t cols = (*confidences)[0].cols();

    // sanity checks
    assert(opts->labels.size() == confidences->size());

    // initialize comparison objects - index of top confidence and top confidence
    Eigen::MatrixXf top_conf = Eigen::MatrixXf::Constant(rows, cols, -1.0f);
    Eigen::MatrixXi top_idx  = Eigen::MatrixXi::Constant(rows, cols, -1);

    // iterate through each label's confidence per-cell, updating max
    for (size_t i = 0; i != confidences->size(); ++i)
    {
      // access this label's confidence map
      const auto& label_conf = (*confidences)[i];

      // mark the label index for these changed cells
      const Eigen::MatrixXi constant = Eigen::MatrixXi::Constant(rows, cols, i);
      top_idx = (label_conf.array() > top_conf.array()).select(constant, top_idx);

      // update confidences (this should handle NaN cells correctly)
      top_conf = (label_conf.array() > top_conf.array()).select(label_conf, top_conf);
      assert(top_conf.array().isNaN().count() == 0);
    }
  
    // update result to return the actual labels, not their indices
    const Eigen::MatrixXi unknown_idx   = Eigen::MatrixXi::Constant(rows, cols, -1);
    const Eigen::MatrixXi unknown_label = Eigen::MatrixXi::Constant(rows, cols, opts->unknown);
    const auto labelmap = [labels = opts->labels] (const int idx) -> int { return labels[idx]; };
    return (top_idx.array() == unknown_idx.array()).select(unknown_label, top_idx.unaryExpr(labelmap));
  }

  // extraction failed
  return std::nullopt;
}

} // namespace ovm::ops