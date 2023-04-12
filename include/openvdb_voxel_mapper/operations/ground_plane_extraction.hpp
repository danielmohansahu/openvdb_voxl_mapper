/* Simple ground plane extraction operation.
 *
 * Ground plane extraction operations.
 */

#pragma once

// STL
#include <memory>
#include <optional>

// OVM
#include "../types.h"

namespace ovm::operations
{

template <typename PointT>
std::optional<Map> ground_plane_extraction_geometric(
  const openvdb::tools::PointIndexGrid::Ptr& grid,
  const Cloud<PointT>& cloud)
{
  // sanity check inputs
  if (!grid || cloud.size() == 0)
    return std::nullopt;

  // extract transform used by the grid and use it for our map
  const auto& transform = grid->transform();

  // initalize output map based on the XY positions of the given grid
  const auto dimensions = grid->evalActiveVoxelDim();
  ovm::Map result = ovm::Map::Zero(dimensions.x(), dimensions.y());

  // iterate through leaf nodes
  for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf)
  {
    // iterate over indices of all points in this leaf node
    for (auto it = leaf->cbeginValueOn(); it; ++it)
    {
      // @TODO
      // // Retrieve point at this index
      // const auto& pt = cloud.getPoint(idx);

      // // Verify the index, world-space position and radius of the point.
      // std::cout << "* PointIndex=[" << idx << "] ";
      // std::cout << "WorldPosition=" << pt.xyz << " ";
      // std::cout << "Label=" << pt.label << " ";
      // std::cout << "Confidence=" << pt.confidence << std::endl;
    }
  }

  return result;
}


} // namespace ovm::operations