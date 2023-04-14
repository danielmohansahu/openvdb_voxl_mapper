/* Simple ground plane extraction operation.
 *
 * Ground plane extraction operations.
 */

#pragma once

// STL
#include <concepts>
#include <optional>

// OVM
#include "../types.h"

// OpenVDB
#include <openvdb/points/PointAttribute.h>

namespace ovm::ops
{

// naive implementation of a ground plane extractor:
//  iterate through all points, taking the minimum Z height
//  in a given column of voxels as the "GROUND"
std::optional<Map> ground_plane_extraction_geometric(const openvdb::points::PointDataGrid::Ptr& grid)
{
  using namespace openvdb::points;
  
  // sanity check inputs
  if (!grid || grid->empty())
    return std::nullopt;

  // initialize output map dimensions and pose from the grid's bounding box
  const auto bbox = grid->evalActiveVoxelBoundingBox();
  const auto dimensions = bbox.dim();
  const auto origin = grid->transform().indexToWorld(bbox.min());

  ovm::Map result;
  result.map = ovm::Map::MapT::Constant(dimensions.y(), dimensions.x(), NAN);
  result.pose = ovm::Map::PoseT {origin.x(), origin.y() };

  // Iterate over all the leaf nodes in the grid.
  for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf)
  {
    // Create a read-only AttributeHandle. Position always uses Vec3f.
    const AttributeHandle<openvdb::Vec3f> handle(leaf->constAttributeArray("P"));

    // Iterate over the point indices in the leaf.
    for (auto idx = leaf->beginIndexOn(); idx; ++idx)
    {
      // Extract the voxel-space position of the point.
      const openvdb::Vec3f voxelPosition = handle.get(*idx);

      // Extract the index-space position of the voxel.
      const openvdb::Vec3d xyz = idx.getCoord().asVec3d();

      // Compute the world-space position of the point.
      const openvdb::Vec3f worldPosition = grid->transform().indexToWorld(voxelPosition + xyz);
      
      // update 2D map value with the lowest Z value found so far
      const auto coords = xyz - bbox.min();
      auto& val = result.map(coords.y(), coords.x());
      val = (std::isnan(val)) ? worldPosition.z() : std::min(val, worldPosition.z());
    }
  }

  return result;
}

} // namespace ovm::ops