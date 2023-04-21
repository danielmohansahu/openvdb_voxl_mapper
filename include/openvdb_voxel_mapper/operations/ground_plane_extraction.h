/* Simple ground plane extraction operation.
 *
 * Ground plane extraction operations.
 */

#pragma once

// STL
#include <optional>

// OVM
#include "../types.h"
#include "operation_utils.h"

// OpenVDB
#include <openvdb/points/PointAttribute.h>

namespace ovm::ops
{

// naive implementation of a ground plane extractor:
//  iterate through all points, taking the minimum Z height in a given column of voxels as the "GROUND"
std::optional<Map> ground_plane_extraction_geometric(const openvdb::points::PointDataGrid::Ptr& grid);

// naive implementation of a ground plane extractor (on GPU):
//  iterate through all points, taking the minimum Z height in a given column of voxels as the "GROUND"
std::optional<Map> ground_plane_extraction_geometric_cuda(const openvdb::points::PointDataGrid::Ptr& grid);

} // namespace ovm::ops