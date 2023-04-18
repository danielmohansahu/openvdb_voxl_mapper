/* Conversions to and from OpenVDB types.
 *
 */

#pragma once

// STL
#include <concepts>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/points/PointAttribute.h>
#include <openvdb/points/PointConversion.h>

namespace ovm::conversions
{

// member requirements for pcl::Point* types
template <typename PointT>
concept bool HasXYZ = requires(PointT pt) { {pt.x && pt.y && pt.z} -> bool; };

// construct an OpenVDB PointDataGrid from a PCL PointCloud
template <typename PointT>
requires HasXYZ<PointT>
openvdb::points::PointDataGrid::Ptr from_pcl(const pcl::PointCloud<PointT>& cloud)
{
  // @TODO support attributes!
  // @TODO breakout voxel size to be configurable
  // @TODO consider making a wrapper around a PCL cloud directly to avoid copying
  //    into a std::vector; need to ensure this works as well with attributes!

  // convenience namespaces
  using namespace openvdb::points;
  using namespace openvdb::math;
  using namespace openvdb::tools;

  // convert from PCL object to a series of vectors (XYZ, Attributes)
  std::vector<openvdb::Vec3f> positions;
  positions.reserve(cloud.size());
  for (const auto& pt : cloud)
    positions.emplace_back(pt.x, pt.y, pt.z);

  // make a position wrapper to support bucketing of points into voxels
  PointAttributeVector<openvdb::Vec3f> positionsWrapper(positions);

  // construct a standard linear transform (i.e. all Voxels are cubes)
  Transform::Ptr transform = Transform::createLinearTransform(0.5);

  // construct an index grid (mapping from voxel space to the positions array)
  PointIndexGrid::Ptr pointIndexGrid = createPointIndexGrid<PointIndexGrid>(positionsWrapper, *transform);

  // move point data into a grid directly
  return createPointDataGrid<NullCodec, PointDataGrid>(*pointIndexGrid, positionsWrapper, *transform);
}

} // namespace ovm::conversions