/* Conversions to and from OpenVDB types for PCL.
 *
 */

#pragma once

// STL
#include <concepts>
#include <optional>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/points/PointAttribute.h>
#include <openvdb/points/PointConversion.h>

// OVM
#include "../types.h"
#include "../openvdb/PointMerge.h"

namespace ovm::conversions
{

// type checking for PCL point - does it have XYZ fields?
template <typename PointT>
concept bool HasXYZ = requires(PointT pt) { {pt.x && pt.y && pt.z} -> bool; };
// type checking for PCL point - does it have a label field?
template <typename PointT>
concept bool HasLabel = requires(PointT pt) { {pt.label} -> bool; };
// type checking for PCL point - does it have a confidence field?
template <typename PointT>
concept bool HasConfidence = requires(PointT pt) { {pt.confidence} -> bool; };

// construct an OpenVDB PointDataGrid from a PCL PointCloud
template <typename PointT>
requires HasXYZ<PointT>
openvdb::points::PointDataGrid::Ptr from_pcl(const pcl::PointCloud<PointT>& cloud, const Options& opts = Options())
{
  // @TODO consider making a wrapper around a PCL cloud directly to avoid copying
  //    into a std::vector; need to ensure this works as well with attributes!

  // convenience namespaces
  using namespace openvdb::points;
  using namespace openvdb::math;
  using namespace openvdb::tools;

  // initialize position vectors
  std::vector<AttPositionT> positions; positions.reserve(cloud.size());

  // initialize timestamps
  //  note we convert from PCL timestamp convention (microseconds) to ours (seconds)
  std::vector<AttStampT> stamps(cloud.size(), cloud.header.stamp * 1e-6f);

  // enable auxiliary attribute vectors
  std::vector<AttLabelT> labels; labels.reserve(cloud.size());
  std::vector<AttConfidenceT> confidences; confidences.reserve(cloud.size());

  // convert from PCL object to a series of vectors (XYZ, Attributes)
  for (const auto& pt : cloud)
  {
    positions.emplace_back(pt.x, pt.y, pt.z);
    if constexpr (HasLabel<PointT>)
      labels.emplace_back(pt.label);
    else
      labels.emplace_back(opts.default_label);
    if constexpr (HasConfidence<PointT>)
      confidences.emplace_back(pt.confidence);
    else
      confidences.emplace_back(opts.default_confidence);
  }

  // construct a standard linear transform (i.e. all Voxels are cubes)
  Transform::Ptr transform = Transform::createLinearTransform(opts.voxel_size);

  // construct an index grid (mapping from voxel space to the positions array)
  PointAttributeVector<AttPositionT> positionsWrapper(positions);
  PointIndexGrid::Ptr pointIndexGrid = createPointIndexGrid<PointIndexGrid>(positionsWrapper, *transform);

  // construct grid with embedded point data
  auto grid = createPointDataGrid<NullCodec, PointDataGrid>(*pointIndexGrid, positionsWrapper, *transform);

  // Append "stamp" attribute to the grid to hold the timestamp.
  appendAttribute(grid->tree(), ATT_STAMP, TypedAttributeArray<AttStampT>::attributeType());
  populateAttribute(grid->tree(), pointIndexGrid->tree(), ATT_STAMP, PointAttributeVector<AttStampT>(stamps));

  // Append "label" attribute to the grid to hold the label.
  appendAttribute(grid->tree(), ATT_LABEL, TypedAttributeArray<AttLabelT>::attributeType());
  populateAttribute(grid->tree(), pointIndexGrid->tree(), ATT_LABEL, PointAttributeVector<AttLabelT>(labels));

  // Append "confidence" attribute to the grid to hold the confidence.
  appendAttribute(grid->tree(), ATT_CONFIDENCE, TypedAttributeArray<AttConfidenceT>::attributeType());
  populateAttribute(grid->tree(), pointIndexGrid->tree(), ATT_CONFIDENCE, PointAttributeVector<AttConfidenceT>(confidences));

  // return constructed grid
  return grid;
}

// convert an openvdb point grid to a PCL PointCloud
template <typename PointT = pcl::PointXYZ>
requires HasXYZ<PointT>
std::optional<pcl::PointCloud<PointT>> to_pcl(const openvdb::points::PointDataGrid::Ptr& grid)
{
  using namespace openvdb::points;

  // handle edge case inputs
  if (!grid || grid->empty())
    return std::nullopt;

  // initialize output cloud
  pcl::PointCloud<PointT> cloud;
  cloud.reserve(pointCount(grid->tree()));

  // iterate through all points in the cloud
  for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf)
  {
    // Extract the position attribute from the leaf by name (P is position).
    AttributeHandle<AttPositionT> positionHandle(leaf->constAttributeArray(ATT_POSITION));
    AttributeHandle<AttStampT> stampHandle(leaf->constAttributeArray(ATT_STAMP));

    // Extract the attribute handles as well (may be unused)
    [[maybe_unused]] AttributeHandle<AttLabelT> labelHandle(leaf->constAttributeArray(ATT_LABEL));
    [[maybe_unused]] AttributeHandle<AttConfidenceT> confidenceHandle(leaf->constAttributeArray(ATT_CONFIDENCE));

    // Iterate over the point indices in the leaf.
    for (auto index = leaf->beginIndexOn(); index; ++index)
    {
      // get position and convert to world space
      auto xyz = grid->transform().indexToWorld(positionHandle.get(*index) + index.getCoord().asVec3d());

      // create openvdb point
      PointT point(xyz.x(), xyz.y(), xyz.z());

      // append additional attributes, if supported
      if constexpr (HasLabel<PointT>)
        point.label = labelHandle.get(*index);
      if constexpr (HasConfidence<PointT>)
        point.confidence = confidenceHandle.get(*index);

      // add point to cloud
      cloud.push_back(std::move(point));

      // update overall cloud timestamp with latest
      if (const auto stamp = stampHandle.get(*index); stamp > cloud.header.stamp)
        cloud.header.stamp = stamp;
    }
  }

  // update metadata (PCL convention has timestamps in microseconds)
  cloud.header.stamp *= 1e6;

  // return full cloud
  return cloud;
}

} // namespace ovm::conversions