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

// OVM
#include "types.h"

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
openvdb::points::PointDataGrid::Ptr from_pcl(const pcl::PointCloud<PointT>& cloud, const Options& opts)
{
  // @TODO consider making a wrapper around a PCL cloud directly to avoid copying
  //    into a std::vector; need to ensure this works as well with attributes!

  // convenience namespaces
  using namespace openvdb::points;
  using namespace openvdb::math;
  using namespace openvdb::tools;

  // initialize position vectors
  std::vector<openvdb::Vec3f> positions; positions.reserve(cloud.size());

  // enable auxiliary attribute vectors
  std::vector<size_t> labels; labels.reserve(cloud.size());
  std::vector<float> confidences; confidences.reserve(cloud.size());

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
  PointAttributeVector<openvdb::Vec3f> positionsWrapper(positions);
  PointIndexGrid::Ptr pointIndexGrid = createPointIndexGrid<PointIndexGrid>(positionsWrapper, *transform);

  // construct grid with embedded point data
  auto grid = createPointDataGrid<NullCodec, PointDataGrid>(*pointIndexGrid, positionsWrapper, *transform);

  // Append "label" attribute to the grid to hold the label.
  appendAttribute(grid->tree(), "label", TypedAttributeArray<size_t>::attributeType());
  PointAttributeVector<size_t> labelsWrapper(labels);
  populateAttribute<PointDataTree,PointIndexTree,PointAttributeVector<size_t>>(grid->tree(), pointIndexGrid->tree(), "label", labelsWrapper);

  // Append "confidence" attribute to the grid to hold the confidence.
  appendAttribute(grid->tree(), "confidence", TypedAttributeArray<float>::attributeType());
  PointAttributeVector<float> confidencesWrapper(confidences);
  populateAttribute(grid->tree(), pointIndexGrid->tree(), "confidence", confidencesWrapper);

  // return constructed grid
  return grid;
}

} // namespace ovm::conversions