/* Conversions to and from OpenVDB types for PCL.
 *
 * @TODO: The logic in these conversions has gotten pretty convoluted;
 *        figure out a way to abstract some of the per-template 
 *        conversion logic into standalone functions.
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
openvdb::points::PointDataGrid::Ptr from_pcl(const pcl::PointCloud<PointT>& cloud,
                                             const std::shared_ptr<const Options>& opts)
{
  // @TODO consider making a wrapper around a PCL cloud directly to avoid copying
  //    into a std::vector; need to ensure this works as well with attributes!

  // convenience namespaces
  using namespace openvdb::points;
  using namespace openvdb::math;
  using namespace openvdb::tools;

  // useful variables
  const size_t num_labels = opts->labels.size();

  // sanity checks
  if (cloud.header.frame_id != opts->frame)
    throw std::runtime_error("Frame mismatch! Current '" + opts->frame
                             + "' vs. incoming '" + cloud.header.frame_id + "'.");
  if constexpr (!HasLabel<PointT>)
    if (num_labels != 0)
      throw std::runtime_error("Semantic extraction requested for invalid Point Type.");
  
  // @TODO - emit some sort of warning for this, but it doesn't warrant spam...
  // if constexpr (!HasLabel<PointT> && HasConfidence<PointT>)
  //   throw std::runtime_error("PointT has 'confidence' but no 'label' - 'confidence' will be ignored.")

  // initialize position vectors
  std::vector<AttPositionT> positions;
  positions.reserve(cloud.size());

  // initialize timestamps
  //  note we convert from PCL timestamp convention (microseconds) to ours (seconds)
  std::vector<AttStampT> stamps(cloud.size(), cloud.header.stamp * 1e-6);

  // enable confidence attribute - each point gets a confidence value per label
  std::vector<AttConfidenceT> confidences(cloud.size() * num_labels, 0.0);

  // convert from PCL object to a series of vectors (XYZ, Attributes)
  for (size_t i = 0; i != cloud.size(); ++i)
  {
    // access point data
    const auto& pt = cloud[i];

    // get spatial information
    positions.emplace_back(pt.x, pt.y, pt.z);
    // get confidence information
    if constexpr (HasLabel<PointT>)
      if (num_labels != 0)
      {
        // get the confidence value, which might be a default
        AttConfidenceT confidence = opts->default_confidence;
        if constexpr (HasConfidence<PointT>)
          confidence = pt.confidence;

        // sanity checks
        assert( (confidence >= 0.0f) && (confidence <= 1.0f) );

        // find the index of this label and set the confidence
        const auto it = std::find(opts->labels.begin(), opts->labels.end(), pt.label);

        // if we don't check it'll assign everything to the first label
        //  classic premature optimization. but it's something very noticeable on the user end...
        assert( it != opts->labels.end() );
        confidences[i * num_labels + std::distance(opts->labels.begin(), it)] = confidence;
      }
  }

  // construct a standard linear transform (i.e. all Voxels are cubes)
  Transform::Ptr transform = Transform::createLinearTransform(opts->voxel_size);

  // construct an index grid (mapping from voxel space to the positions array)
  PointAttributeVector<AttPositionT> positionsWrapper(positions);
  PointIndexGrid::Ptr pointIndexGrid = createPointIndexGrid<PointIndexGrid>(positionsWrapper, *transform);

  // construct grid with embedded point data
  auto grid = createPointDataGrid<NullCodec, PointDataGrid>(*pointIndexGrid, positionsWrapper, *transform);

  // Append "stamp" attribute to the grid to hold the timestamp.
  appendAttribute(grid->tree(), ATT_STAMP, TypedAttributeArray<AttStampT>::attributeType());
  populateAttribute(grid->tree(), pointIndexGrid->tree(), ATT_STAMP, PointAttributeVector<AttStampT>(stamps));

  if (num_labels != 0)
  {
    // Append "confidence" attribute to the grid to hold the confidence. stride is 'num_labels', because each point has num_labels possible confidences
    appendAttribute(grid->tree(), ATT_CONFIDENCE, TypedAttributeArray<AttConfidenceT>::attributeType(), num_labels);
    populateAttribute(grid->tree(), pointIndexGrid->tree(), ATT_CONFIDENCE, PointAttributeVector<AttConfidenceT>(confidences, num_labels), num_labels);
  }

  // return constructed grid
  return grid;
}

// convert an openvdb point grid to a PCL PointCloud
template <typename PointT = pcl::PointXYZ>
requires HasXYZ<PointT>
std::optional<pcl::PointCloud<PointT>> to_pcl(const openvdb::points::PointDataGrid::Ptr& grid,
                                              const std::shared_ptr<const Options>& options)
{
  using namespace openvdb::points;

  // handle edge case inputs
  if (!grid || grid->empty())
    return std::nullopt;

  // sanity checks
  [[maybe_unused]] const size_t num_labels = options->labels.size();
  if constexpr (HasLabel<PointT>)
    if (num_labels == 0)
      throw std::runtime_error("Cannot convert to desired Point Type - no label information.");

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
    [[maybe_unused]] std::unique_ptr<AttributeHandle<AttConfidenceT>> confidenceHandle;
    if constexpr (HasLabel<PointT>)
      confidenceHandle = std::make_unique<AttributeHandle<AttConfidenceT>>(leaf->constAttributeArray(ATT_CONFIDENCE));

    // Iterate over the point indices in the leaf.
    for (auto index = leaf->beginIndexOn(); index; ++index)
    {
      // get position and convert to world space
      auto xyz = grid->transform().indexToWorld(positionHandle.get(*index) + index.getCoord().asVec3d());

      // create openvdb point
      PointT point(xyz.x(), xyz.y(), xyz.z());

      // append additional attributes, if supported
      if constexpr (HasLabel<PointT>)
      {
        // sanity checks
        assert (num_labels != 0);
        assert (confidenceHandle);

        // extract the top label associated with this point, and possibly the confidence
        AttConfidenceT max_confidence = 0.0;
        size_t max_confidence_idx = num_labels;
        for (size_t i = 0; i != num_labels; ++i)
          if (const auto confidence = confidenceHandle->get(*index, i); confidence > max_confidence)
          {
            max_confidence     = confidence;
            max_confidence_idx = i;
          }
        // set the values
        point.label = (max_confidence_idx < num_labels) ? options->labels[max_confidence_idx] : options->unknown;
        if constexpr (HasConfidence<PointT>)
          point.confidence = max_confidence;
      }

      // add point to cloud
      cloud.push_back(std::move(point));

      // update overall cloud timestamp with latest
      if (const auto stamp = stampHandle.get(*index); stamp > cloud.header.stamp)
        cloud.header.stamp = stamp;
    }
  }

  // update metadata (PCL convention has timestamps in microseconds)
  cloud.header.stamp *= 1e6;
  cloud.header.frame_id = options->frame;

  // return full cloud
  return cloud;
}

} // namespace ovm::conversions