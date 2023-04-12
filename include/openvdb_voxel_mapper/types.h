/* types.h
 *
 * Base types used throughout this codebase.
 */

#pragma once

// STL
#include <vector>
#include <assert.h>

// Eigen
#include <Eigen/Core>

// OpenVDB
#include <openvdb/openvdb.h>

namespace ovm
{

// our "Map" class is just an eigen matrix
typedef Eigen::MatrixXf Map;

// simplest supported point type: XYZ coordinates
struct PointXYZ
{
  openvdb::Vec3f xyz;   // cartesian position in world space
  PointXYZ(const openvdb::Vec3f xyz_) : xyz(xyz_) {}
}; // struct PointXYZ

// XYZ point with classification label
struct PointXYZL
{
  openvdb::Vec3f xyz;   // cartesian position in world space
  int label;            // label identifier
  PointXYZL(const openvdb::Vec3f xyz_, const int label_) : xyz(xyz_), label(label_) {}
}; // struct PointXYZL

// XYZ point with label and confidence
struct PointXYZLC
{
  openvdb::Vec3f xyz;   // cartesian position in world space
  int label;            // label identifier
  float confidence;     // classification confidence [0,1.0]
  PointXYZLC(const openvdb::Vec3f xyz_, const int label_, const float confidence_)
   : xyz(xyz_), label(label_), confidence(confidence_) {}
}; // struct PointXYZLC

// OpenVDB Point-partitioner compatible Cloud class
template <typename PointT>
class Cloud
{
 public:
  using ValueType = openvdb::Vec3f;
  using value_type = openvdb::Vec3f;
  using PosType = openvdb::Vec3f;

  // default empty constructor
  Cloud() = default;

  // expected constructor from a 1-D vector of points
  explicit Cloud(const std::vector<PointT>& data) : _data(data) {}

  // required interface for OpenVDB Point-partitioner
  size_t size() const { return _data.size(); }

  // required interface for OpenVDB Point-partitioner
  void getPos(size_t n, PosType& xyz) const { xyz = _data[n].xyz; }

  // required interface for OpenVDB Point-partitioner
  void get(ValueType& value, size_t n) const { value = _data[n].xyz; }

  // required interface for OpenVDB Point-partitioner
  void get(ValueType& value, size_t n, openvdb::Index m) const { value = _data[n + m].xyz; }

  // access raw point directly
  const PointT& getPoint(size_t n) const
  {
    return _data[n];
  }

  // add an individual point into the cloud
  void insert(const PointT& point)
  {
    _data.push_back(point);
  }

  // add another cloud into this cloud
  void concatenate(const Cloud& other)
  {
    _data.insert(_data.end(), other._data.begin(), other._data.end());
  }

 private:
  // core data
  std::vector<PointT> _data;

}; // Cloud


} // namespace ovm