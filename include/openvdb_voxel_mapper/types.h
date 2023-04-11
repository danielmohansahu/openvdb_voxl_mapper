/* types.h
 *
 * Base types used throughout this codebase.
 */

#pragma once

// STL
#include <vector>
#include <assert.h>

// OpenVDB
#include <openvdb/openvdb.h>

namespace ovm
{

struct Point
{
  openvdb::Vec3f xyz;   // cartesian position in world space
  int label;            // label identifier
  float confidence;     // classification confidence [0,1.0]

  Point(const openvdb::Vec3f xyz_, const int label_, const float confidence_)
   : xyz(xyz_), label(label_), confidence(confidence_) {}
}; // struct Point

// OpenVDB Point-partitioner compatible Cloud class
class Cloud
{
 public:
  using ValueType = openvdb::Vec3f;
  using value_type = openvdb::Vec3f;
  using PosType = openvdb::Vec3f;

  // expected constructor
  Cloud() {}
  explicit Cloud(const std::vector<Point>& data) : _data(data) {}

  // required interface for OpenVDB Point-partitioner
  size_t size() const { return _data.size(); }
  void getPos(size_t n, PosType& xyz) const { xyz = _data[n].xyz; }
  void get(ValueType& value, size_t n) const { value = _data[n].xyz; }
  void get(ValueType& value, size_t n, openvdb::Index m) const { value = _data[n + m].xyz; }

  // add an individual point into the cloud
  void insert(const Point& point)
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
  std::vector<Point> _data;

}; // Cloud


} // namespace ovm