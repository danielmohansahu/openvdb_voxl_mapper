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

/* Cloud class - individual point attributes are stored as aligned vectors.
*/
struct Cloud
{
  std::vector<openvdb::Vec3f> xyz;  // XYZ positions in World coordinate frame
  std::vector<size_t> labels;       // class label
  std::vector<float> confidences;   // class confidences

  // return the size (number of points) 
  size_t size()
  {
    assert(xyz.size() == labels.size() && labels.size() == confidences.size());
    return xyz.size();
  }

  // concatenation with another cloud
  void concatenate(const Cloud& other)
  {
    xyz.insert(xyz.end(), other.xyz.begin(), other.xyz.end());
    labels.insert(labels.end(), other.labels.begin(), other.labels.end());
    confidences.insert(confidences.end(), other.confidences.begin(), other.confidences.end());
  }
}; // struct Cloud

} // namespace ovm