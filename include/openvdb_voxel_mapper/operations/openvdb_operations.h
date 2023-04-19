/* utility_operations.h
 *
 * Generic operations to support multithreaded traversal and
 * analysis of the OpenVDB PointDataGrid data structure.
 */

#pragma once

// STL
#include <limits>

// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/points/PointDelete.h>

namespace ovm::ops
{

// filter attribute by user-supplied function
template <typename AttT>
class AttributeCriterionFilter
{
 public:
  // convenience typedefs
  using Handle = openvdb::points::AttributeHandle<AttT>;

  // construct from scratch
  AttributeCriterionFilter(const size_t index, const std::function<bool(const AttT&)>& criterion)
    : _index(index), _criterion(criterion) {}

  // copy constructor
  AttributeCriterionFilter(const AttributeCriterionFilter& filter)
    : _index(filter._index), _criterion(filter._criterion)
  {
    if (filter._handle)
      _handle.reset(new Handle(*filter._handle));
  }

  // whether or not filter is ready for use
  inline bool initialized() const { return bool(_handle); }

  // current filter state
  static openvdb::points::index::State state() { return openvdb::points::index::PARTIAL; }

  // current filter state
  template <typename LeafT>
  static openvdb::points::index::State state(const LeafT&) { return openvdb::points::index::PARTIAL; }

  // turn over a new leaf
  template <typename LeafT>
  void reset(const LeafT& leaf)
  {
    assert(leaf.hasAttribute(_index));
    _handle.reset(new Handle(leaf.constAttributeArray(_index)));
  }

  // return true if the filter is valid
  template <typename IterT>
  bool valid(const IterT& iter) const
  {
    // call user supplied criterion function
    assert(_handle);
    return _criterion(_handle->get(*iter));
  }

 private:
  const size_t _index;                                // attribute index
  typename Handle::UniquePtr _handle;                 // attribute handle;
  const std::function<bool(const AttT&)> _criterion;  // user supplied attribute criteria
}; // class AttributeCriterionFilter


// drop points that don't meet the given criterion for the given attribute
template <typename AttT, typename PointDataTreeT>
void drop_by_attribute_criterion(PointDataTreeT& tree,
                                 const std::string& attribute,
                                 const std::function<bool(const AttT&)>& criterion)
{
  // we're using an internal function here, which could cause maintenance problems...
  using openvdb::points::point_delete_internal::DeleteByFilterOp;

  // handle edge case
  if (tree.empty())
    return;

  // get attribute unique ID
  openvdb::Index64 index = tree.cbeginLeaf()->attributeSet().descriptor().find(attribute);
  if (index == openvdb::points::AttributeSet::INVALID_POS)
    throw std::runtime_error("Unable to filter by attribute " + attribute + ", which doesn't exist!");

  // construct a filter to weed out attributes that don't meet the criterion
  AttributeCriterionFilter filter(index, criterion);

  // acquire registry lock to avoid locking when appending attributes in parallel
  {
    openvdb::points::AttributeArray::ScopedRegistryLock lock;

    // perform actual deletion
    openvdb::tree::LeafManager leaf_manager(tree);
    DeleteByFilterOp<PointDataTreeT,AttributeCriterionFilter<AttT>> delete_operation(filter, &lock);
    tbb::parallel_for(leaf_manager.leafRange(), delete_operation);
  }

  // remove empty leaf nodes
  openvdb::tools::pruneInactive(tree);
}
  
} // namespace ovm::ops