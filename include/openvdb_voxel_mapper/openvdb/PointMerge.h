// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

/// @author Nick Avramoussis
///
/// @file PointMerge.h
///
/// @brief Merge Point Data Grids together
///

#ifndef OPENVDB_POINTS_POINT_MERGE_HAS_BEEN_INCLUDED
#define OPENVDB_POINTS_POINT_MERGE_HAS_BEEN_INCLUDED

#include <openvdb/points/PointDataGrid.h>
#include <openvdb/points/PointAttribute.h>
#include <openvdb/points/PointGroup.h>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace points {

///////////////////////////////////////////////////

/// @brief Method to merge a source point data grid into a target point data grid.
///
/// @details This function steals the point data from the source grid and inserts it
///          into the target grid. After this function returns, the source grid is
///          guaranteed to be empty. The target grid will implicitly create attributes
///          that do not exist and initialise them with a zero ValueType.
///
/// @note    Grid must have the same transforms
///
/// @param targetGrid    the PointDataGrid to merge data into
/// @param sourceGrid    the PointDataGrid to steal data from
///
inline void mergePoints(PointDataGrid& targetGrid, PointDataGrid& sourceGrid);

/// @brief Method to merge a number of source point data grids into a single target
///        point data grid.
///
/// @details This function steals the point data from all provided source grids and
///          inserts it into the target grid. After this function returns, all source
///          grids are guaranteed to be empty. The target grid will implicitly create
///          attributes that do not exist and initialise them with a zero ValueType.
///          Note that the target attibute order is taking from the first source grid
///          if the target grid is empty.
///
/// @note    Grid must have the same transforms. This function is significantly different
///          to the above merge, as it optimises for multiple grids
///
/// @param targetGrid    the PointDataGrid to merge data into
/// @param sourceGrids   a vector of shared PointDataGrid points to steal data from
///
inline void mergePoints(PointDataGrid& targetGrid, std::vector<PointDataGrid::Ptr>& sourceGrids);

namespace point_merge_internal {

struct AttributeDescription {

    AttributeDescription()
        : mType()
        , mStride()
        , mFlags()
        , mMetaDefaultValue() {}

    bool hasConstantStride() const { return bool(mFlags & AttributeArray::CONSTANTSTRIDE); }
    bool hidden() const { return bool(mFlags & AttributeArray::HIDDEN); }
    bool transient() const { return bool(mFlags & AttributeArray::TRANSIENT); }

    openvdb::NamePair mType;
    openvdb::Index mStride;
    uint8_t mFlags;
    openvdb::Metadata::ConstPtr mMetaDefaultValue;
};

// small struct to parallelize the calls to reorderAttributes
struct ReorderOp
{
    ReorderOp(const std::vector<PointDataTree::LeafNodeType*>& leafNodes,
              const AttributeSet::DescriptorPtr& descriptor)
        : mLeafNodes(leafNodes)
        , mDescriptor(descriptor) {}
    void operator()(const tbb::blocked_range<size_t>& range) const {
        for (size_t n = range.begin(), N = range.end(); n < N; ++n) {
            mLeafNodes[n]->reorderAttributes(mDescriptor);
        }
    }

private:
    const std::vector<PointDataTree::LeafNodeType*>& mLeafNodes;
    const AttributeSet::DescriptorPtr& mDescriptor;
}; // class ReorderOp

struct SetFlagsOp
{
    using LeafManagerT = tree::LeafManager<points::PointDataTree>;
    SetFlagsOp(const size_t array,
               const uint8_t flags)
        : mArray(array)
        , mFlags(flags) {}
    void operator()(const LeafManagerT::LeafRange& range) const {
        for (auto leaf = range.begin(); leaf; ++leaf) {
            AttributeArray& array = leaf->attributeArray(mArray);
            array.setHidden(mFlags & AttributeArray::HIDDEN);
            array.setTransient(mFlags & AttributeArray::TRANSIENT);
        }
    }

private:
    const size_t mArray;
    const uint8_t mFlags;
}; // class SetFlagsOp

inline void mergeLeafNodes(PointDataTree::LeafNodeType& target,
                           const std::vector<PointDataTree::LeafNodeType*> sources)
{
    using LeafNodeT = PointDataTree::LeafNodeType;

    struct CopyIter
    {
        using T = std::vector<std::pair<Index, Index>>;

        CopyIter() = default;
        operator bool() const { return index < data.size(); }
        CopyIter& operator++() { index++; return *this; }
        Index sourceIndex() const { assert(*this); return data[index].first; }
        Index targetIndex() const { assert(*this); return data[index].second; }
        //
        inline void addSourceTarget(const Index& s, const Index& t) {
            data.emplace_back(s,t);
        }

    private:
        T data;
        T::size_type index = 0;
    }; // struct CopyIter


    size_t totalPointCount(target.pointCount());
    for (const auto& leaf : sources) {
        assert(leaf->origin() == target.origin());
        totalPointCount += leaf->pointCount();
    }

    const AttributeSet& targetAttributeSet = target.attributeSet();
    const size_t attributeSetSize(targetAttributeSet.size());

    std::unique_ptr<AttributeSet> newAttributeSet(new AttributeSet(targetAttributeSet, totalPointCount));

    std::vector<AttributeArray*> attributeArrays;
    attributeArrays.reserve(attributeSetSize);
    for (size_t i = 0; i < attributeSetSize; ++i) {
        attributeArrays.emplace_back(newAttributeSet->get(i));
    }

    Index attributeIndex(0);
    std::vector<LeafNodeT::ValueType> endOffsets;
    endOffsets.reserve(LeafNodeT::NUM_VALUES);

    std::vector<CopyIter> copyIterators;
    copyIterators.resize(1 + sources.size()); // one copy iterator per leaf

    for (size_t n = 0; n < LeafNodeT::NUM_VALUES; ++n) {
        const Coord coord(target.offsetToGlobalCoord(n));

        // configure target leaf source/target id pairs
        auto ci = copyIterators.begin();
        for (auto iter = target.beginIndexVoxel(coord); iter; ++iter, ++attributeIndex) {
            ci->addSourceTarget(/*source*/*iter, /*target*/attributeIndex);
        }

        // configure source leaf(s) source/target id pairs
        for (const auto& leaf : sources) {
            ++ci; // next copy iterator
            for (auto iter = leaf->beginIndexVoxel(coord); iter; ++iter, ++attributeIndex) {
                ci->addSourceTarget(/*source*/*iter, /*target*/attributeIndex);
            }
        }

        endOffsets.emplace_back(LeafNodeT::ValueType(attributeIndex));
    }

    // do the actual copy
    // @todo  this method preserve the tranfer order of attributes but there is no
    //  doubt a lot of improvements that can be made here (this was quickly bumped to
    //  support VDB 8.0.0).
    for (size_t i = 0; i < attributeSetSize; ++i) {
        size_t j = 0;
        attributeArrays[i]->copyValues(*targetAttributeSet.getConst(i), copyIterators[j++]);
        for (const auto& leaf : sources) {
            attributeArrays[i]->copyValues(*leaf->attributeSet().getConst(i), copyIterators[j++]);
        }
    }

    target.replaceAttributeSet(newAttributeSet.release());
    target.setOffsets(endOffsets, /*updateValueMask*/true);
}

struct MergeOverlapping
{
    using LeafNodeT = PointDataTree::LeafNodeType;

    MergeOverlapping(PointDataTree& targetTree,
                     std::vector<LeafNodeT*>& leafNodes)
        : mTargetTree(targetTree)
        , mLeafNodes(leafNodes) {}

    void operator()(const tbb::blocked_range<size_t>& range) const {
        for (size_t n = range.begin(), N = range.end(); n < N; ++n) {
            LeafNodeT*& sourceLeaf = mLeafNodes[n];
            LeafNodeT* const targetLeaf(mTargetTree.probeLeaf(sourceLeaf->origin()));
            assert(targetLeaf);
            std::vector<LeafNodeT*> nodes;
            nodes.push_back(sourceLeaf);
            mergeLeafNodes(*targetLeaf, nodes);
            delete sourceLeaf;
        }
    }

private:
    PointDataTree&            mTargetTree;
    std::vector<LeafNodeT*>&  mLeafNodes;
};

struct MergeMultiOverlapping
{
    using LeafNodeT = PointDataTree::LeafNodeType;

    MergeMultiOverlapping(PointDataTree& targetTree,
                          std::vector<std::vector<LeafNodeT*> >& leafNodes)
        : mTargetTree(targetTree)
        , mLeafNodes(leafNodes) {}

    void operator()(const tbb::blocked_range<size_t>& range) const {
        for (size_t n = range.begin(), N = range.end(); n < N; ++n) {
            std::vector<LeafNodeT*>& sourceLeafNodes(mLeafNodes[n]);
            assert(sourceLeafNodes.front());
            LeafNodeT* const targetLeaf(mTargetTree.probeLeaf(sourceLeafNodes.front()->origin()));
            assert(targetLeaf);
            mergeLeafNodes(*targetLeaf, sourceLeafNodes);
            for (auto& leaf : sourceLeafNodes) delete leaf;
        }
    }

private:
    PointDataTree&                         mTargetTree;
    std::vector<std::vector<LeafNodeT*> >&  mLeafNodes;
};

template<typename ValueType>
struct CopyAttributeOp
{
    using LeafRangeT = tree::LeafManager<PointDataTree>::LeafRange;
    using OldAttributeHandleT = AttributeHandle<ValueType>;
    using NewAttributeHandleT = AttributeWriteHandle<ValueType>;

    CopyAttributeOp(const size_t oldAttribute,
                    const size_t newAttribute)
        : mOldAttribute(oldAttribute)
        , mNewAttribute(newAttribute) {}

    void operator()(const LeafRangeT& range) const {
        for (LeafRangeT::Iterator leaf=range.begin(); leaf; ++leaf) {
            typename OldAttributeHandleT::Ptr oldHandle = OldAttributeHandleT::create(leaf->constAttributeArray(mOldAttribute));
            typename NewAttributeHandleT::Ptr newHandle = NewAttributeHandleT::create(leaf->attributeArray(mNewAttribute));
            const size_t size(oldHandle->size());
            for (size_t i = 0; i < size; ++i) {
                newHandle->set(i, oldHandle->get(i));
            }
        }
    }

private:
    const size_t mOldAttribute;
    const size_t mNewAttribute;
};

template<typename ValueType>
inline void configureAttribute(PointDataTree& tree,
                               const Name& name,
                               const AttributeDescription& details,
                               const bool attributeExists)
{
    assert(tree.cbeginLeaf());

    Name targetName(name);
    bool reassign(false);

    if (attributeExists) {
        const AttributeSet& attributeSet = tree.cbeginLeaf()->attributeSet();
        const AttributeSet::Descriptor& descriptor = attributeSet.descriptor();
        const size_t pos = descriptor.find(targetName);
        // If the attribute exists, check to see if the compression
        // values match (collectPointData ensures that the value types already
        // match) - if they don't we need to reconfigure
        if (descriptor.type(pos).second != details.mType.second) {
            targetName = descriptor.uniqueName(targetName);
            reassign = true;
        }
    }

    const bool hasMeta(details.mMetaDefaultValue);
    const Metadata* meta = details.mMetaDefaultValue.get();

    if (reassign || !attributeExists) {
        points::appendAttribute(tree, targetName, details.mType, details.mStride,
                                details.hasConstantStride(), meta, details.hidden(),
                                details.transient());
    }

    if (reassign) {
        // copy attribute
        const AttributeSet::Descriptor& descriptor = tree.cbeginLeaf()->attributeSet().descriptor();
        const size_t oldPosition(descriptor.find(name));
        const size_t newPosition(descriptor.find(targetName));
        assert(oldPosition != newPosition);

        CopyAttributeOp<ValueType> copyAttributeOp(oldPosition, newPosition);
        tree::LeafManager<PointDataTree> leafManager(tree);
        tbb::parallel_for(leafManager.leafRange(), copyAttributeOp);

        // For position, as it can't be dropped directly, rename it...
        if (name == "P") {
            const Name tmp(descriptor.uniqueName("P"));
            points::renameAttribute(tree, "P", tmp);
            points::dropAttribute(tree, tmp);
        }
        else {
            points::dropAttribute(tree, oldPosition);
        }
        points::renameAttribute(tree, targetName, name);
    }
    else {
        // else attribute has not been copied (new or exists) so ensure meta
        // and flags are correct
        const AttributeSet& attributeSet = tree.cbeginLeaf()->attributeSet();
        if (hasMeta) {
            assert(typeNameAsString<ValueType>() == meta->typeName());
            if (attributeExists) {
                const AttributeSet::DescriptorPtr descriptorPtr = attributeSet.descriptorPtr();
                descriptorPtr->setDefaultValue(name, *meta);
            }
            else {
                const ValueType& value = static_cast<const TypedMetadata<ValueType>*>(meta)->value();
                points::collapseAttribute<ValueType>(tree, name, value);
            }
        }

        const AttributeArray* const array = attributeSet.getConst(name);
        if (array->flags() != details.mFlags) {
            const size_t pos = attributeSet.descriptor().find(name);
            tree::LeafManager<PointDataTree> leafManager(tree);
            SetFlagsOp setFlagsOp(pos, details.mFlags);
            tbb::parallel_for(leafManager.leafRange(), setFlagsOp);
        }
    }
}

inline void collectPointData(const PointDataTree& tree,
                             std::map<Name, AttributeDescription>& uniqueAttributes,
                             std::set<Name>& uniqueGroups)
{
    const auto leafIter = tree.cbeginLeaf();
    if (!leafIter) return;

    const AttributeSet& attributeSet(leafIter->attributeSet());
    const AttributeSet::Descriptor& descriptor = attributeSet.descriptor();
    const AttributeSet::Util::NameToPosMap& map = descriptor.map();

    for(const auto& mapIter : map) {
        const Name& name = mapIter.first;
        const AttributeArray* const array = attributeSet.getConst(mapIter.second);
        assert(array);
        if (points::isGroup(*array)) continue;

        auto it = uniqueAttributes.find(name);
        const bool hasAttribute(it != uniqueAttributes.end());
        AttributeDescription& attribute = hasAttribute ? it->second : uniqueAttributes[name];

        if (hasAttribute) {
            const NamePair& type = descriptor.type(mapIter.second);
            if (attribute.mType.first != type.first)
                throw std::runtime_error("Cannot merge Point Data Grids with matching attribute names of different types");
            if (attribute.mStride != array->stride() || attribute.hasConstantStride() != array->hasConstantStride())
                throw std::runtime_error("Cannot merge Point Data Grids with matching attribute names of different strides");
            // If codec are mismatching, the resultant codec should alway be null
            if (attribute.mType.second != type.second)
                attribute.mType.second = NullCodec::name();
        }
        else {
            attribute.mType = descriptor.type(mapIter.second);
            attribute.mStride = array->stride();
            attribute.mFlags = array->flags();
            // Returns nullptr is no default value
            attribute.mMetaDefaultValue = descriptor.getMetadata()["default:" + name];
        }
    }

    const AttributeSet::Util::NameToPosMap& groupMap = descriptor.groupMap();
    for(const auto& mapIter : groupMap) {
        uniqueGroups.insert(mapIter.first);
    }
}

inline void configureAttributes(PointDataTree& tree,
                                const std::map<Name, AttributeDescription>& uniqueAttributes)
{
    const auto leafIter = tree.cbeginLeaf();
    if (!leafIter) return;

    for (const auto& attribute : uniqueAttributes)
    {
        const Name& name = attribute.first;
        const bool hasAttribute(leafIter->hasAttribute(name));

        const AttributeDescription& details = attribute.second;
        const std::string& typeName(details.mType.first);

        if (typeName == typeNameAsString<bool>())                       configureAttribute<bool>(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<int16_t>())               configureAttribute<int16_t>(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<int32_t>())               configureAttribute<int32_t>(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<int64_t>())               configureAttribute<int64_t>(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<float>())                 configureAttribute<float>(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<double>())                configureAttribute<double>(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<Vec3i>())                 configureAttribute<Vec3i>(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<Vec3f>())                 configureAttribute<Vec3f>(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<Vec3R>())                 configureAttribute<Vec3R>(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<math::Mat3<float> >())    configureAttribute<math::Mat3<float> >(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<math::Mat3<double> >())   configureAttribute<math::Mat3<double> >(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<math::Mat4<float> >())    configureAttribute<math::Mat4<float> >(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<math::Mat4<double> >())   configureAttribute<math::Mat4<double> >(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<math::Quat<float> >())    configureAttribute<math::Quat<float> >(tree, name, details, hasAttribute);
        else if (typeName == typeNameAsString<math::Quat<double> >())   configureAttribute<math::Quat<double> >(tree, name, details, hasAttribute);
        else if (!hasAttribute) {
            // If an attribute is not one of the default types above, we can still append it but
            // differing compression types and default value creation can't be supported due to
            // missing public factorymethods
            points::appendAttribute(tree,
                name,
                details.mType,
                details.mStride,
                details.hasConstantStride(),
                details.mMetaDefaultValue.get(),
                details.hidden(),
                details.transient());
        };
    }
}

// Configure points groups based on the order of a target set. Must be
// followed by an attribute reorder
inline void configureGroupsFromTarget(PointDataTree& tree,
                                      const AttributeSet& targetSet)
{
    const AttributeSet::Descriptor& targetDescriptor = targetSet.descriptor();
    const AttributeSet& attributeSet(tree.cbeginLeaf()->attributeSet());
    AttributeSet::DescriptorPtr descriptor = attributeSet.descriptorPtr();

    if (targetDescriptor.hasSameAttributes(*descriptor)) return;

    const AttributeSet::Util::NameToPosMap& targetGroupMap = targetDescriptor.groupMap();

    std::map<Name, std::map<uint8_t, Name>> groupData;
    for (const auto& groupIter : targetGroupMap) {
        const AttributeSet::Util::GroupIndex& idx = targetSet.groupIndex(groupIter.first);
        for (const auto& mapIter : targetDescriptor.map()) {
            if (mapIter.second == idx.first) {
                groupData[mapIter.first][idx.second] = groupIter.first;
                break;
            }
        }
    }

    // gather arrays to remove
    std::vector<size_t> arraysToRemove;
    for (size_t i = 0; i < attributeSet.size(); i++) {
        if (points::isGroup(*(attributeSet.getConst(i)))) {
            arraysToRemove.push_back(i);
        }
    }

    const size_t numGroupArrays(descriptor->count(GroupAttributeArray::attributeType()));
    const size_t groupBits(AttributeSet::Descriptor::groupBits());
    size_t nextGroupOffset(groupBits * numGroupArrays);

    tree::LeafManager<PointDataTree> leafManager(tree);
    tree::LeafManager<PointDataTree>::LeafRange range(leafManager.leafRange());

    // for every group array and subsequent group, create the array and copy the group
    // to the new position if it exists in tree
    for (const auto& groupArray : groupData) {
        const Name& groupArrayName = groupArray.first;

        if (descriptor->find(groupArrayName) != AttributeSet::INVALID_POS) {
            descriptor->rename(groupArrayName, descriptor->uniqueName("__group"));
        }

        descriptor = descriptor->duplicateAppend(groupArrayName, GroupAttributeArray::attributeType());
        const size_t pos = descriptor->find(groupArrayName);
        const uint8_t flags = targetSet.getConst(groupArrayName)->flags();

        // append

        leafManager.foreach(
            [&descriptor, pos, flags](PointDataTree::LeafNodeType& leaf, size_t /*idx*/) {
                const AttributeSet::Descriptor::Ptr expected = leaf.attributeSet().descriptorPtr();
                auto attribute =
                    leaf.appendAttribute(*expected, descriptor, pos, /*stride*/1, /*constant*/true);
                if (flags & AttributeArray::HIDDEN)     attribute->setHidden(true);
                if (flags & AttributeArray::TRANSIENT)  attribute->setTransient(true);
            }, /*threaded=*/ true
        );

        for (const auto& group : groupArray.second) {
            const Name& groupName(group.second);

            // do nothing if the group doesn't exist,
            // the offset will be added later
            if (!descriptor->hasGroup(groupName)) continue;

            const Name tmpGroupName = descriptor->uniqueGroupName(groupName);
            descriptor->setGroup(tmpGroupName, nextGroupOffset + group.first);

            const AttributeSet::Util::GroupIndex sourceIndex(attributeSet.groupIndex(groupName));
            const AttributeSet::Util::GroupIndex targetIndex(attributeSet.groupIndex(tmpGroupName));

            points::point_group_internal::CopyGroupOp<PointDataTree> copy(targetIndex, sourceIndex);
            tbb::parallel_for(range, copy);
            descriptor->renameGroup(groupName, descriptor->uniqueGroupName(groupName));
            descriptor->renameGroup(tmpGroupName, groupName);
        }

        nextGroupOffset += groupBits;
    }

    // drop the group attribute arrays - this also erases the groups
    // from the group map and updates offsets

    points::dropAttributes(tree, arraysToRemove);

    // Update the group map with matching target offsets - these
    // won't be correct until redorder is called but have to be updated
    // first due to group map checking in redorderAttributes()

    descriptor = attributeSet.descriptorPtr();
    for (const auto& groupIter : targetGroupMap) {
        descriptor->setGroup(groupIter.first, groupIter.second);
    }
}

struct RemapStringsOp
{
    using LeafRangeT = tree::LeafManager<PointDataTree>::LeafRange;

    RemapStringsOp( const std::vector<size_t>& stringIndices,
                    const std::unordered_map<std::string, uint32_t>& cache,
                    const MetaMap& meta)
        : mStringIndices(stringIndices)
        , mCache(cache)
        , mMeta(meta) {}

    void operator()(const LeafRangeT& range) const {
        for (auto leaf = range.begin(); leaf; ++leaf) {
            for (const size_t pos : mStringIndices) {

                StringAttributeHandle stringHandle(leaf->constAttributeArray(pos), mMeta);
                AttributeWriteHandle<uint32_t> stringIdHandle(leaf->attributeArray(pos));

                for (auto iter = leaf->beginIndexOn(); iter; ++iter) {
                    const std::string& value = stringHandle.get(*iter);
                    if (value.empty()) continue;

                    auto stringIdIter = mCache.find(value);
                    assert(stringIdIter != mCache.end());

                    stringIdHandle.set(*iter, stringIdIter->second);
                }
            }
        }
    }

private:
    const std::vector<size_t>& mStringIndices;
    const std::unordered_map<std::string, uint32_t>& mCache;
    const MetaMap& mMeta;
}; // struct RemapStringsOp


inline void remapStringAttributes(PointDataTree::Ptr targetTree, std::vector<PointDataTree::Ptr>& sourceTrees)
{
    struct Local
    {
        // extracted from AttributeArrayString.cc until this functionality moves into the main library

        static bool isStringMeta(const Name& key, const Metadata::Ptr& meta)
        {
            // string attribute metadata must have a key that starts "string:"
            if (key.compare(0, 7, "string:") != 0)      return false;
            // ensure the metadata is StringMetadata
            if (meta->typeName() != "string")           return false;

            return true;
        }

        static Index getStringIndex(const Name& key)
        {
            Name indexStr = key.substr(7, key.size() - 7);

            // extract the index as an unsigned integer
            std::istringstream indexSS(indexStr);
            Index index;
            indexSS >> index;

            return (index + 1);
        }
    };

    // handle strings (if present) - do a quick pass over descriptors to figure this
    // out and avoid building the caches if no strings are present

    std::vector<PointDataTree::Ptr> stringSourceTrees;

    for (auto& tree : sourceTrees) {
        const auto& descriptor = tree->cbeginLeaf()->attributeSet().descriptor();
        for (size_t pos = 0; pos < descriptor.size(); pos++) {
            const NamePair& type = descriptor.type(pos);
            if (type.second == StringAttributeArray::attributeType().second) {
                stringSourceTrees.emplace_back(tree);
                break;
            }
        }
    }

    if (stringSourceTrees.empty()) return;

    const auto& targetDescriptor = targetTree->cbeginLeaf()->attributeSet().descriptorPtr();
    MetaMap& targetMeta = targetDescriptor->getMetadata();

    // Update the target metadata with all source strings - scoped to ensure cleanup
    // of the StringMetaInserter internal cache structures

    {
        StringMetaInserter inserter(targetMeta);

        for (const auto& tree : stringSourceTrees) {
            const auto& descriptor = tree->cbeginLeaf()->attributeSet().descriptor();
            const MetaMap& meta = descriptor.getMetadata();

            for (auto it = meta.beginMeta(); it != meta.endMeta(); ++it) {
                if (Local::isStringMeta(it->first, it->second)) {
                    const auto* stringMeta = static_cast<StringMetadata*>(it->second.get());
                    assert(!stringMeta->value().empty());
                    inserter.insert(stringMeta->value());
                }
            }
        }
    }

    // populate a cache of strings to ids

    std::unordered_map<std::string, uint32_t> cache;


    for (auto it = targetMeta.beginMeta(), itEnd = targetMeta.endMeta();
         it != itEnd; ++it) {

        const Name& key = it->first;
        const Metadata::Ptr meta = it->second;

        // ensure the metadata is StringMetadata and key starts "string:"
        if (!Local::isStringMeta(key, meta)) continue;

        const auto* stringMeta = static_cast<StringMetadata*>(meta.get());
        cache[stringMeta->value()] = Local::getStringIndex(key);
    }


    for (const auto& tree : stringSourceTrees) {

        const auto& descriptor = tree->cbeginLeaf()->attributeSet().descriptor();
        const MetaMap& meta = descriptor.getMetadata();

        std::vector<size_t> stringIndices;
        for (size_t pos = 0; pos < descriptor.size(); pos++) {
            const NamePair& type = descriptor.type(pos);
            if (type.second == StringAttributeArray::attributeType().second) {
                stringIndices.emplace_back(pos);
            }
        }

        RemapStringsOp remapOp(stringIndices, cache, meta);
        tree::LeafManager<PointDataTree> leafManager(*tree);
        tbb::parallel_for(leafManager.leafRange(), remapOp);
    }
}

} // point_merge_internal

inline void mergePoints(PointDataGrid& targetGrid, PointDataGrid& sourceGrid)
{
    // early exit if no points in the source grid

    PointDataTree& sourceTree = sourceGrid.tree();
    if (!sourceTree.cbeginLeaf()) return;

    if (targetGrid.transform() != sourceGrid.transform())
        throw std::runtime_error("Point Data Grid transforms differ");

    PointDataTree& targetTree = targetGrid.tree();
    const auto targetLeafIter(targetTree.cbeginLeaf());

    // if no leaves in the target tree, "steal" the tree from the source grid

    if (!targetLeafIter) {
        targetGrid.setTree(sourceGrid.treePtr());
        sourceGrid.newTree();
        return;
    }

    // handle string attributes

    {
        std::vector<PointDataTree::Ptr> sourceTrees = { sourceGrid.treePtr() };
        point_merge_internal::remapStringAttributes(targetGrid.treePtr(), sourceTrees);
    }

    using AttributeMap = std::map<Name, point_merge_internal::AttributeDescription>;

    AttributeMap uniqueAttributes;
    std::set<Name> uniqueGroups;
    point_merge_internal::collectPointData(targetTree, uniqueAttributes, uniqueGroups);
    point_merge_internal::collectPointData(sourceTree, uniqueAttributes, uniqueGroups);

    point_merge_internal::configureAttributes(targetTree, uniqueAttributes);
    point_merge_internal::configureAttributes(sourceTree, uniqueAttributes);

    if (!uniqueGroups.empty()) {
        std::vector<Name> groups;
        groups.insert(groups.end(), uniqueGroups.begin(), uniqueGroups.end());
        points::appendGroups(targetTree, groups);
        points::compactGroups(targetTree);
        point_merge_internal::configureGroupsFromTarget(sourceTree, targetLeafIter->attributeSet());
    }

    std::vector<PointDataTree::LeafNodeType*> leafNodes;
    leafNodes.reserve(sourceTree.leafCount());
    sourceTree.stealNodes(leafNodes);

    if (targetLeafIter) {
        // make sure all the attributes are in the right order based on a target descriptor
        const AttributeSet::DescriptorPtr& targetDescriptor = targetLeafIter->attributeSet().descriptorPtr();
        point_merge_internal::ReorderOp reorderOp(leafNodes, targetDescriptor);
        tbb::blocked_range<size_t> range(0, leafNodes.size());
        tbb::parallel_for(range, reorderOp);
    }

    // merge any non overlapping nodes from the sourceGrid into the targetGrid
    // and leave leafNodes with only the overlapping nodes - use swap/pop
    // as order does not need to be preserved

    for (auto iter = leafNodes.begin(); iter != leafNodes.end();) {
        PointDataTree::LeafNodeType*& leaf(*iter);
        const Coord& origin = leaf->origin();
        if (targetTree.probeConstLeaf(origin)) ++iter;
        else {
            targetTree.addLeaf(leaf);
            std::swap(leaf, leafNodes.back());
            leafNodes.pop_back();
        }
    }

    // if the source grid is not empty, we have active overlapping leaf nodes
    if (leafNodes.empty()) return;

    point_merge_internal::MergeOverlapping mergeOverlappingOp(targetTree, leafNodes);
    tbb::blocked_range<size_t> range(0, leafNodes.size());
    tbb::parallel_for(range, mergeOverlappingOp);
}

inline void mergePoints(PointDataGrid& targetGrid, std::vector<PointDataGrid::Ptr>& sourceGrids)
{
    // collect tree pointers, remove empty grids and error on mismatching transform
    std::vector<PointDataTree::Ptr> sourceTrees;
    for(const auto& sourceGrid : sourceGrids) {
        if (!sourceGrid) continue;
        if (!sourceGrid->tree().cbeginLeaf()) continue;
        if (targetGrid.transform() != sourceGrid->transform())
            throw std::runtime_error("Point Data Grid transforms differ");
        sourceTrees.push_back(sourceGrid->treePtr());
    }

    sourceGrids.clear();
    if (sourceTrees.empty()) return;

    // sourceTrees is guaranteed to have some leaf data - ensure the target tree
    // does too

    if (!targetGrid.tree().cbeginLeaf()) {
        PointDataTree::Ptr newTarget = sourceTrees.front();
        std::swap(newTarget, sourceTrees.back());
        sourceTrees.pop_back();
        targetGrid.setTree(newTarget);
        if (sourceTrees.empty()) return;
    }

    // handle string attributes

    point_merge_internal::remapStringAttributes(targetGrid.treePtr(), sourceTrees);

    PointDataTree& targetTree = targetGrid.tree();
    const auto targetLeafIter(targetTree.cbeginLeaf());
    assert(targetLeafIter);

    // find the unique list of attributes and configure the default values
    // and compression types

    using AttributeMap = std::map<Name, point_merge_internal::AttributeDescription>;

    AttributeMap uniqueAttributes;
    std::set<Name> uniqueGroups;
    point_merge_internal::collectPointData(targetTree, uniqueAttributes, uniqueGroups);
    for(const auto& sourceTree : sourceTrees) {
        point_merge_internal::collectPointData(*sourceTree, uniqueAttributes, uniqueGroups);
    }

    point_merge_internal::configureAttributes(targetTree, uniqueAttributes);
    for(const auto& sourceTree : sourceTrees) {
        point_merge_internal::configureAttributes(*sourceTree, uniqueAttributes);
    }

    if (!uniqueGroups.empty()) {
        std::vector<Name> groups;
        groups.insert(groups.end(), uniqueGroups.begin(), uniqueGroups.end());
        points::appendGroups(targetTree, groups);
        points::compactGroups(targetTree);
        for(const auto& sourceTree : sourceTrees) {
            point_merge_internal::configureGroupsFromTarget(*sourceTree, targetLeafIter->attributeSet());
        }
    }

    size_t totalLeafCount(0);
    for(const auto& sourceTree : sourceTrees) {
        totalLeafCount += sourceTree->leafCount();
    }

    assert(totalLeafCount != 0);
    std::vector<PointDataTree::LeafNodeType*> leafNodes;
    leafNodes.reserve(totalLeafCount);

    for(const auto& sourceTree : sourceTrees) {
        sourceTree->stealNodes(leafNodes);
    }

    // make sure all the attributes are in the right order based on a target descriptor

    {
        AttributeSet::DescriptorPtr targetDescriptor;
        if (targetLeafIter) targetDescriptor = targetTree.cbeginLeaf()->attributeSet().descriptorPtr();
        else targetDescriptor = leafNodes.front()->attributeSet().descriptorPtr();
        // only need to reorder source grids
        point_merge_internal::ReorderOp reorderOp(leafNodes, targetDescriptor);
        tbb::blocked_range<size_t> range(0, totalLeafCount);
        tbb::parallel_for(range, reorderOp);
    }

    // merge any non overlapping nodes from the sourceGrid into the targetGrid
    // and store overlapping nodes

    std::map<openvdb::Coord, std::vector<PointDataTree::LeafNodeType*>> overlappingNodes;

    for (const auto& leaf : leafNodes) {
        const Coord& origin = leaf->origin();
        if (targetTree.probeConstLeaf(origin)) overlappingNodes[origin].push_back(leaf);
        else targetTree.addLeaf(leaf);
    }

    // the container is not empty, we have active overlapping leaf nodes

    if (overlappingNodes.empty()) return;

    const size_t numOverlapping(overlappingNodes.size());
    std::vector<std::vector<PointDataTree::LeafNodeType*>> overlappingLeafNodes(numOverlapping);
    size_t i(0);
    for (const auto& nodes : overlappingNodes) {
        std::vector<PointDataTree::LeafNodeType*>& nodesToMerge = overlappingLeafNodes[i++];
        nodesToMerge.insert(nodesToMerge.begin(), nodes.second.begin(), nodes.second.end());
    }

    point_merge_internal::MergeMultiOverlapping mergeOverlappingOp(targetTree, overlappingLeafNodes);
    tbb::blocked_range<size_t> range(0, numOverlapping);
    tbb::parallel_for(range, mergeOverlappingOp);
}


} // namespace points
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif //OPENVDB_POINTS_POINT_MERGE_HAS_BEEN_INCLUDED
