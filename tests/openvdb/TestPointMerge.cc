// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

#include <openvdb/openvdb.h>
#include <openvdb/points/PointAttribute.h>
#include <openvdb/points/PointGroup.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include <openvdb_voxel_mapper/openvdb/PointMerge.h>

#include <gtest/gtest.h>

using namespace openvdb;
using namespace openvdb::points;

class TestPointMerge: public ::testing::Test
{
public:
    void SetUp() override { openvdb::initialize(); }
    void TearDown() override { openvdb::uninitialize(); }
}; // class TestPointMerge

TEST_F(TestPointMerge, testMerge)
{
    const float voxelSize = 0.1f;
    math::Transform::Ptr transform(math::Transform::createLinearTransform(voxelSize));
    std::vector<Vec3f> points1{{0,0,0}};
    std::vector<Vec3f> points2{{10,0,0}};

    PointDataGrid::Ptr grid1 = createPointDataGrid<NullCodec, PointDataGrid, Vec3f>(points1, *transform);
    PointDataGrid::Ptr grid2 = createPointDataGrid<NullCodec, PointDataGrid, Vec3f>(points2, *transform);
    mergePoints(*grid1, *grid2);

    // Ensure point in B was merged into A

    math::Coord coord = transform->worldToIndexCellCentered(points2[0]);
    EXPECT_TRUE(grid1->tree().probeLeaf(coord)->pointCount() == 1);

    // Ensure original point in A still exists

    coord = transform->worldToIndexCellCentered(points1[0]);
    EXPECT_TRUE(grid1->tree().probeLeaf(coord)->pointCount() == 1);
}


TEST_F(TestPointMerge, testGroupMerge)
{
    const float voxelSize = 0.1f;
    math::Transform::Ptr transform(math::Transform::createLinearTransform(voxelSize));
    std::vector<Vec3f> points{{0,0,0}};

    {
        PointDataGrid::Ptr grid1 = createPointDataGrid<NullCodec, PointDataGrid, Vec3f>(points, *transform);
        PointDataGrid::Ptr grid2 = createPointDataGrid<NullCodec, PointDataGrid, Vec3f>(points, *transform);
        appendGroup(grid2->tree(), "a1");
        setGroup(grid2->tree(), "a1", true);

        mergePoints(*grid1, *grid2);
        EXPECT_TRUE(grid1->tree().cbeginLeaf());
        EXPECT_TRUE(!grid2->tree().cbeginLeaf());

        const auto leafIter = grid1->tree().cbeginLeaf();
        const auto& desc = leafIter->attributeSet().descriptor();

        EXPECT_TRUE(leafIter);
        EXPECT_TRUE(desc.hasGroup("a1"));
        EXPECT_EQ(leafIter->pointCount(), Index64(2));
        GroupHandle handle(leafIter->groupHandle("a1"));
        EXPECT_EQ(handle.get(0), false);
        EXPECT_EQ(handle.get(1), true);
    }

    {
        PointDataGrid::Ptr grid1 = createPointDataGrid<NullCodec, PointDataGrid, Vec3f>(points, *transform);
        PointDataGrid::Ptr grid2 = createPointDataGrid<NullCodec, PointDataGrid, Vec3f>(points, *transform);
        appendGroup(grid1->tree(), "a1");
        setGroup(grid1->tree(), "a1", true);

        mergePoints(*grid1, *grid2);
        EXPECT_TRUE(grid1->tree().cbeginLeaf());
        EXPECT_TRUE(!grid2->tree().cbeginLeaf());

        const auto leafIter = grid1->tree().cbeginLeaf();
        const auto& desc = leafIter->attributeSet().descriptor();

        EXPECT_TRUE(leafIter);
        EXPECT_TRUE(desc.hasGroup("a1"));
        EXPECT_EQ(leafIter->pointCount(), Index64(2));
        GroupHandle handle(leafIter->groupHandle("a1"));
        EXPECT_EQ(handle.get(0), true);
        EXPECT_EQ(handle.get(1), false);
    }
}


TEST_F(TestPointMerge, testMultiAttributeMerge)
{
    // five points across four leafs with transform1, all the in same leaf with
    // transform2

    math::Transform::Ptr transform1(math::Transform::createLinearTransform(1.0));
    math::Transform::Ptr transform2(math::Transform::createLinearTransform(10.0));
    std::vector<Vec3s> positions{{1, 1, 1}, {1, 3, 1}, {2, 5, 1}, {5, 1, 1}, {5, 5, 1}};
    const Index64 totalPointCount(positions.size() * 3);

    {
        PointDataGrid::Ptr grid1 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform1);
        PointDataTree& tree1 = grid1->tree();

        appendAttribute<short>(tree1, "a1");
        appendAttribute<Vec3f>(tree1, "a2");
        appendAttribute<double>(tree1, "a5");
        appendAttribute<Vec3i>(tree1, "a6");

        PointDataGrid::Ptr grid2 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform1);
        PointDataTree& tree2 = grid2->tree();

        appendAttribute<int64_t>(tree2, "a3");
        appendAttribute<double>(tree2, "a5");
        appendAttribute<short>(tree2, "a1");
        appendAttribute<Vec3f>(tree2, "a2");

        PointDataGrid::Ptr grid3 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform1);
        PointDataTree& tree3 = grid3->tree();

        appendAttribute<short>(tree3, "a4");
        appendAttribute<Vec3f>(tree3, "a2");
        appendAttribute<int64_t>(tree3, "a3");
        appendAttribute<short>(tree3, "a1");

        //   grid1 has:
        //    - a1: short, a2: vec3f, a5: double, a6: vec3short
        //   grid2 has:
        //    - a3: int64_t, a5: double, a1: short, a2: vec3f
        //   grid3 has:
        //    - a4: short, a2: vec3f, a3: int64_t, a1: short

        std::vector<PointDataGrid::Ptr> grids;
        grids.push_back(grid2);
        grids.push_back(grid3);

        mergePoints(*grid1, grids);

        EXPECT_TRUE(grid1->tree().cbeginLeaf());
        EXPECT_TRUE(!grid2->tree().cbeginLeaf());
        EXPECT_TRUE(!grid3->tree().cbeginLeaf());
        EXPECT_EQ(totalPointCount, pointCount(tree1));

        const auto leafIter = tree1.cbeginLeaf();

        EXPECT_TRUE(leafIter);
        EXPECT_TRUE(leafIter->hasAttribute("a1"));
        EXPECT_TRUE(leafIter->attributeArray("a1").hasValueType<short>());
        EXPECT_TRUE(leafIter->hasAttribute("a2"));
        EXPECT_TRUE(leafIter->attributeArray("a2").hasValueType<Vec3f>());
        EXPECT_TRUE(leafIter->hasAttribute("a3"));
        EXPECT_TRUE(leafIter->attributeArray("a3").hasValueType<int64_t>());
        EXPECT_TRUE(leafIter->hasAttribute("a4"));
        EXPECT_TRUE(leafIter->attributeArray("a4").hasValueType<short>());
        EXPECT_TRUE(leafIter->hasAttribute("a5"));
        EXPECT_TRUE(leafIter->attributeArray("a5").hasValueType<double>());
        EXPECT_TRUE(leafIter->hasAttribute("a6"));
        EXPECT_TRUE(leafIter->attributeArray("a6").hasValueType<Vec3i>());
    }

    {
        PointDataGrid::Ptr grid1 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform2);
        PointDataTree& tree1 = grid1->tree();

        appendAttribute<short>(tree1, "a1");
        appendAttribute<Vec3f>(tree1, "a2");
        appendAttribute<double>(tree1, "a5");
        appendAttribute<Vec3i>(tree1, "a6");

        PointDataGrid::Ptr grid2 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform2);
        PointDataTree& tree2 = grid2->tree();

        appendAttribute<int64_t>(tree2, "a3");
        appendAttribute<double>(tree2, "a5");
        appendAttribute<short>(tree2, "a1");
        appendAttribute<Vec3f>(tree2, "a2");

        PointDataGrid::Ptr grid3 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform2);
        PointDataTree& tree3 = grid3->tree();

        appendAttribute<short>(tree3, "a4");
        appendAttribute<Vec3f>(tree3, "a2");
        appendAttribute<int64_t>(tree3, "a3");
        appendAttribute<short>(tree3, "a1");

        //   grid1 has:
        //    - a1: short, a2: vec3f, a5: double, a6: vec3short
        //   grid2 has:
        //    - a3: int64_t, a5: double, a1: short, a2: vec3f
        //   grid3 has:
        //    - a4: short, a2: vec3f, a3: int64_t, a1: short

        std::vector<PointDataGrid::Ptr> grids;
        grids.push_back(grid2);
        grids.push_back(grid3);

        mergePoints(*grid1, grids);

        EXPECT_TRUE(grid1->tree().cbeginLeaf());
        EXPECT_TRUE(!grid2->tree().cbeginLeaf());
        EXPECT_TRUE(!grid3->tree().cbeginLeaf());
        EXPECT_EQ(totalPointCount, pointCount(tree1));

        const auto leafIter = tree1.cbeginLeaf();

        EXPECT_TRUE(leafIter);
        EXPECT_TRUE(leafIter->hasAttribute("a1"));
        EXPECT_TRUE(leafIter->attributeArray("a1").hasValueType<short>());
        EXPECT_TRUE(leafIter->hasAttribute("a2"));
        EXPECT_TRUE(leafIter->attributeArray("a2").hasValueType<Vec3f>());
        EXPECT_TRUE(leafIter->hasAttribute("a3"));
        EXPECT_TRUE(leafIter->attributeArray("a3").hasValueType<int64_t>());
        EXPECT_TRUE(leafIter->hasAttribute("a4"));
        EXPECT_TRUE(leafIter->attributeArray("a4").hasValueType<short>());
        EXPECT_TRUE(leafIter->hasAttribute("a5"));
        EXPECT_TRUE(leafIter->attributeArray("a5").hasValueType<double>());
        EXPECT_TRUE(leafIter->hasAttribute("a6"));
        EXPECT_TRUE(leafIter->attributeArray("a6").hasValueType<Vec3i>());
    }
}


TEST_F(TestPointMerge, testMultiGroupMerge)
{
    // five points across four leafs with transform1, all the in same leaf with
    // transform2

    math::Transform::Ptr transform1(math::Transform::createLinearTransform(1.0));
    math::Transform::Ptr transform2(math::Transform::createLinearTransform(10.0));
    std::vector<Vec3s> positions{{1, 1, 1}, {1, 3, 1}, {2, 5, 1}, {5, 1, 1}, {5, 5, 1}};
    const Index64 totalPointCount(positions.size() * 3);

    {
        PointDataGrid::Ptr grid1 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform1);
        PointDataTree& tree1 = grid1->tree();

        appendGroup(tree1, "a1");
        appendGroup(tree1, "a2");
        appendGroup(tree1, "a5");
        appendGroup(tree1, "a6");
        appendGroup(tree1, "a7");

        PointDataGrid::Ptr grid2 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform1);
        PointDataTree& tree2 = grid2->tree();

        appendGroup(tree2, "a3");
        appendGroup(tree2, "a5");
        appendGroup(tree2, "a1");
        appendGroup(tree2, "a2");
        appendGroup(tree2, "a8");

        PointDataGrid::Ptr grid3 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform1);
        PointDataTree& tree3 = grid3->tree();

        appendGroup(tree3, "a4");
        appendGroup(tree3, "a2");
        appendGroup(tree3, "a3");
        appendGroup(tree3, "a1");
        appendGroup(tree3, "a9");

        //   grid1 has:
        //    - a1, a2, a5, a6, a7
        //   grid2 has:
        //    - a3, a5, a1, a2, a8
        //   grid3 has:
        //    - a4, a2, a3, a1, a9

        std::vector<PointDataGrid::Ptr> grids;
        grids.push_back(grid2);
        grids.push_back(grid3);

        mergePoints(*grid1, grids);

        EXPECT_TRUE(grid1->tree().cbeginLeaf());
        EXPECT_TRUE(!grid2->tree().cbeginLeaf());
        EXPECT_TRUE(!grid3->tree().cbeginLeaf());
        EXPECT_EQ(totalPointCount, pointCount(tree1));

        const auto leafIter = tree1.cbeginLeaf();
        EXPECT_TRUE(leafIter);

        const auto& desc = leafIter->attributeSet().descriptor();

        EXPECT_TRUE(desc.hasGroup("a1"));
        EXPECT_TRUE(desc.hasGroup("a2"));
        EXPECT_TRUE(desc.hasGroup("a3"));
        EXPECT_TRUE(desc.hasGroup("a4"));
        EXPECT_TRUE(desc.hasGroup("a5"));
        EXPECT_TRUE(desc.hasGroup("a6"));
        EXPECT_TRUE(desc.hasGroup("a7"));
        EXPECT_TRUE(desc.hasGroup("a8"));
        EXPECT_TRUE(desc.hasGroup("a9"));
    }

    {
        PointDataGrid::Ptr grid1 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform2);
        PointDataTree& tree1 = grid1->tree();

        appendGroup(tree1, "a1");
        appendGroup(tree1, "a2");
        appendGroup(tree1, "a5");
        appendGroup(tree1, "a6");
        appendGroup(tree1, "a7");

        PointDataGrid::Ptr grid2 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform2);
        PointDataTree& tree2 = grid2->tree();

        appendGroup(tree2, "a3");
        appendGroup(tree2, "a5");
        appendGroup(tree2, "a1");
        appendGroup(tree2, "a2");
        appendGroup(tree2, "a8");

        PointDataGrid::Ptr grid3 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform2);
        PointDataTree& tree3 = grid3->tree();

        appendGroup(tree3, "a4");
        appendGroup(tree3, "a2");
        appendGroup(tree3, "a3");
        appendGroup(tree3, "a1");
        appendGroup(tree3, "a9");

        std::vector<PointDataGrid::Ptr> grids;
        grids.push_back(grid2);
        grids.push_back(grid3);

        mergePoints(*grid1, grids);

        EXPECT_TRUE(grid1->tree().cbeginLeaf());
        EXPECT_TRUE(!grid2->tree().cbeginLeaf());
        EXPECT_TRUE(!grid3->tree().cbeginLeaf());
        EXPECT_EQ(totalPointCount, pointCount(tree1));

        const auto leafIter = tree1.cbeginLeaf();
        EXPECT_TRUE(leafIter);

        const auto& desc = leafIter->attributeSet().descriptor();

        EXPECT_TRUE(desc.hasGroup("a1"));
        EXPECT_TRUE(desc.hasGroup("a2"));
        EXPECT_TRUE(desc.hasGroup("a3"));
        EXPECT_TRUE(desc.hasGroup("a4"));
        EXPECT_TRUE(desc.hasGroup("a5"));
        EXPECT_TRUE(desc.hasGroup("a6"));
        EXPECT_TRUE(desc.hasGroup("a7"));
        EXPECT_TRUE(desc.hasGroup("a8"));
        EXPECT_TRUE(desc.hasGroup("a9"));
    }
}


TEST_F(TestPointMerge, testCompressionMerge)
{
    // five points across four leafs with transform1, all the in same leaf with
    // transform2

    math::Transform::Ptr transform1(math::Transform::createLinearTransform(1.0));
    math::Transform::Ptr transform2(math::Transform::createLinearTransform(10.0));
    std::vector<Vec3s> positions{{1, 1, 1}, {1, 3, 1}, {2, 5, 1}, {5, 1, 1}, {5, 5, 1}};
    const Index64 totalPointCount(positions.size() * 2);

    using PositionType = TypedAttributeArray<Vec3f, NullCodec>;

    {
        PointDataGrid::Ptr grid1 = createPointDataGrid<FixedPointCodec<false>, PointDataGrid>(positions, *transform1);
        PointDataGrid::Ptr grid2 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform1);

        mergePoints(*grid1, *grid2);

        EXPECT_TRUE(grid1->tree().cbeginLeaf());
        EXPECT_TRUE(!grid2->tree().cbeginLeaf());
        EXPECT_EQ(totalPointCount, pointCount(grid1->tree()));

        const auto leafIter = grid1->tree().cbeginLeaf();
        EXPECT_TRUE(leafIter);
        EXPECT_TRUE(leafIter->hasAttribute("P"));
        EXPECT_TRUE(leafIter->attributeArray("P").isType<PositionType>());
    }

    {
        PointDataGrid::Ptr grid1 = createPointDataGrid<FixedPointCodec<false>, PointDataGrid>(positions, *transform2);
        PointDataGrid::Ptr grid2 = createPointDataGrid<NullCodec, PointDataGrid>(positions, *transform2);

        mergePoints(*grid1, *grid2);

        EXPECT_TRUE(grid1->tree().cbeginLeaf());
        EXPECT_TRUE(!grid2->tree().cbeginLeaf());
        EXPECT_EQ(totalPointCount, pointCount(grid1->tree()));

        const auto leafIter = grid1->tree().cbeginLeaf();
        EXPECT_TRUE(leafIter);
        EXPECT_TRUE(leafIter->hasAttribute("P"));
        EXPECT_TRUE(leafIter->attributeArray("P").isType<PositionType>());
    }
}


TEST_F(TestPointMerge, testStringMerge)
{
    std::vector<Vec3s> positions1{{1, 1, 1}, {1, 3, 1}, {2, 5, 1}};
    std::vector<Vec3s> positions2{{1, 2, 1}, {100, 3, 1}, {5, 2, 8}};

    std::vector<std::string> str1{"abc", "def", "foo"};
    std::vector<std::string> str2{"bar", "ijk", "def"};

    math::Transform::Ptr transform(math::Transform::createLinearTransform(1.0));

    PointAttributeVector<Vec3s> posWrapper1(positions1);
    PointAttributeVector<Vec3s> posWrapper2(positions2);

    tools::PointIndexGrid::Ptr indexGrid1 = tools::createPointIndexGrid<tools::PointIndexGrid>(posWrapper1, *transform);
    PointDataGrid::Ptr grid1 = createPointDataGrid<NullCodec, PointDataGrid>(*indexGrid1, posWrapper1, *transform);

    tools::PointIndexGrid::Ptr indexGrid2 = tools::createPointIndexGrid<tools::PointIndexGrid>(posWrapper2, *transform);
    PointDataGrid::Ptr grid2 = createPointDataGrid<NullCodec, PointDataGrid>(*indexGrid2, posWrapper2, *transform);

    appendAttribute<std::string>(grid1->tree(), "test");
    appendAttribute<std::string>(grid2->tree(), "test");

    PointAttributeVector<std::string> strWrapper1(str1);
    PointAttributeVector<std::string> strWrapper2(str2);

    populateAttribute<PointDataTree, tools::PointIndexTree>(
        grid1->tree(), indexGrid1->tree(), "test", strWrapper1);
    populateAttribute<PointDataTree, tools::PointIndexTree>(
        grid2->tree(), indexGrid2->tree(), "test", strWrapper2);

    mergePoints(*grid1, *grid2);

    const MetaMap& meta = grid1->tree().cbeginLeaf()->attributeSet().descriptor().getMetadata();

    std::vector<std::string> stringValues;

    for (auto it = meta.beginMeta(); it != meta.endMeta(); ++it) {
        stringValues.push_back(it->second->str());
    }

    // each expected string occurs once and once only

    EXPECT_EQ(stringValues.size(), size_t(5));
    EXPECT_TRUE(std::find(stringValues.begin(), stringValues.end(), "abc") != stringValues.end());
    EXPECT_TRUE(std::find(stringValues.begin(), stringValues.end(), "def") != stringValues.end());
    EXPECT_TRUE(std::find(stringValues.begin(), stringValues.end(), "foo") != stringValues.end());
    EXPECT_TRUE(std::find(stringValues.begin(), stringValues.end(), "bar") != stringValues.end());
    EXPECT_TRUE(std::find(stringValues.begin(), stringValues.end(), "ijk") != stringValues.end());

    stringValues.clear();

    for (auto leafIter = grid1->tree().cbeginLeaf(); leafIter; ++leafIter) {
        StringAttributeHandle handle(leafIter->constAttributeArray("test"), leafIter->attributeSet().descriptor().getMetadata());
        for (auto indexIter = leafIter->beginIndexOn(); indexIter; ++indexIter) {
            std::string str = handle.get(*indexIter);
            stringValues.push_back(str);
        }
    }

    EXPECT_EQ(stringValues.size(), size_t(6));

    EXPECT_TRUE(std::find(stringValues.begin(), stringValues.end(), "abc") != stringValues.end());
    EXPECT_TRUE(std::find(stringValues.begin(), stringValues.end(), "def") != stringValues.end());
    EXPECT_TRUE(std::find(stringValues.begin(), stringValues.end(), "foo") != stringValues.end());
    EXPECT_TRUE(std::find(stringValues.begin(), stringValues.end(), "bar") != stringValues.end());
    EXPECT_TRUE(std::find(stringValues.begin(), stringValues.end(), "ijk") != stringValues.end());
}
