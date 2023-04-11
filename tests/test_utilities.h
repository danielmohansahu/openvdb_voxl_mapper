/* Utility functions used throughout unit tests.
 *
 */

// STL
#include <string>

// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/points/PointCount.h>

// OVM
#include <openvdb_voxel_mapper/types.h>

namespace ovm_test
{

// construct a random cloud, evenly sampled across a sphere
ovm::Cloud make_random_cloud(const float voxelSize = 0.5,
                             const size_t pointsPerVoxel = 8,
                             const float radius = 20.0,
                             const std::string filename = "")
{
  // basically copied from "Random Point Generation" example:
  //  https://www.openvdb.org/documentation/doxygen/codeExamples.html
  
  using namespace openvdb;
  using namespace openvdb::points;
  using namespace openvdb::tools;

  // Generate a level set grid.
  FloatGrid::Ptr sphereGrid = createLevelSetSphere<FloatGrid>(radius, /*center=*/openvdb::Vec3f(0,0,0), voxelSize);

  // Retrieve the number of leaf nodes in the grid.
  Index leafCount = sphereGrid->tree().leafCount();

  // Use the topology to create a PointDataTree
  PointDataTree::Ptr pointTree(new PointDataTree(sphereGrid->tree(), 0, openvdb::TopologyCopy()));

  // Ensure all tiles have been voxelized
  pointTree->voxelizeActiveTiles();

  // Define the position type and codec using fixed-point 16-bit compression.
  using PositionAttribute = TypedAttributeArray<Vec3f, FixedPointCodec<false>>;
  NamePair positionType = PositionAttribute::attributeType();

  // Create a new Attribute Descriptor with position only
  AttributeSet::Descriptor::Ptr descriptor(AttributeSet::Descriptor::create(positionType));

  // Determine the number of points / voxel and points / leaf.
  Index voxelsPerLeaf = PointDataGrid::TreeType::LeafNodeType::SIZE;
  Index pointsPerLeaf = pointsPerVoxel * voxelsPerLeaf;

  // Iterate over the leaf nodes in the point tree.
  for (auto leafIter = pointTree->beginLeaf(); leafIter; ++leafIter)
  {
    // Initialize the attributes using the descriptor and point count.
    leafIter->initializeAttributes(descriptor, pointsPerLeaf);
    // Initialize the voxel offsets
    openvdb::Index offset(0);
    for (openvdb::Index index = 0; index < voxelsPerLeaf; ++index)
    {
      offset += pointsPerVoxel;
      leafIter->setOffsetOn(index, offset);
    }
  }

  // Create the points grid.
  PointDataGrid::Ptr points = PointDataGrid::create(pointTree);

  // Set the name of the grid.
  points->setName("RandomPoints");

  // Copy the transform from the sphere grid.
  points->setTransform(sphereGrid->transform().copy());

  // Randomize the point positions.
  std::mt19937 generator(/*seed=*/0);
  std::uniform_real_distribution<> distribution(-0.5, 0.5);

  // initialize resulting cloud
  ovm::Cloud result;

  // Iterate over the leaf nodes in the point tree.
  for (auto leafIter = points->tree().beginLeaf(); leafIter; ++leafIter)
  {
    // Create an AttributeWriteHandle for position.
    // Note that the handle only requires the value type, not the codec.
    AttributeArray& array = leafIter->attributeArray("P");
    AttributeWriteHandle<openvdb::Vec3f> handle(array);

    // Iterate over the point indices in the leaf.
    for (auto indexIter = leafIter->beginIndexOn(); indexIter; ++indexIter)
    {
      // Compute a new random position (in the range -0.5 => 0.5).
      openvdb::Vec3f positionVoxelSpace(distribution(generator));

      // Set the position of this point.
      // As point positions are stored relative to the voxel center, it is
      // not necessary to convert these voxel space values into
      // world-space during this process.
      handle.set(*indexIter, positionVoxelSpace);

      // get this position in world space
      openvdb::Vec3f pointWorldLocation = points->transform().indexToWorld(indexIter.getCoord().asVec3d() + positionVoxelSpace);

      // store this in our raw cloud format (note we don't bother saving it in the cloud, as we're lazy)
      result.xyz.push_back(pointWorldLocation);
      result.labels.push_back(0);
      result.confidences.push_back(1.0);
    }
  }

  // save, if given a filename
  if (filename != "")
    openvdb::io::File(filename).write({points});

  // return full cloud
  return result;
}

// convenience overload
ovm::Cloud make_random_cloud(const std::string filename)
{
  return make_random_cloud(0.5, 8, 20.0, filename);
}

} // namespace ovm_test