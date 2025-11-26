// Unit tests for mesh collision helpers: pointInsideMesh, computeUnsignedDistanceToMesh,
// getClosestPointOnMesh, and PotentialFieldObstacle::computeSignedDistanceAndNormal.
// We build a synthetic unit cube mesh programmatically to avoid external resources.

#include "pfield_library/pfield/pf_obstacle.hpp"
#include "pfield_library/pfield/mesh_collision.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <filesystem>
#include <fstream>

namespace {


  static std::string writeUnitCubeAsciiSTLToTemp() {
    // Pick a temp path
    namespace fs = std::filesystem;
    fs::path tmp = fs::temp_directory_path() / "unit_cube_test.stl";

    // ASCII STL content from above:
    static const char* kAsciiStl = R"(solid unit_cube
  facet normal 1 0 0
    outer loop
      vertex 0.5 -0.5 -0.5
      vertex 0.5  0.5 -0.5
      vertex 0.5  0.5  0.5
    endloop
  endfacet
  facet normal 1 0 0
    outer loop
      vertex 0.5 -0.5 -0.5
      vertex 0.5  0.5  0.5
      vertex 0.5 -0.5  0.5
    endloop
  endfacet

  facet normal -1 0 0
    outer loop
      vertex -0.5 -0.5 -0.5
      vertex -0.5  0.5  0.5
      vertex -0.5  0.5 -0.5
    endloop
  endfacet
  facet normal -1 0 0
    outer loop
      vertex -0.5 -0.5 -0.5
      vertex -0.5 -0.5  0.5
      vertex -0.5  0.5  0.5
    endloop
  endfacet

  facet normal 0 1 0
    outer loop
      vertex -0.5 0.5 -0.5
      vertex  0.5 0.5  0.5
      vertex  0.5 0.5 -0.5
    endloop
  endfacet
  facet normal 0 1 0
    outer loop
      vertex -0.5 0.5 -0.5
      vertex -0.5 0.5  0.5
      vertex  0.5 0.5  0.5
    endloop
  endfacet

  facet normal 0 -1 0
    outer loop
      vertex -0.5 -0.5 -0.5
      vertex  0.5 -0.5 -0.5
      vertex  0.5 -0.5  0.5
    endloop
  endfacet
  facet normal 0 -1 0
    outer loop
      vertex -0.5 -0.5 -0.5
      vertex  0.5 -0.5  0.5
      vertex -0.5 -0.5  0.5
    endloop
  endfacet

  facet normal 0 0 1
    outer loop
      vertex -0.5 -0.5 0.5
      vertex  0.5 -0.5 0.5
      vertex  0.5  0.5 0.5
    endloop
  endfacet
  facet normal 0 0 1
    outer loop
      vertex -0.5 -0.5 0.5
      vertex  0.5  0.5 0.5
      vertex -0.5  0.5 0.5
    endloop
  endfacet

  facet normal 0 0 -1
    outer loop
      vertex -0.5 -0.5 -0.5
      vertex  0.5  0.5 -0.5
      vertex  0.5 -0.5 -0.5
    endloop
  endfacet
  facet normal 0 0 -1
    outer loop
      vertex -0.5 -0.5 -0.5
      vertex -0.5  0.5 -0.5
      vertex  0.5  0.5 -0.5
    endloop
  endfacet
endsolid unit_cube
)";

    std::ofstream(tmp) << kAsciiStl;

    // Build a file:// URI (POSIX). On Windows, you may need file:///<drive>:/path
    std::string path = tmp.string();
    std::string uri = "file://" + path;
    return uri;
  }


  MeshCollisionData buildUnitCubeMesh() {
    // Cube centered at origin spanning [-0.5,0.5] in each axis.
    std::vector<Eigen::Vector3d> verts = {
      {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5}, // bottom
      {-0.5, -0.5,  0.5}, {0.5, -0.5,  0.5}, {0.5, 0.5,  0.5}, {-0.5, 0.5,  0.5}  // top
    };
    // 12 triangles (two per face) using right-handed ordering
    std::vector<Eigen::Vector3i> tris = {
      {0, 1, 2}, {0, 2, 3}, // bottom
      {4, 6, 5}, {4, 7, 6}, // top
      {0, 4, 5}, {0, 5, 1}, // -Y side
      {1, 5, 6}, {1, 6, 2}, // +X side
      {2, 6, 7}, {2, 7, 3}, // +Y side
      {3, 7, 4}, {3, 4, 0}  // -X side
    };

    // Build FCL BVH
    auto model = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
    std::vector<coal::Vec3s> fclVerts;
    fclVerts.reserve(verts.size());
    for (auto& v : verts) fclVerts.emplace_back(v.x(), v.y(), v.z());
    std::vector<coal::Triangle> fclTris;
    fclTris.reserve(tris.size());
    for (auto& t : tris) fclTris.emplace_back(t.x(), t.y(), t.z());
    model->beginModel();
    model->addSubModel(fclVerts, fclTris);
    model->endModel();

    // Populate MeshCollisionData similar to loadMesh
    MeshCollisionData data;
    data.bvh = model;
    data.vertices = verts;
    data.triangles = tris;
    data.centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d minB = Eigen::Vector3d::Constant(1e9);
    Eigen::Vector3d maxB = -minB;
    double maxR2 = 0.0;
    for (auto& v : verts) {
      minB = minB.cwiseMin(v);
      maxB = maxB.cwiseMax(v);
      double d2 = v.squaredNorm();
      if (d2 > maxR2) maxR2 = d2;
    }
    data.aabbMin = minB;
    data.aabbMax = maxB;
    data.radius = std::sqrt(maxR2);
    data.maxExtent = (maxB - minB).maxCoeff();
    data.collisionObject = std::make_shared<coal::CollisionObject>(data.bvh, coal::Transform3s::Identity());
    return data;
  }

} // namespace

TEST(MeshCollision, LoadMesh_FromAsciiSTL_UnitCube) {
  const std::string uri = writeUnitCubeAsciiSTLToTemp();

  auto meshPtr = loadMesh(uri);
  ASSERT_TRUE(meshPtr) << "loadMesh returned null";

  // Basic geometry checks should match the synthetic cube.
  EXPECT_NEAR(meshPtr->aabbMin.x(), -0.5, 1e-6);
  EXPECT_NEAR(meshPtr->aabbMin.y(), -0.5, 1e-6);
  EXPECT_NEAR(meshPtr->aabbMin.z(), -0.5, 1e-6);
  EXPECT_NEAR(meshPtr->aabbMax.x(), 0.5, 1e-6);
  EXPECT_NEAR(meshPtr->aabbMax.y(), 0.5, 1e-6);
  EXPECT_NEAR(meshPtr->aabbMax.z(), 0.5, 1e-6);

  // Centroid (as you compute it) should be at or very near the origin.
  EXPECT_NEAR(meshPtr->centroid.x(), 0.0, 1e-6);
  EXPECT_NEAR(meshPtr->centroid.y(), 0.0, 1e-6);
  EXPECT_NEAR(meshPtr->centroid.z(), 0.0, 1e-6);

  // Max extent and radius should match expectations.
  EXPECT_NEAR(meshPtr->maxExtent, 1.0, 1e-6);
  EXPECT_NEAR(meshPtr->radius, std::sqrt(0.75), 1e-6); // sqrt( (0.5)^2 * 3 )

  // Make sure COAL objects exist
  ASSERT_TRUE(meshPtr->bvh);
  ASSERT_TRUE(meshPtr->collisionObject);

  // Quick smoke tests on the helpers
  EXPECT_TRUE(pointInsideMesh(*meshPtr, Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_FALSE(pointInsideMesh(*meshPtr, Eigen::Vector3d(0.6, 0.0, 0.0)));

  EXPECT_NEAR(computeUnsignedDistanceToMesh(*meshPtr, Eigen::Vector3d(0.8, 0.0, 0.0)), 0.3, 1e-6);
  {
    const Eigen::Vector3d p(0.0, 0.0, 0.3);
    const double d = computeUnsignedDistanceToMesh(*meshPtr, p);
    const bool inside = pointInsideMesh(*meshPtr, p);
    const double signed_d = inside ? -std::abs(d) : std::abs(d);
    EXPECT_NEAR(signed_d, -0.2, 1e-6);
  }
}


TEST(MeshCollision, PointInsideAndOutsideCube) {
  MeshCollisionData cube = buildUnitCubeMesh();
  // Cube is {-0.5,-0.5,-0.5} to {0.5,0.5,0.5}
  EXPECT_TRUE(pointInsideMesh(cube, Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(pointInsideMesh(cube, Eigen::Vector3d(0.2, -0.1, 0.3)));
  EXPECT_FALSE(pointInsideMesh(cube, Eigen::Vector3d(0.6, 0.0, 0.0)));
  EXPECT_FALSE(pointInsideMesh(cube, Eigen::Vector3d(0.0, 0.0, 0.6)));
}

TEST(MeshCollision, DistanceOutsideCube) {
  MeshCollisionData cube = buildUnitCubeMesh();
  // Point on surface should give ~0 distance (tolerate small numerical err)
  EXPECT_NEAR(computeUnsignedDistanceToMesh(cube, Eigen::Vector3d(0.5, 0.0, 0.0)), 0.0, 1e-6);
  // Point outside along +X: cube max x = 0.5, point=0.8 => expected distance 0.3
  EXPECT_NEAR(computeUnsignedDistanceToMesh(cube, Eigen::Vector3d(0.8, 0.0, 0.0)), 0.3, 1e-6);
  // Outside in diagonal direction: compare to analytical distance to box
  Eigen::Vector3d p(0.8, 0.9, 0.0); // outside in x & y
  double dx = 0.8 - 0.5; // 0.3
  double dy = 0.9 - 0.5; // 0.4
  double expected = std::sqrt(dx * dx + dy * dy);
  EXPECT_NEAR(computeUnsignedDistanceToMesh(cube, p), expected, 1e-6);
}

TEST(MeshCollision, DistanceInsideCube_SignedViaInsideCheck) {
  MeshCollisionData cube = buildUnitCubeMesh();
  // Center of cube is 0.5m away from any face; distance should be -0.5
  {
    const Eigen::Vector3d p(0.0, 0.0, 0.0);
    const double d = computeUnsignedDistanceToMesh(cube, p);
    const bool inside = pointInsideMesh(cube, p);
    const double signed_d = inside ? -std::abs(d) : std::abs(d);
    EXPECT_NEAR(signed_d, -0.5, 1e-6);
  }
  // Point closer to +Z face: distance should be -(0.5 - 0.3) = -0.2
  {
    const Eigen::Vector3d p(0.0, 0.0, 0.3);
    const double d = computeUnsignedDistanceToMesh(cube, p);
    const bool inside = pointInsideMesh(cube, p);
    const double signed_d = inside ? -std::abs(d) : std::abs(d);
    EXPECT_NEAR(signed_d, -0.2, 1e-6);
  }
}

TEST(MeshCollision, ClosestPointBasicCases) {
  MeshCollisionData cube = buildUnitCubeMesh();
  Eigen::Vector3d cp;
  // Outside near +X face, within YZ extents
  ASSERT_TRUE(getClosestPointOnMesh(cube, Eigen::Vector3d(0.8, 0.2, 0.0), cp));
  EXPECT_NEAR(cp.x(), 0.5, 1e-6);
  EXPECT_NEAR(cp.y(), 0.2, 1e-6);
  EXPECT_NEAR(cp.z(), 0.0, 1e-6);

  // Outside beyond +X and +Y, Z in-range -> clamp Y to +0.5
  ASSERT_TRUE(getClosestPointOnMesh(cube, Eigen::Vector3d(0.8, 0.9, 0.1), cp));
  EXPECT_NEAR(cp.x(), 0.5, 1e-6);
  EXPECT_NEAR(cp.y(), 0.5, 1e-6);
  EXPECT_NEAR(cp.z(), 0.1, 1e-6);

  // Inside near +Z face; closest point lies on z=+0.5 plane
  ASSERT_TRUE(getClosestPointOnMesh(cube, Eigen::Vector3d(0.1, -0.2, 0.4), cp));
  EXPECT_NEAR(cp.x(), 0.1, 1e-6);
  EXPECT_NEAR(cp.y(), -0.2, 1e-6);
  EXPECT_NEAR(cp.z(), 0.5, 1e-6);
}

TEST(PFObstacle, SignedDistanceAndNormal_Sphere) {
  // Sphere of radius 1 at origin
  PotentialFieldObstacle sphere(
    "world",
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE, ObstacleGroup::STATIC,
    ObstacleGeometry(/*radius*/1.0, /*L*/0.0, /*W*/0.0, /*H*/0.0));

  double sd; Eigen::Vector3d n;
  sphere.computeSignedDistanceAndNormal(Eigen::Vector3d(2.0, 0.0, 0.0), sd, n);
  EXPECT_NEAR(sd, 1.0, 1e-9);
  EXPECT_NEAR(n.x(), 1.0, 1e-9);
  EXPECT_NEAR(n.y(), 0.0, 1e-9);
  EXPECT_NEAR(n.z(), 0.0, 1e-9);

  sphere.computeSignedDistanceAndNormal(Eigen::Vector3d(0.5, 0.0, 0.0), sd, n);
  EXPECT_NEAR(sd, -0.5, 1e-9);
  EXPECT_NEAR(n.x(), 1.0, 1e-9);
  EXPECT_NEAR(n.y(), 0.0, 1e-9);
  EXPECT_NEAR(n.z(), 0.0, 1e-9);
}

TEST(PFObstacle, SignedDistanceAndNormal_Box_AxisAligned) {
  // Axis-aligned box: length=2, width=2, height=2 centered at origin
  PotentialFieldObstacle box(
    "world",
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
    ObstacleType::BOX, ObstacleGroup::STATIC,
    ObstacleGeometry(/*r*/0.0, /*L*/2.0, /*W*/2.0, /*H*/2.0));

  double sd; Eigen::Vector3d n;
  // Outside along +X
  box.computeSignedDistanceAndNormal(Eigen::Vector3d(3.0, 0.0, 0.0), sd, n);
  EXPECT_NEAR(sd, 2.0, 1e-9); // from x=3 to face at x=+1
  EXPECT_NEAR(n.x(), 1.0, 1e-9);
  EXPECT_NEAR(n.y(), 0.0, 1e-9);
  EXPECT_NEAR(n.z(), 0.0, 1e-9);

  // Inside near +Z face
  box.computeSignedDistanceAndNormal(Eigen::Vector3d(0.2, -0.1, 0.9), sd, n);
  EXPECT_NEAR(sd, -0.1, 1e-9);
  EXPECT_NEAR(n.x(), 0.0, 1e-9);
  EXPECT_NEAR(n.y(), 0.0, 1e-9);
  EXPECT_NEAR(n.z(), 1.0, 1e-9);
}

TEST(PFObstacle, SignedDistanceAndNormal_Cylinder) {
  // Cylinder radius=1, height=2 centered at origin, axis along Z
  PotentialFieldObstacle cyl(
    "world",
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
    ObstacleType::CYLINDER, ObstacleGroup::STATIC,
    ObstacleGeometry(/*r*/1.0, /*L*/0.0, /*W*/0.0, /*H*/2.0));

  double sd; Eigen::Vector3d n;
  // Outside on the side within height
  cyl.computeSignedDistanceAndNormal(Eigen::Vector3d(1.3, 0.0, 0.2), sd, n);
  EXPECT_NEAR(sd, 0.3, 1e-9);
  EXPECT_NEAR(n.x(), 1.0, 1e-9);
  EXPECT_NEAR(n.y(), 0.0, 1e-9);
  EXPECT_NEAR(n.z(), 0.0, 1e-9);

  // Above the top cap center
  cyl.computeSignedDistanceAndNormal(Eigen::Vector3d(0.0, 0.0, 1.3), sd, n);
  EXPECT_NEAR(sd, 0.3, 1e-9);
  EXPECT_NEAR(n.x(), 0.0, 1e-9);
  EXPECT_NEAR(n.y(), 0.0, 1e-9);
  EXPECT_NEAR(n.z(), 1.0, 1e-9);

  // Inside near side
  cyl.computeSignedDistanceAndNormal(Eigen::Vector3d(0.9, 0.0, 0.0), sd, n);
  EXPECT_NEAR(sd, -0.1, 1e-9);
  EXPECT_NEAR(n.x(), -1.0, 1e-9);
  EXPECT_NEAR(n.y(), 0.0, 1e-9);
  EXPECT_NEAR(n.z(), 0.0, 1e-9);
}

TEST(PFObstacle, SignedDistanceAndNormal_MeshCube) {
  // Create a PF obstacle of type MESH and inject our synthetic cube mesh
  PotentialFieldObstacle meshObs(
    "world",
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
    ObstacleType::MESH, ObstacleGroup::STATIC,
    ObstacleGeometry(/*r*/0.0, /*L*/1.0, /*W*/1.0, /*H*/1.0));
  auto cubePtr = std::make_shared<MeshCollisionData>(buildUnitCubeMesh());
  // Assign the synthetic mesh directly (allowed by private->public hack at include)
  meshObs.setMeshCollisionData(cubePtr);

  double sd; Eigen::Vector3d n;
  // Outside along +X
  meshObs.computeSignedDistanceAndNormal(Eigen::Vector3d(0.8, 0.0, 0.0), sd, n);
  EXPECT_NEAR(sd, 0.3, 1e-6);
  EXPECT_NEAR(n.x(), 1.0, 1e-6);
  EXPECT_NEAR(n.y(), 0.0, 1e-6);
  EXPECT_NEAR(n.z(), 0.0, 1e-6);

  // Inside near +Z face
  meshObs.computeSignedDistanceAndNormal(Eigen::Vector3d(0.0, 0.0, 0.3), sd, n);
  EXPECT_NEAR(sd, -0.2, 1e-6);
  EXPECT_NEAR(n.x(), 0.0, 1e-6);
  EXPECT_NEAR(n.y(), 0.0, 1e-6);
  EXPECT_NEAR(n.z(), 1.0, 1e-6);
}

TEST(MeshCollision, BroadPhaseRejectsFarPoint) {
  MeshCollisionData cube = buildUnitCubeMesh();
  // AABB max extent is 1.0 in each axis, far point should be rejected quickly.
  EXPECT_FALSE(pointInsideMesh(cube, Eigen::Vector3d(5.0, 5.0, 5.0)));
}

TEST(MeshCollision, InfluenceMarginDerivationExample) {
  // Construct obstacle using the cube mesh logic by feeding a dummy resource is not feasible here.
  // Instead validate that maxExtent computed matches cube side length (1.0) and radius is sqrt(3)/2.
  MeshCollisionData cube = buildUnitCubeMesh();
  EXPECT_NEAR(cube.maxExtent, 1.0, 1e-9);
  EXPECT_NEAR(cube.radius, std::sqrt(0.75), 1e-9); // max vertex norm = sqrt( (0.5)^2 *3 )
}

TEST(MeshCollision, LoadMeshFromFile) {
  namespace fs = std::filesystem;
  // Locate the test resource relative to this source file
  fs::path src(__FILE__);
  fs::path objPath = src.parent_path() / "resources" / "simple_cube.obj";
  ASSERT_TRUE(fs::exists(objPath)) << "Missing test resource: " << objPath.string();

  // Use file:// URI so geometric_shapes can load from filesystem
  std::string uri = std::string("file://") + objPath.string();

  auto mesh1 = loadMesh(uri);
  ASSERT_TRUE(mesh1);
  EXPECT_TRUE(mesh1->bvh);
  EXPECT_TRUE(mesh1->collisionObject);
  EXPECT_GT(mesh1->vertices.size(), 0u);
  EXPECT_GT(mesh1->triangles.size(), 0u);

  // Basic properties of the unit cube mesh
  EXPECT_NEAR(mesh1->aabbMin.x(), -0.5, 1e-9);
  EXPECT_NEAR(mesh1->aabbMin.y(), -0.5, 1e-9);
  EXPECT_NEAR(mesh1->aabbMin.z(), -0.5, 1e-9);
  EXPECT_NEAR(mesh1->aabbMax.x(), 0.5, 1e-9);
  EXPECT_NEAR(mesh1->aabbMax.y(), 0.5, 1e-9);
  EXPECT_NEAR(mesh1->aabbMax.z(), 0.5, 1e-9);
  EXPECT_NEAR(mesh1->centroid.x(), 0.0, 1e-12);
  EXPECT_NEAR(mesh1->centroid.y(), 0.0, 1e-12);
  EXPECT_NEAR(mesh1->centroid.z(), 0.0, 1e-12);
  EXPECT_NEAR(mesh1->radius, std::sqrt(0.75), 1e-9); // sqrt( (0.5)^2 * 3 )
  EXPECT_NEAR(mesh1->maxExtent, 1.0, 1e-9);

  // Cache behavior: same URI should return the same shared instance
  auto mesh2 = loadMesh(uri);
  ASSERT_TRUE(mesh2);
  EXPECT_EQ(mesh1.get(), mesh2.get());

  // Distance queries via COAL-backed methods
  EXPECT_NEAR(computeUnsignedDistanceToMesh(*mesh1, Eigen::Vector3d(0.5, 0.0, 0.0)), 0.0, 1e-6);
  EXPECT_NEAR(computeUnsignedDistanceToMesh(*mesh1, Eigen::Vector3d(0.8, 0.0, 0.0)), 0.3, 1e-6);
  {
    const Eigen::Vector3d p(0.0, 0.0, 0.0);
    const double d = computeUnsignedDistanceToMesh(*mesh1, p);
    const bool inside = pointInsideMesh(*mesh1, p);
    const double signed_d = inside ? -std::abs(d) : std::abs(d);
    EXPECT_NEAR(signed_d, -0.5, 1e-6);
  }
}
