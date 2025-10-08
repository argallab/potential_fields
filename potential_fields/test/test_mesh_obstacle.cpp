// Unit tests for mesh collision helpers: pointInsideMesh & distanceToMesh
// We build a synthetic unit cube mesh programmatically to avoid external resources.

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fcl/fcl.h>
#include <vector>
#include <memory>

#include "pfield/mesh_collision.hpp"

namespace {

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
    auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    std::vector<fcl::Vector3d> fclVerts;
    fclVerts.reserve(verts.size());
    for (auto& v : verts) fclVerts.emplace_back(v.x(), v.y(), v.z());
    std::vector<fcl::Triangle> fclTris;
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
    data.meshObj = std::make_shared<fcl::CollisionObjectd>(data.bvh, fcl::Transform3d::Identity());
    return data;
  }

} // namespace

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
  EXPECT_NEAR(distanceToMesh(cube, Eigen::Vector3d(0.5, 0.0, 0.0)), 0.0, 1e-6);
  // Point outside along +X: cube max x = 0.5, point=0.8 => expected distance 0.3
  EXPECT_NEAR(distanceToMesh(cube, Eigen::Vector3d(0.8, 0.0, 0.0)), 0.3, 1e-6);
  // Outside in diagonal direction: compare to analytical distance to box
  Eigen::Vector3d p(0.8, 0.9, 0.0); // outside in x & y
  double dx = 0.8 - 0.5; // 0.3
  double dy = 0.9 - 0.5; // 0.4
  double expected = std::sqrt(dx * dx + dy * dy);
  EXPECT_NEAR(distanceToMesh(cube, p), expected, 1e-6);
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
