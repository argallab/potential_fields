// Copyright 2025 Sharwin Patil
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include "pfield/pf_obstacle.hpp"
#include "pfield/pf_obstacle_geometry.hpp"

// ============================================================================
// Tests for toCoalCollisionObject()
// ============================================================================

TEST(PFObstacleCoal, ToCoalCollisionObject_Sphere) {
  // Create a sphere obstacle
  pfield::PotentialFieldObstacle sphere(
    "test_sphere",
    Eigen::Vector3d(1.0, 2.0, 3.0),
    Eigen::Quaterniond::Identity(),
    pfield::ObstacleGroup::STATIC,
    std::make_unique<pfield::SphereGeometry>(0.5)
  );

  // Convert to COAL collision object
  auto coalObj = sphere.toCoalCollisionObject();
  ASSERT_NE(coalObj, nullptr);

  // Check transform
  const auto& transform = coalObj->getTransform();
  EXPECT_NEAR(transform.getTranslation()[0], 1.0, 1e-6);
  EXPECT_NEAR(transform.getTranslation()[1], 2.0, 1e-6);
  EXPECT_NEAR(transform.getTranslation()[2], 3.0, 1e-6);

  // Check rotation (identity quaternion)
  const auto& rotation = transform.getRotation();
  EXPECT_NEAR(rotation(0, 0), 1.0, 1e-6);
  EXPECT_NEAR(rotation(1, 1), 1.0, 1e-6);
  EXPECT_NEAR(rotation(2, 2), 1.0, 1e-6);

  // Check geometry type (should be Sphere)
  EXPECT_NE(coalObj->collisionGeometry(), nullptr);
}

TEST(PFObstacleCoal, ToCoalCollisionObject_Box) {
  // Create a box obstacle
  pfield::PotentialFieldObstacle box(
    "test_box",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    pfield::ObstacleGroup::STATIC,
    std::make_unique<pfield::BoxGeometry>(2.0, 1.0, 0.5)
  );

  auto coalObj = box.toCoalCollisionObject();
  ASSERT_NE(coalObj, nullptr);
  EXPECT_NE(coalObj->collisionGeometry(), nullptr);
}

TEST(PFObstacleCoal, ToCoalCollisionObject_Cylinder) {
  // Create a cylinder obstacle
  pfield::PotentialFieldObstacle cylinder(
    "test_cylinder",
    Eigen::Vector3d(-1.0, -2.0, -3.0),
    Eigen::Quaterniond::Identity(),
    pfield::ObstacleGroup::STATIC,
    std::make_unique<pfield::CylinderGeometry>(0.3, 1.2)
  );

  auto coalObj = cylinder.toCoalCollisionObject();
  ASSERT_NE(coalObj, nullptr);

  // Check transform
  const auto& transform = coalObj->getTransform();
  EXPECT_NEAR(transform.getTranslation()[0], -1.0, 1e-6);
  EXPECT_NEAR(transform.getTranslation()[1], -2.0, 1e-6);
  EXPECT_NEAR(transform.getTranslation()[2], -3.0, 1e-6);
}

// ============================================================================
// Tests for ELLIPSOID obstacle type
// ============================================================================

// Helper to build an axis-aligned ellipsoid at the origin
static pfield::PotentialFieldObstacle makeEllipsoid(double sx, double sy, double sz) {
  return pfield::PotentialFieldObstacle(
    "test_ellipsoid",
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(),
    pfield::ObstacleGroup::ROBOT,
    std::make_unique<pfield::EllipsoidGeometry>(sx, sy, sz)
  );
}

TEST(PFObstacleEllipsoid, CoalObjectCreated) {
  auto ellipsoid = makeEllipsoid(0.1, 0.2, 0.3);
  auto coalObj = ellipsoid.toCoalCollisionObject();
  ASSERT_NE(coalObj, nullptr);
  EXPECT_NE(coalObj->collisionGeometry(), nullptr);
}

TEST(PFObstacleEllipsoid, PointOutside_PositiveSignedDistance) {
  // Semi-axes: a=0.1, b=0.2, c=0.3. Point at (0.5, 0, 0) is outside.
  auto ellipsoid = makeEllipsoid(0.1, 0.2, 0.3);
  double dist;
  Eigen::Vector3d normal;
  ellipsoid.computeSignedDistanceAndNormal(Eigen::Vector3d(0.5, 0.0, 0.0), dist, normal);
  EXPECT_GT(dist, 0.0);
}

TEST(PFObstacleEllipsoid, PointInside_NegativeSignedDistance) {
  // Semi-axes: a=0.5, b=0.5, c=0.5. Point at (0.1, 0.1, 0.1) is inside.
  auto ellipsoid = makeEllipsoid(0.5, 0.5, 0.5);
  double dist;
  Eigen::Vector3d normal;
  ellipsoid.computeSignedDistanceAndNormal(Eigen::Vector3d(0.1, 0.1, 0.1), dist, normal);
  EXPECT_LT(dist, 0.0);
}

TEST(PFObstacleEllipsoid, PointOnSurface_NearZeroSignedDistance) {
  // For a=1, b=1, c=1 (sphere), the surface is at radius=1. Point exactly on surface.
  auto ellipsoid = makeEllipsoid(1.0, 1.0, 1.0);
  double dist;
  Eigen::Vector3d normal;
  ellipsoid.computeSignedDistanceAndNormal(Eigen::Vector3d(1.0, 0.0, 0.0), dist, normal);
  EXPECT_NEAR(dist, 0.0, 1e-9);
}

TEST(PFObstacleEllipsoid, NormalPointsOutward_OnAxis) {
  // For a point on the +X axis outside the ellipsoid, normal should point in +X direction.
  auto ellipsoid = makeEllipsoid(0.3, 0.2, 0.1);
  double dist;
  Eigen::Vector3d normal;
  ellipsoid.computeSignedDistanceAndNormal(Eigen::Vector3d(1.0, 0.0, 0.0), dist, normal);
  EXPECT_GT(dist, 0.0);
  EXPECT_NEAR(normal.x(), 1.0, 1e-6);
  EXPECT_NEAR(normal.y(), 0.0, 1e-6);
  EXPECT_NEAR(normal.z(), 0.0, 1e-6);
}

TEST(PFObstacleEllipsoid, NormalIsUnitVector) {
  auto ellipsoid = makeEllipsoid(0.2, 0.4, 0.3);
  double dist;
  Eigen::Vector3d normal;
  ellipsoid.computeSignedDistanceAndNormal(Eigen::Vector3d(0.5, 0.5, 0.5), dist, normal);
  EXPECT_NEAR(normal.norm(), 1.0, 1e-6);
}

TEST(PFObstacleEllipsoid, SmoothDistance_MonotonicAlongAxis) {
  // Signed distance should increase monotonically as we move away along X.
  auto ellipsoid = makeEllipsoid(0.3, 0.2, 0.1);
  const std::vector<double> xs = {0.5, 1.0, 1.5, 2.0};
  double prevDist = -1e9;
  for (double x : xs) {
    double dist;
    Eigen::Vector3d normal;
    ellipsoid.computeSignedDistanceAndNormal(Eigen::Vector3d(x, 0.0, 0.0), dist, normal);
    EXPECT_GT(dist, prevDist);
    prevDist = dist;
  }
}

TEST(PFObstacleEllipsoid, HalfDimensions_ReturnsSemiAxes) {
  auto ellipsoid = makeEllipsoid(0.1, 0.2, 0.3);
  Eigen::Vector3d halfDims = ellipsoid.halfDimensions();
  EXPECT_NEAR(halfDims.x(), 0.1, 1e-9);
  EXPECT_NEAR(halfDims.y(), 0.2, 1e-9);
  EXPECT_NEAR(halfDims.z(), 0.3, 1e-9);
}

TEST(PFObstacleEllipsoid, WithinInfluenceZone_OutsideInfluence) {
  auto ellipsoid = makeEllipsoid(0.1, 0.1, 0.1);
  // Point at (1.0, 0, 0): distance to surface ~0.9, which is >> influence 0.05
  EXPECT_FALSE(ellipsoid.withinInfluenceZone(Eigen::Vector3d(1.0, 0.0, 0.0), 0.05));
}

TEST(PFObstacleEllipsoid, WithinInfluenceZone_InsideInfluence) {
  auto ellipsoid = makeEllipsoid(0.1, 0.1, 0.1);
  // Point at (0.15, 0, 0): just outside surface (surface at 0.1), distance ~0.05
  EXPECT_TRUE(ellipsoid.withinInfluenceZone(Eigen::Vector3d(0.15, 0.0, 0.0), 0.1));
}

TEST(PFObstacleEllipsoid, TypeAndGroupAreCorrect) {
  auto ellipsoid = makeEllipsoid(0.1, 0.2, 0.3);
  EXPECT_EQ(ellipsoid.getType(), pfield::ObstacleType::ELLIPSOID);
  EXPECT_EQ(ellipsoid.getGroup(), pfield::ObstacleGroup::ROBOT);
}

TEST(PFObstacleEllipsoid, TypeToStringRoundTrip) {
  EXPECT_EQ(pfield::obstacleTypeToString(pfield::ObstacleType::ELLIPSOID), "Ellipsoid");
  EXPECT_EQ(pfield::stringToObstacleType("Ellipsoid"), pfield::ObstacleType::ELLIPSOID);
}

TEST(PFObstacleEllipsoid, PCARotated_PointOutside) {
  // Ellipsoid with semi-axes (0.5, 0.2, 0.1) rotated 90° around Z.
  // After rotation, the long axis (0.5) aligns with world Y instead of X.
  // A point at (0, 1.0, 0) in obstacle frame is along the rotated long axis — outside.
  Eigen::Matrix3d pcaAxes;
  // 90° rotation around Z: X→Y, Y→-X
  pcaAxes << 0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0;

  pfield::PotentialFieldObstacle ellipsoid(
    "pca_ellipsoid", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
    pfield::ObstacleGroup::ROBOT,
    std::make_unique<pfield::EllipsoidGeometry>(0.5, 0.2, 0.1, pcaAxes, Eigen::Vector3d::Zero())
  );

  double dist;
  Eigen::Vector3d normal;
  // Point at world (0, 1.0, 0): in PCA frame this is (1.0, 0, 0), along the 0.5 semi-axis → outside
  ellipsoid.computeSignedDistanceAndNormal(Eigen::Vector3d(0.0, 1.0, 0.0), dist, normal);
  EXPECT_GT(dist, 0.0);
}

TEST(PFObstacleEllipsoid, PCARotated_PointInside) {
  // Same rotated ellipsoid. A point at (0.1, 0, 0) is along the rotated short axis (0.2 in Y→X).
  // In PCA frame: pcaAxes^T * (0.1, 0, 0) = (0, 0.1, 0), which is along the 0.2 semi-axis → inside.
  Eigen::Matrix3d pcaAxes;
  pcaAxes << 0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0;

  pfield::PotentialFieldObstacle ellipsoid(
    "pca_ellipsoid", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
    pfield::ObstacleGroup::ROBOT,
    std::make_unique<pfield::EllipsoidGeometry>(0.5, 0.2, 0.1, pcaAxes, Eigen::Vector3d::Zero())
  );

  double dist;
  Eigen::Vector3d normal;
  ellipsoid.computeSignedDistanceAndNormal(Eigen::Vector3d(0.1, 0.0, 0.0), dist, normal);
  EXPECT_LT(dist, 0.0);
}

TEST(PFObstacleEllipsoid, CentroidOffset_ShiftsCenter) {
  // Ellipsoid centered at (1.0, 0, 0) in the obstacle frame (centroid_offset = (1.0, 0, 0)).
  // A point at (1.0, 0, 0) in obstacle frame should be at the center → inside.
  // A point at (0, 0, 0) in obstacle frame is (−1, 0, 0) relative to center → outside (semi_x=0.1).
  pfield::PotentialFieldObstacle ellipsoid(
    "offset_ellipsoid", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
    pfield::ObstacleGroup::ROBOT,
    std::make_unique<pfield::EllipsoidGeometry>(
      0.1, 0.1, 0.1, Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0))
  );

  double dist;
  Eigen::Vector3d normal;

  // Point at (1.0, 0, 0): at the ellipsoid center → inside
  ellipsoid.computeSignedDistanceAndNormal(Eigen::Vector3d(1.0, 0.0, 0.0), dist, normal);
  EXPECT_LT(dist, 0.0);

  // Point at (0.0, 0.0, 0.0): 1.0 from center, semi_x=0.1 → far outside
  ellipsoid.computeSignedDistanceAndNormal(Eigen::Vector3d(0.0, 0.0, 0.0), dist, normal);
  EXPECT_GT(dist, 0.0);
}

TEST(PFObstacleCoal, ToCoalCollisionObject_WithRotation) {
  // Create obstacle with non-identity rotation
  Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

  pfield::PotentialFieldObstacle sphere(
    "rotated_sphere",
    Eigen::Vector3d::Zero(),
    rotation,
    pfield::ObstacleGroup::STATIC,
    std::make_unique<pfield::SphereGeometry>(1.0)
  );

  auto coalObj = sphere.toCoalCollisionObject();
  ASSERT_NE(coalObj, nullptr);

  // Verify rotation was applied (check quaternion components)
  // COAL uses w,x,y,z order
  const auto& transform = coalObj->getTransform();
  const auto& rot = transform.getRotation();

  // The rotation matrix should not be identity
  bool isIdentity = (rot - Eigen::Matrix3d::Identity()).norm() < 1e-6;
  EXPECT_FALSE(isIdentity);
}
