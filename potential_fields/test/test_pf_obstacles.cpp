#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include "pfield/pf_obstacle.hpp"

// ============================================================================
// Tests for toCoalCollisionObject()
// ============================================================================

TEST(PFObstacleCoal, ToCoalCollisionObject_Sphere) {
  // Create a sphere obstacle
  PotentialFieldObstacle sphere(
    "test_sphere",
    Eigen::Vector3d(1.0, 2.0, 3.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.5, 0.0, 0.0, 0.0)  // radius = 0.5
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
  PotentialFieldObstacle box(
    "test_box",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::BOX,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.0, 2.0, 1.0, 0.5)  // length=2, width=1, height=0.5
  );

  auto coalObj = box.toCoalCollisionObject();
  ASSERT_NE(coalObj, nullptr);
  EXPECT_NE(coalObj->collisionGeometry(), nullptr);
}

TEST(PFObstacleCoal, ToCoalCollisionObject_Cylinder) {
  // Create a cylinder obstacle
  PotentialFieldObstacle cylinder(
    "test_cylinder",
    Eigen::Vector3d(-1.0, -2.0, -3.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::CYLINDER,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.3, 0.0, 0.0, 1.2)  // radius=0.3, height=1.2
  );

  auto coalObj = cylinder.toCoalCollisionObject();
  ASSERT_NE(coalObj, nullptr);

  // Check transform
  const auto& transform = coalObj->getTransform();
  EXPECT_NEAR(transform.getTranslation()[0], -1.0, 1e-6);
  EXPECT_NEAR(transform.getTranslation()[1], -2.0, 1e-6);
  EXPECT_NEAR(transform.getTranslation()[2], -3.0, 1e-6);
}

TEST(PFObstacleCoal, ToCoalCollisionObject_WithRotation) {
  // Create obstacle with non-identity rotation
  Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

  PotentialFieldObstacle sphere(
    "rotated_sphere",
    Eigen::Vector3d::Zero(),
    rotation,
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(1.0, 0.0, 0.0, 0.0)
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

// ============================================================================
// Tests for computeMinimumDistanceTo() - Sphere-Sphere
// ============================================================================

TEST(PFObstacleDistance, SphereSphere_Separated) {
  // Two spheres separated by distance
  PotentialFieldObstacle sphere1(
    "sphere1",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(1.0, 0.0, 0.0, 0.0)  // radius = 1.0
  );

  PotentialFieldObstacle sphere2(
    "sphere2",
    Eigen::Vector3d(5.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(1.0, 0.0, 0.0, 0.0)  // radius = 1.0
  );

  Eigen::Vector3d normal;
  double distance = sphere1.computeMinimumDistanceTo(sphere2, normal);

  // Distance should be 5.0 - 1.0 - 1.0 = 3.0
  EXPECT_NEAR(distance, 3.0, 1e-4);

  // Normal should point from sphere1 to sphere2 (along +X)
  EXPECT_NEAR(normal.x(), 1.0, 1e-4);
  EXPECT_NEAR(normal.y(), 0.0, 1e-4);
  EXPECT_NEAR(normal.z(), 0.0, 1e-4);
}

TEST(PFObstacleDistance, SphereSphere_Touching) {
  PotentialFieldObstacle sphere1(
    "sphere1",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(2.0, 0.0, 0.0, 0.0)
  );

  PotentialFieldObstacle sphere2(
    "sphere2",
    Eigen::Vector3d(3.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(1.0, 0.0, 0.0, 0.0)
  );

  Eigen::Vector3d normal;
  double distance = sphere1.computeMinimumDistanceTo(sphere2, normal);

  // Distance should be 3.0 - 2.0 - 1.0 = 0.0
  EXPECT_NEAR(distance, 0.0, 1e-4);
}

TEST(PFObstacleDistance, SphereSphere_Overlapping) {
  PotentialFieldObstacle sphere1(
    "sphere1",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(2.0, 0.0, 0.0, 0.0)
  );

  PotentialFieldObstacle sphere2(
    "sphere2",
    Eigen::Vector3d(2.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(1.5, 0.0, 0.0, 0.0)
  );

  Eigen::Vector3d normal;
  double distance = sphere1.computeMinimumDistanceTo(sphere2, normal);

  // Distance should be negative (penetration): 2.0 - 2.0 - 1.5 = -1.5
  EXPECT_LT(distance, 0.0);
  EXPECT_NEAR(distance, -1.5, 1e-3);
}

// ============================================================================
// Tests for computeMinimumDistanceTo() - Sphere-Box
// ============================================================================

TEST(PFObstacleDistance, SphereBox_Separated) {
  PotentialFieldObstacle sphere(
    "sphere",
    Eigen::Vector3d(3.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.5, 0.0, 0.0, 0.0)  // radius = 0.5
  );

  PotentialFieldObstacle box(
    "box",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::BOX,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.0, 2.0, 2.0, 2.0)  // 2x2x2 box, half-extents = 1.0
  );

  Eigen::Vector3d normal;
  double distance = sphere.computeMinimumDistanceTo(box, normal);

  // Box extends from -1 to 1 in X, sphere center at 3.0 with radius 0.5
  // Distance = 3.0 - 1.0 - 0.5 = 1.5
  EXPECT_NEAR(distance, 1.5, 1e-3);

  // Normal should point from sphere center toward box (negative X direction from sphere perspective)
  EXPECT_LT(normal.x(), 0.0);
}

TEST(PFObstacleDistance, BoxSphere_Separated) {
  // Test the reverse (box to sphere) to ensure symmetry
  PotentialFieldObstacle box(
    "box",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::BOX,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.0, 1.0, 1.0, 1.0)  // 1x1x1 box
  );

  PotentialFieldObstacle sphere(
    "sphere",
    Eigen::Vector3d(2.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.25, 0.0, 0.0, 0.0)
  );

  Eigen::Vector3d normal;
  double distance = box.computeMinimumDistanceTo(sphere, normal);

  // Box extends to 0.5 in +X, sphere starts at 1.75
  // Distance = 2.0 - 0.5 - 0.25 = 1.25
  EXPECT_NEAR(distance, 1.25, 1e-3);
}

// ============================================================================
// Tests for computeMinimumDistanceTo() - Sphere-Cylinder
// ============================================================================

TEST(PFObstacleDistance, SphereCylinder_Separated) {
  PotentialFieldObstacle sphere(
    "sphere",
    Eigen::Vector3d(2.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.3, 0.0, 0.0, 0.0)  // radius = 0.3
  );

  PotentialFieldObstacle cylinder(
    "cylinder",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::CYLINDER,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.5, 0.0, 0.0, 2.0)  // radius=0.5, height=2.0
  );

  Eigen::Vector3d normal;
  double distance = sphere.computeMinimumDistanceTo(cylinder, normal);

  // Cylinder extends to radius 0.5, sphere at 2.0 with radius 0.3
  // Distance = 2.0 - 0.5 - 0.3 = 1.2
  EXPECT_NEAR(distance, 1.2, 1e-3);
}

TEST(PFObstacleDistance, CylinderSphere_Separated) {
  PotentialFieldObstacle cylinder(
    "cylinder",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::CYLINDER,
    ObstacleGroup::STATIC,
    ObstacleGeometry(1.0, 0.0, 0.0, 3.0)  // radius=1.0, height=3.0
  );

  PotentialFieldObstacle sphere(
    "sphere",
    Eigen::Vector3d(0.0, 3.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.5, 0.0, 0.0, 0.0)
  );

  Eigen::Vector3d normal;
  double distance = cylinder.computeMinimumDistanceTo(sphere, normal);

  // Cylinder extends to radius 1.0 in Y, sphere at 3.0 with radius 0.5
  // Distance = 3.0 - 1.0 - 0.5 = 1.5
  EXPECT_NEAR(distance, 1.5, 1e-3);
}

// ============================================================================
// Tests for computeMinimumDistanceTo() - Box-Box
// ============================================================================

TEST(PFObstacleDistance, BoxBox_Separated) {
  PotentialFieldObstacle box1(
    "box1",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::BOX,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.0, 1.0, 1.0, 1.0)  // 1x1x1 box, half-extent = 0.5
  );

  PotentialFieldObstacle box2(
    "box2",
    Eigen::Vector3d(3.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::BOX,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.0, 1.0, 1.0, 1.0)
  );

  Eigen::Vector3d normal;
  double distance = box1.computeMinimumDistanceTo(box2, normal);

  // Box1 extends to 0.5, Box2 starts at 2.5
  // Distance = 2.5 - 0.5 = 2.0
  EXPECT_NEAR(distance, 2.0, 1e-3);
}

TEST(PFObstacleDistance, BoxBox_TouchingCorners) {
  // Two boxes touching at a corner
  PotentialFieldObstacle box1(
    "box1",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::BOX,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.0, 2.0, 2.0, 2.0)  // half-extent = 1.0
  );

  PotentialFieldObstacle box2(
    "box2",
    Eigen::Vector3d(2.0, 2.0, 2.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::BOX,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.0, 2.0, 2.0, 2.0)
  );

  Eigen::Vector3d normal;
  double distance = box1.computeMinimumDistanceTo(box2, normal);

  // Corner to corner distance
  // Box1 corner at (1,1,1), Box2 corner at (1,1,1) - they touch
  EXPECT_NEAR(distance, 0.0, 1e-2);
}

// ============================================================================
// Tests for computeMinimumDistanceTo() - Cylinder-Cylinder
// ============================================================================

TEST(PFObstacleDistance, CylinderCylinder_ParallelSeparated) {
  PotentialFieldObstacle cyl1(
    "cyl1",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::CYLINDER,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.5, 0.0, 0.0, 2.0)  // radius=0.5, height=2.0
  );

  PotentialFieldObstacle cyl2(
    "cyl2",
    Eigen::Vector3d(3.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::CYLINDER,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.5, 0.0, 0.0, 2.0)
  );

  Eigen::Vector3d normal;
  double distance = cyl1.computeMinimumDistanceTo(cyl2, normal);

  // Cylinders separated by 3.0 - 0.5 - 0.5 = 2.0
  EXPECT_NEAR(distance, 2.0, 1e-3);
}

// ============================================================================
// Tests for computeMinimumDistanceTo() - Mixed Complex Cases
// ============================================================================

TEST(PFObstacleDistance, BoxCylinder_Separated) {
  PotentialFieldObstacle box(
    "box",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::BOX,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.0, 1.0, 1.0, 1.0)
  );

  PotentialFieldObstacle cylinder(
    "cylinder",
    Eigen::Vector3d(0.0, 3.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::CYLINDER,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.5, 0.0, 0.0, 2.0)
  );

  Eigen::Vector3d normal;
  double distance = box.computeMinimumDistanceTo(cylinder, normal);

  // Box extends to 0.5 in Y, cylinder starts at 2.5 (3.0 - 0.5)
  // Distance = 2.5 - 0.5 = 2.0
  EXPECT_NEAR(distance, 2.0, 1e-2);
}

// ============================================================================
// Tests for Normal Vector Direction
// ============================================================================

TEST(PFObstacleDistance, NormalDirection_SphereSphere) {
  PotentialFieldObstacle sphere1(
    "sphere1",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(1.0, 0.0, 0.0, 0.0)
  );

  PotentialFieldObstacle sphere2(
    "sphere2",
    Eigen::Vector3d(0.0, 5.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(1.0, 0.0, 0.0, 0.0)
  );

  Eigen::Vector3d normal;
  sphere1.computeMinimumDistanceTo(sphere2, normal);

  // Normal should point from sphere1 to sphere2 (along +Y)
  EXPECT_NEAR(normal.normalized().x(), 0.0, 1e-4);
  EXPECT_NEAR(normal.normalized().y(), 1.0, 1e-4);
  EXPECT_NEAR(normal.normalized().z(), 0.0, 1e-4);
}

// ============================================================================
// Tests for Edge Cases
// ============================================================================

TEST(PFObstacleDistance, SameLocation_SphereSphere) {
  // Two spheres at exactly the same location
  PotentialFieldObstacle sphere1(
    "sphere1",
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(1.0, 0.0, 0.0, 0.0)
  );

  PotentialFieldObstacle sphere2(
    "sphere2",
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(1.0, 0.0, 0.0, 0.0)
  );

  Eigen::Vector3d normal;
  double distance = sphere1.computeMinimumDistanceTo(sphere2, normal);

  // Should be negative (fully overlapping)
  EXPECT_LT(distance, 0.0);

  // Normal should be well-defined (fallback to reasonable direction)
  EXPECT_NEAR(normal.norm(), 1.0, 1e-4);
}

TEST(PFObstacleDistance, VerySmallSpheres) {
  PotentialFieldObstacle sphere1(
    "tiny1",
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.001, 0.0, 0.0, 0.0)
  );

  PotentialFieldObstacle sphere2(
    "tiny2",
    Eigen::Vector3d(0.1, 0.0, 0.0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry(0.001, 0.0, 0.0, 0.0)
  );

  Eigen::Vector3d normal;
  double distance = sphere1.computeMinimumDistanceTo(sphere2, normal);

  // Distance should be approximately 0.1 - 0.001 - 0.001 = 0.098
  EXPECT_NEAR(distance, 0.098, 1e-4);
}
