#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include "pfield/pf_obstacle.hpp"

using namespace pfield;

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
