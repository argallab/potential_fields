#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include "pfield/pfield.hpp"
#include "pfield/pf_obstacle.hpp"
#include "pfield/spatial_vector.hpp"

TEST(PotentialFieldTest, AddAndRemoveObstacles) {
  PotentialField pf;
  PotentialFieldObstacle o1(
    "o1",
    Eigen::Vector3d(1.0f, 1.0f, 1.0f),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry{1.0f, 0.0f, 0.0f, 0.0f},
    2.0f,
    10.0f
  );
  pf.addObstacle(o1);
  EXPECT_EQ(pf.getObstacles().size(), 1);

  // Overwrite same ID
  PotentialFieldObstacle o1b(
    "o1",
    Eigen::Vector3d(2.0f, 2.0f, 2.0f),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry{1.5f, 0.0f, 0.0f, 0.0f},
    2.5f,
    12.0f
  );
  pf.addObstacle(o1b);
  EXPECT_EQ(pf.getObstacles().size(), 1);
  EXPECT_EQ(pf.getObstacles()[0].getType(), ObstacleType::SPHERE);
  EXPECT_EQ(pf.getObstacles()[0].getPosition().x(), 2.0f);
  EXPECT_EQ(pf.getObstacles()[0].getPosition().y(), 2.0f);
  EXPECT_EQ(pf.getObstacles()[0].getPosition().z(), 2.0f);
  EXPECT_EQ(pf.getObstacles()[0].getGeometry().radius, 1.5f);
  EXPECT_EQ(pf.getObstacles()[0].getInfluenceZoneScale(), 2.5f);
  EXPECT_EQ(pf.getObstacles()[0].getRepulsiveGain(), 12.0f);

  // Add another and remove
  pf.addObstacle(PotentialFieldObstacle(
    "o2",
    Eigen::Vector3d(-1.0f, -1.0f, 0.0f),
    Eigen::Quaterniond::Identity(),
    ObstacleType::BOX,
    ObstacleGroup::STATIC,
    ObstacleGeometry{1.0f, 2.0f, 8.0f, 1.0f},
    2.0f,
    8.0f
  ));
  EXPECT_EQ(pf.getObstacles().size(), 2);
  EXPECT_TRUE(pf.removeObstacle("o2"));
  EXPECT_FALSE(pf.removeObstacle("o99"));  // nonexistent
  EXPECT_EQ(pf.getObstacles().size(), 1);
}

TEST(PotentialFieldTest, ClearObstacles) {
  PotentialField pf;
  pf.addObstacle(PotentialFieldObstacle(
    "o1",
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry{2.0, 0.0, 0.0, 0.0},
    2.0,
    10.0f
  ));
  pf.addObstacle(PotentialFieldObstacle(
    "o2",
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry{2.0, 0.0, 0.0, 0.0},
    2.0,
    10.0f
  ));
  pf.clearObstacles();
  EXPECT_TRUE(pf.getObstacles().empty());
}

TEST(PotentialFieldTest, VelocityAtGoalIsZero) {
  SpatialVector goal(Eigen::Vector3d(1.0f, 1.0f, 1.0f));
  PotentialField pf(goal, 1.0f, 0.0f);
  TaskSpaceTwist v = pf.evaluateVelocityAtPose(goal);
  EXPECT_EQ(v.getLinearVelocity(), Eigen::Vector3d::Zero());
  EXPECT_EQ(v.getAngularVelocity(), Eigen::Vector3d::Zero());
}

TEST(PotentialFieldTest, AttractiveFieldPullsTowardGoal) {
  SpatialVector goal;
  PotentialField pf(goal, 1.0f, 0.0f);
  SpatialVector query(Eigen::Vector3d(1.0f, 0.0f, 0.0f));
  TaskSpaceTwist vel = pf.evaluateVelocityAtPose(query);
  EXPECT_LT(vel.getLinearVelocity().x(), 0);  // should pull toward origin
  EXPECT_NEAR(vel.getLinearVelocity().y(), 0.0f, 1e-5);
  EXPECT_NEAR(vel.getLinearVelocity().z(), 0.0f, 1e-5);
}

TEST(PotentialFieldTest, AttractiveGainScaling) {
  SpatialVector goal(Eigen::Vector3d::Zero());
  PotentialField pf(goal, 3.0, 0.0);
  SpatialVector query(Eigen::Vector3d(2.0, 0.0, 0.0));
  TaskSpaceTwist vel = pf.evaluateVelocityAtPose(query);
  // magnitude = gain * distance = 3 * 2 = 6, direction toward goal: negative x
  EXPECT_NEAR(vel.getLinearVelocity().x(), -6.0, 1e-6);
  EXPECT_NEAR(vel.getLinearVelocity().y(), 0.0, 1e-6);
}

TEST(PotentialFieldTest, RepulsiveFieldPushesAwayFromObstacle) {
  Eigen::Vector3d goalPosition(0.0f, 0.0f, 0.0f);
  PotentialField pf(SpatialVector(goalPosition), 0.0f, 0.0f);  // no attraction
  auto sphereObst = PotentialFieldObstacle(
    "o0",
    goalPosition,
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry{2.0, 0.0, 0.0, 0.0},
    2.0f,
    10.0f
  );
  auto boxObst = PotentialFieldObstacle(
    "o1",
    goalPosition,
    Eigen::Quaterniond::Identity(),
    ObstacleType::BOX,
    ObstacleGroup::STATIC,
    ObstacleGeometry{2.0, 2.0, 2.0, 0.0},
    2.0f,
    10.0f
  );
  auto cylinderObst = PotentialFieldObstacle(
    "o2",
    goalPosition,
    Eigen::Quaterniond::Identity(),
    ObstacleType::CYLINDER,
    ObstacleGroup::STATIC,
    ObstacleGeometry{1.0, 0.0, 0.0, 2.0},
    2.0f,
    10.0f
  );
  pf.addObstacle(sphereObst);
  SpatialVector query(Eigen::Vector3d(1.0, 0.0, 0.0));
  TaskSpaceTwist vel = pf.evaluateVelocityAtPose(query);
  EXPECT_GT(vel.getLinearVelocity().x(), 0);  // push away from origin
  EXPECT_NEAR(vel.getLinearVelocity().y(), 0.0f, 1e-5);
  EXPECT_NEAR(vel.getLinearVelocity().z(), 0.0f, 1e-5);
  pf.addObstacle(boxObst);
  SpatialVector query2(Eigen::Vector3d(0.0, 1.0, 0.0));
  TaskSpaceTwist vel2 = pf.evaluateVelocityAtPose(query2);
  EXPECT_GT(vel2.getLinearVelocity().y(), 0);  // push away from origin
  EXPECT_NEAR(vel2.getLinearVelocity().x(), 0.0f, 1e-5);
  EXPECT_NEAR(vel2.getLinearVelocity().z(), 0.0f, 1e-5);
  pf.addObstacle(cylinderObst);
  SpatialVector query3(Eigen::Vector3d(0.0, 0.0, 1.0));
  TaskSpaceTwist vel3 = pf.evaluateVelocityAtPose(query3);
  EXPECT_GT(vel3.getLinearVelocity().z(), 0);  // push away from origin
  EXPECT_NEAR(vel3.getLinearVelocity().x(), 0.0f, 1e-5);
  EXPECT_NEAR(vel3.getLinearVelocity().y(), 0.0f, 1e-5);
}

TEST(PotentialFieldTest, BoxWithinObstacleAxisAligned) {
  Eigen::Vector3d center(1, 2, 3);
  ObstacleGeometry geom{0.0, 2.0, 4.0, 6.0};  // length=2, width=4, height=6
  PotentialFieldObstacle box(
    "o1", center, Eigen::Quaterniond::Identity(),
    ObstacleType::BOX, ObstacleGroup::STATIC, geom, 1.0, 1.0);
  // Points exactly on faces and inside
  EXPECT_TRUE(box.withinObstacle(Eigen::Vector3d(2.0, 2, 3)));  // +x face
  EXPECT_TRUE(box.withinObstacle(Eigen::Vector3d(0.0, 2, 3)));  // -x face
  EXPECT_TRUE(box.withinObstacle(Eigen::Vector3d(1, 4, 3)));    // +y face
  EXPECT_TRUE(box.withinObstacle(Eigen::Vector3d(1, 0, 3)));    // -y face
  EXPECT_FALSE(box.withinObstacle(Eigen::Vector3d(1, 5, 3)));   // outside +y
}

TEST(PotentialFieldTest, BoxWithinInfluenceZoneAxisAligned) {
  Eigen::Vector3d center(0, 0, 0);
  ObstacleGeometry geom{0.0, 1.0, 1.0, 1.0};
  PotentialFieldObstacle box(
    "o1", center, Eigen::Quaterniond::Identity(),
    ObstacleType::BOX, ObstacleGroup::STATIC, geom, 2.0, 1.0);
  // half-dims = (1,1,1)/2 * scale = (1,1,1)
  EXPECT_TRUE(box.withinInfluenceZone(Eigen::Vector3d(1.0, 0.5, 0.0)));
  EXPECT_FALSE(box.withinInfluenceZone(Eigen::Vector3d(1.1, 0.0, 0.0)));
}

TEST(PotentialFieldTest, BoxWithinObstacleRotated) {
  Eigen::Vector3d center(0, 0, 0);
  // rotate box 45 about Z
  Eigen::AngleAxisd rot(M_PI / 4, Eigen::Vector3d::UnitZ());
  ObstacleGeometry geom{0.0, 2.0, 2.0, 2.0};
  PotentialFieldObstacle box(
    "o1", center, Eigen::Quaterniond(rot),
    ObstacleType::BOX, ObstacleGroup::STATIC, geom, 1.0, 1.0);
  // In world frame, a local-axis-aligned point (1,0,0) maps to (cos45,sin45,0)
  Eigen::Vector3d query = rot * Eigen::Vector3d(1.0, 0, 0);
  EXPECT_TRUE(box.withinObstacle(query));
  // a point just outside local x=1 face maps out slightly
  Eigen::Vector3d out = rot * Eigen::Vector3d(1.1, 0, 0);
  EXPECT_FALSE(box.withinObstacle(out));
}

TEST(PotentialFieldTest, CylinderWithinObstacleAxisAligned) {
  Eigen::Vector3d center(5, -5, 0);
  ObstacleGeometry geom{3.0, 0.0, 0.0, 4.0};  // radius=3, height=4
  PotentialFieldObstacle cyl(
    "o1", center, Eigen::Quaterniond::Identity(),
    ObstacleType::CYLINDER, ObstacleGroup::STATIC, geom, 1.0, 1.0);
  // radial and z within half-height=2
  EXPECT_TRUE(cyl.withinObstacle(Eigen::Vector3d(8, -5, 1.0)));
  EXPECT_FALSE(cyl.withinObstacle(Eigen::Vector3d(9, -5, 1.0)));   // outside radius
  EXPECT_FALSE(cyl.withinObstacle(Eigen::Vector3d(5, -5, 3.0)));   // outside height
}

TEST(PotentialFieldTest, CylinderWithinInfluenceZoneAxisAligned) {
  Eigen::Vector3d center(0, 0, 0);
  ObstacleGeometry geom{1.0, 0.0, 0.0, 2.0};
  PotentialFieldObstacle cyl(
    "o1", center, Eigen::Quaterniond::Identity(),
    ObstacleType::CYLINDER, ObstacleGroup::STATIC, geom, 3.0, 1.0);
  // effective radius=3, half-height=3
  EXPECT_TRUE(cyl.withinInfluenceZone(Eigen::Vector3d(2.9, 0, 0)));
  EXPECT_FALSE(cyl.withinInfluenceZone(Eigen::Vector3d(3.1, 0, 0)));
}

TEST(PotentialFieldTest, CylinderWithinObstacleRotated) {
  Eigen::Vector3d center(0, 0, 0);
  // rotate cylinder so its axis tilts 90 into Y
  Eigen::AngleAxisd tilt(M_PI / 2, Eigen::Vector3d::UnitX());
  ObstacleGeometry geom{2.0, 0.0, 0.0, 4.0};  // radius=2, height=4
  PotentialFieldObstacle cyl(
    "o1", center, Eigen::Quaterniond(tilt),
    ObstacleType::CYLINDER, ObstacleGroup::STATIC, geom, 1.0, 1.0);
  // In local frame this point is on the +Z face; after tilt it's along +Y
  Eigen::Vector3d worldPoint = tilt * Eigen::Vector3d(0, 0, 2.0);
  EXPECT_TRUE(cyl.withinObstacle(worldPoint));
  // Outside along that axis
  Eigen::Vector3d outside = tilt * Eigen::Vector3d(0, 0, 2.1);
  EXPECT_FALSE(cyl.withinObstacle(outside));
}

TEST(PotentialFieldTest, NearZeroDistanceAttraction) {
  PotentialField pf(SpatialVector(Eigen::Vector3d(1.0f, 1.0f, 1.0f)), 1.0f, 0.0f);
  // Use an offset slightly larger than translationalTolerance to avoid zeroing
  SpatialVector nearGoal(Eigen::Vector3d(1 + 2e-3f, 1.0f, 1.0f));
  TaskSpaceTwist vel = pf.evaluateVelocityAtPose(nearGoal);
  EXPECT_GT(std::abs(vel.getLinearVelocity().x()), 1e-3f);  // Should be small but not zero
  EXPECT_NEAR(vel.getLinearVelocity().y(), 0.0f, 1e-6f);
  EXPECT_NEAR(vel.getLinearVelocity().z(), 0.0f, 1e-6f);
}

TEST(PotentialFieldTest, RepulsionAtSurfaceBoundary) {
  PotentialField pf(SpatialVector(Eigen::Vector3d(10.0f, 10.0f, 10.0f)), 0.0f, 0.0f);
  PotentialFieldObstacle obs(
    "o1",
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry{1.0, 0.0, 0.0, 0.0},
    2.0f,
    10.0f
  );
  pf.addObstacle(obs);
  SpatialVector query(Eigen::Vector3d(2.0f, 0.0f, 0.0f));  // At influence radius
  TaskSpaceTwist v = pf.evaluateVelocityAtPose(query);
  EXPECT_NEAR(v.getLinearVelocity().x(), 0.0f, 1e-6f);  // At boundary, repulsive force should approach 0
  SpatialVector query2(Eigen::Vector3d(2.1f, 0.0f, 0.0f));  // Outside influence radius
  TaskSpaceTwist v2 = pf.evaluateVelocityAtPose(query2);
  EXPECT_NEAR(v2.getLinearVelocity().x(), 0.0f, 1e-6f);
  SpatialVector query3(Eigen::Vector3d(1.9f, 0.0f, 0.0f));  // Inside influence radius
  TaskSpaceTwist v3 = pf.evaluateVelocityAtPose(query3);
  EXPECT_GT(v3.getLinearVelocity().x(), 0.0f);  // Should be pushing away from obstacle
}

TEST(PotentialFieldTest, RepulsionMonotonicity) {
  SpatialVector goal; // Default goal at origin
  PotentialField pf(goal, 0.0, 0.0);
  PotentialFieldObstacle obs(
    "o1",
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry{0.5, 0.0, 0.0, 0.0},
    4.0,
    2.0
  );
  pf.addObstacle(obs);
  SpatialVector q1(Eigen::Vector3d(1.0, 0.0, 0.0));
  SpatialVector q2(Eigen::Vector3d(2.0, 0.0, 0.0));
  auto v1 = pf.evaluateVelocityAtPose(q1);
  auto v2 = pf.evaluateVelocityAtPose(q2);
  // both push along +x; closer point should see stronger force
  EXPECT_GT(v1.getLinearVelocity().x(), v2.getLinearVelocity().x());
}

TEST(PotentialFieldTest, SymmetricObstaclesCancelAxes) {
  SpatialVector goal; // Default goal at origin
  PotentialField pf(goal, 0.0, 0.0);
  pf.addObstacle(PotentialFieldObstacle(
    "o1",
    Eigen::Vector3d(-1, 0, 0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry{0.5, 0.0, 0.0, 0.0},
    6.0,
    5.0
  ));
  pf.addObstacle(PotentialFieldObstacle(
    "o2",
    Eigen::Vector3d(1, 0, 0),
    Eigen::Quaterniond::Identity(),
    ObstacleType::SPHERE,
    ObstacleGroup::STATIC,
    ObstacleGeometry{0.5, 0.0, 0.0, 0.0},
    6.0,
    5.0
  ));
  SpatialVector q(Eigen::Vector3d(0, 2.0, 0));  // directly above both
  auto vel = pf.evaluateVelocityAtPose(q);
  // x‐components should cancel, only y‐component remains
  EXPECT_NEAR(vel.getLinearVelocity().x(), 0.0, 1e-5);
  EXPECT_GT(vel.getLinearVelocity().y(), 0.0);
}


TEST(PotentialFieldTest, RotationalAttraction) {
  SpatialVector goal;
  SpatialVector query; // Same position, different orientation
  goal.setOrientationEuler(0, 0, 0);
  query.setOrientationEuler(0, 0, M_PI_2);  // 90 degrees yaw

  PotentialField pf(goal, 0.0f, 10.0f);  // No translation pull
  TaskSpaceTwist vel = pf.evaluateVelocityAtPose(query);
  // Linear velocity should be ~0 (positions are equal)
  EXPECT_NEAR(vel.getLinearVelocity().x(), 0.0f, 1e-3);
  EXPECT_NEAR(vel.getLinearVelocity().y(), 0.0f, 1e-3);
  EXPECT_NEAR(vel.getLinearVelocity().z(), 0.0f, 1e-3);
  // Angular velocity should be about +Z for a +90deg yaw error, with magnitude gain * angle
  EXPECT_NEAR(vel.getAngularVelocity().x(), 0.0f, 1e-3);
  EXPECT_NEAR(vel.getAngularVelocity().y(), 0.0f, 1e-3);
  EXPECT_GT(vel.getAngularVelocity().z(), 0.0f);
  EXPECT_NEAR(vel.getAngularVelocity().norm(), 10.0 * (M_PI / 2), 1e-2);
}

TEST(PotentialFieldTest, TranslationAndRotation) {
  SpatialVector goal(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  SpatialVector query(Eigen::Vector3d(1.0, 0.0, 0.0));
  query.setOrientationEuler(0, 0, M_PI_2);
  PotentialField pf(goal, 1.0, 10.0);
  auto vel = pf.evaluateVelocityAtPose(query);
  // translational: pulls toward origin (x < 0)
  EXPECT_LT(vel.getLinearVelocity().x(), 0.0);
  // rotational: should produce non-zero angular velocity about Z toward the goal
  EXPECT_GT(vel.getAngularVelocity().norm(), 0.0);
}
