#include <gtest/gtest.h>
#include "pfield.hpp"
#include "sphere_obstacle.hpp"
#include "spatial_vector.hpp"
#include <eigen3/Eigen/Dense>

TEST(PotentialFieldTest, AddAndRemoveObstacles) {
  PotentialField pf;
  SphereObstacle o1(1, Eigen::Vector3d(1.0f, 1.0f, 1.0f), 1.0f, 2.0f, 10.0f);
  pf.addObstacle(o1);
  EXPECT_EQ(pf.getObstacles().size(), 1);

  // Overwrite same ID
  SphereObstacle o1b(1, Eigen::Vector3d(2.0f, 2.0f, 2.0f), 1.5f, 2.5f, 12.0f);
  pf.addObstacle(o1b);
  EXPECT_EQ(pf.getObstacles().size(), 1);
  EXPECT_EQ(pf.getObstacles()[0].getPosition().x(), 2.0f);
  EXPECT_EQ(pf.getObstacles()[0].getPosition().y(), 2.0f);
  EXPECT_EQ(pf.getObstacles()[0].getPosition().z(), 2.0f);
  EXPECT_EQ(pf.getObstacles()[0].getRadius(), 1.5f);
  EXPECT_EQ(pf.getObstacles()[0].getInfluenceRadius(), 2.5f);
  EXPECT_EQ(pf.getObstacles()[0].getRepulsiveGain(), 12.0f);

  // Add another and remove
  pf.addObstacle(SphereObstacle(2, Eigen::Vector3d(-1.0f, -1.0f, 0.0f), 1.0f, 2.0f, 8.0f));
  EXPECT_EQ(pf.getObstacles().size(), 2);
  EXPECT_TRUE(pf.removeObstacle(2));
  EXPECT_FALSE(pf.removeObstacle(99));  // nonexistent
  EXPECT_EQ(pf.getObstacles().size(), 1);
}

TEST(PotentialFieldTest, ClearObstacles) {
  PotentialField pf;
  pf.addObstacle(SphereObstacle(1, Eigen::Vector3d::Zero(), 1.0f, 2.0f, 10.0f));
  pf.addObstacle(SphereObstacle(2, Eigen::Vector3d::Zero(), 1.0f, 2.0f, 10.0f));
  pf.clearObstacles();
  EXPECT_TRUE(pf.getObstacles().empty());
}

TEST(PotentialFieldTest, VelocityAtGoalIsZero) {
  SpatialVector goal(Eigen::Vector3d(1.0f, 1.0f, 1.0f));
  PotentialField pf(goal, 1.0f, 0.0f);
  SpatialVector v = pf.evaluateVelocityAtPose(goal);
  EXPECT_EQ(v.getPosition(), Eigen::Vector3d::Zero());
  EXPECT_EQ(v.getOrientation(), Eigen::Quaterniond::Identity());
}

TEST(PotentialFieldTest, AttractiveFieldPullsTowardGoal) {
  SpatialVector goal;
  PotentialField pf(goal, 1.0f, 0.0f);
  SpatialVector query(Eigen::Vector3d(1.0f, 0.0f, 0.0f));
  SpatialVector vel = pf.evaluateVelocityAtPose(query);
  EXPECT_LT(vel.getPosition().x(), 0);  // should pull toward origin
  EXPECT_NEAR(vel.getPosition().y(), 0.0f, 1e-5);
  EXPECT_NEAR(vel.getPosition().z(), 0.0f, 1e-5);
}

TEST(PotentialFieldTest, AttractiveGainScaling) {
  SpatialVector goal(Eigen::Vector3d::Zero());
  PotentialField pf(goal, 3.0, 0.0);
  SpatialVector query(Eigen::Vector3d(2.0, 0.0, 0.0));
  SpatialVector vel = pf.evaluateVelocityAtPose(query);
  // magnitude = gain * distance = 3 * 2 = 6, direction toward goal: negative x
  EXPECT_NEAR(vel.getPosition().x(), -6.0, 1e-6);
  EXPECT_NEAR(vel.getPosition().y(), 0.0, 1e-6);
}

TEST(PotentialFieldTest, RepulsiveFieldPushesAwayFromObstacle) {
  Eigen::Vector3d goalPosition(0.0f, 0.0f, 0.0f);
  PotentialField pf(SpatialVector(goalPosition), 0.0f, 0.0f);  // no attraction
  pf.addObstacle(SphereObstacle(0, goalPosition, 0.5f, 2.0f, 10.0f));
  SpatialVector query(Eigen::Vector3d(1.0, 0.0, 0.0));
  SpatialVector vel = pf.evaluateVelocityAtPose(query);
  EXPECT_GT(vel.getPosition().x(), 0);  // push away from origin
  EXPECT_NEAR(vel.getPosition().y(), 0.0f, 1e-5);
  EXPECT_NEAR(vel.getPosition().z(), 0.0f, 1e-5);
}

TEST(PotentialFieldTest, NearZeroDistanceAttraction) {
  PotentialField pf(SpatialVector(Eigen::Vector3d(1.0f, 1.0f, 1.0f)), 1.0f, 0.0f);
  SpatialVector nearGoal(Eigen::Vector3d(1 + 1e-3f, 1.0f, 1.0f));
  SpatialVector vel = pf.evaluateVelocityAtPose(nearGoal);
  EXPECT_GT(std::abs(vel.getPosition().x()), 1e-3f);  // Should be small but not zero
  EXPECT_NEAR(vel.getPosition().y(), 0.0f, 1e-6f);
  EXPECT_NEAR(vel.getPosition().z(), 0.0f, 1e-6f);
}

TEST(PotentialFieldTest, RepulsionAtSurfaceBoundary) {
  PotentialField pf(SpatialVector(Eigen::Vector3d(10.0f, 10.0f, 10.0f)), 0.0f, 0.0f);
  SphereObstacle obs(1, Eigen::Vector3d::Zero(), 1.0f, 2.0f, 10.0f);
  pf.addObstacle(obs);
  SpatialVector query(Eigen::Vector3d(2.0f, 0.0f, 0.0f));  // At influence radius
  SpatialVector v = pf.evaluateVelocityAtPose(query);
  EXPECT_NEAR(v.getPosition().x(), 0.0f, 1e-6f);  // At boundary, repulsive force should approach 0
  SpatialVector query2(Eigen::Vector3d(2.1f, 0.0f, 0.0f));  // Outside influence radius
  SpatialVector v2 = pf.evaluateVelocityAtPose(query2);
  EXPECT_NEAR(v2.getPosition().x(), 0.0f, 1e-6f);
  SpatialVector query3(Eigen::Vector3d(1.9f, 0.0f, 0.0f));  // Inside influence radius
  SpatialVector v3 = pf.evaluateVelocityAtPose(query3);
  EXPECT_GT(v3.getPosition().x(), 0.0f);  // Should be pushing away from obstacle
}

TEST(PotentialFieldTest, RepulsionMonotonicity) {
  SpatialVector goal; // Default goal at origin
  PotentialField pf(goal, 0.0, 0.0);
  SphereObstacle obs(1, Eigen::Vector3d::Zero(), 0.5, 4.0, 2.0);
  pf.addObstacle(obs);
  SpatialVector q1(Eigen::Vector3d(1.0, 0.0, 0.0));
  SpatialVector q2(Eigen::Vector3d(2.0, 0.0, 0.0));
  auto v1 = pf.evaluateVelocityAtPose(q1);
  auto v2 = pf.evaluateVelocityAtPose(q2);
  // both push along +x; closer point should see stronger force
  EXPECT_GT(v1.getPosition().x(), v2.getPosition().x());
}

TEST(PotentialExtensiveTest, SymmetricObstaclesCancelAxes) {
  SpatialVector goal; // Default goal at origin
  PotentialField pf(goal, 0.0, 0.0);
  pf.addObstacle(SphereObstacle(1, Eigen::Vector3d(-1, 0, 0), 0.5, 3.0, 5.0));
  pf.addObstacle(SphereObstacle(2, Eigen::Vector3d(1, 0, 0), 0.5, 3.0, 5.0));
  SpatialVector q(Eigen::Vector3d(0, 2.0, 0));  // directly above both
  auto vel = pf.evaluateVelocityAtPose(q);
  // x‐components should cancel, only y‐component remains
  EXPECT_NEAR(vel.getPosition().x(), 0.0, 1e-5);
  EXPECT_GT(vel.getPosition().y(), 0.0);
}


TEST(PotentialFieldTest, RotationalAttraction) {
  SpatialVector goal;
  SpatialVector query; // Same position, different orientation
  goal.setOrientationEuler(0, 0, 0);
  query.setOrientationEuler(0, 0, M_PI_2);  // 90 degrees yaw 

  PotentialField pf(goal, 0.0f, 10.0f);  // No translation pull
  SpatialVector vel = pf.evaluateVelocityAtPose(query);
  EXPECT_NEAR(vel.getPosition().x(), 0.0f, 1e-3);
  EXPECT_NEAR(vel.getPosition().y(), 0.0f, 1e-3);
  EXPECT_NEAR(vel.getPosition().z(), 0.0f, 1e-3);
  EXPECT_NEAR(vel.getOrientation().x(), 0.0f, 1e-3);
  EXPECT_NEAR(vel.getOrientation().y(), 0.0f, 1e-3);
  EXPECT_NEAR(vel.getOrientation().z(), 0.0f, 1e-3);
  EXPECT_NEAR(vel.getOrientation().w(), -1.0f, 1e-3);  // Should be rotating towards goal
}

TEST(PotentialFieldTest, TranslationAndRotation) {
  SpatialVector goal(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  SpatialVector query(Eigen::Vector3d(1.0, 0.0, 0.0));
  query.setOrientationEuler(0, 0, M_PI_2);
  PotentialField pf(goal, 1.0, 10.0);
  auto vel = pf.evaluateVelocityAtPose(query);
  // translational: pulls toward origin (x < 0)
  EXPECT_LT(vel.getPosition().x(), 0.0);
  // rotational: orients toward identity, so quaternion should have a negative w‐component after multiplication
  EXPECT_LT(vel.getOrientation().w(), 0.0);
}



