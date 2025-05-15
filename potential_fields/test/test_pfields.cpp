#include <gtest/gtest.h>
#include "pfield.hpp"
#include "sphere_obstacle.hpp"
#include "spatial_vector.hpp"

TEST(PotentialFieldTest, UpdateGoalPositionAndGain) {
  PotentialField pf;
  SpatialVector goal(1.0f, 2.0f, 3.0f);
  pf.updateGoalPosition(goal);
  EXPECT_EQ(pf.getGoalPosition(), goal);
}

TEST(PotentialFieldTest, AddAndRemoveObstacles) {
  PotentialField pf;
  SphereObstacle o1(1, SpatialVector(1, 1, 1), 1.0f, 2.0f, 10.0f);
  pf.addObstacle(o1);
  EXPECT_EQ(pf.getObstacles().size(), 1);

  // Overwrite same ID
  SphereObstacle o1b(1, SpatialVector(2, 2, 2), 1.5f, 2.5f, 12.0f);
  pf.addObstacle(o1b);
  EXPECT_EQ(pf.getObstacles().size(), 1);
  EXPECT_EQ(pf.getObstacles()[0].getPosition().getX(), 2.0f);

  // Add another and remove
  pf.addObstacle(SphereObstacle(2, SpatialVector(-1, -1, 0), 1.0f, 2.0f, 8.0f));
  EXPECT_EQ(pf.getObstacles().size(), 2);
  EXPECT_TRUE(pf.removeObstacle(2));
  EXPECT_FALSE(pf.removeObstacle(99));  // nonexistent
  EXPECT_EQ(pf.getObstacles().size(), 1);
}

TEST(PotentialFieldTest, ClearObstacles) {
  PotentialField pf;
  pf.addObstacle(SphereObstacle(1, SpatialVector(), 1, 2, 10));
  pf.addObstacle(SphereObstacle(2, SpatialVector(), 1, 2, 10));
  pf.clearObstacles();
  EXPECT_TRUE(pf.getObstacles().empty());
}

TEST(PotentialFieldTest, VelocityAtGoalIsZero) {
  SpatialVector goal(1, 1, 1);
  PotentialField pf(goal, 1.0f);
  SpatialVector v = pf.computeVelocityAtPosition(goal);
  EXPECT_NEAR(v.getX(), 0.0f, 1e-5);
  EXPECT_NEAR(v.getY(), 0.0f, 1e-5);
  EXPECT_NEAR(v.getZ(), 0.0f, 1e-5);
}

TEST(PotentialFieldTest, AttractiveFieldPullsTowardGoal) {
  SpatialVector goal(0, 0, 0);
  PotentialField pf(goal, 2.0f);
  SpatialVector point(1, 0, 0);
  SpatialVector vel = pf.computeVelocityAtPosition(point);
  EXPECT_LT(vel.getX(), 0);  // should pull toward origin
  EXPECT_NEAR(vel.getY(), 0.0f, 1e-5);
}

TEST(PotentialFieldTest, RepulsiveFieldPushesAwayFromObstacle) {
  PotentialField pf(SpatialVector(0, 0, 0), 0.0f);  // no attraction
  pf.addObstacle(SphereObstacle(0, SpatialVector(0, 0, 0), 0.5f, 2.0f, 10.0f));
  SpatialVector point(1.0, 0.0, 0.0);
  SpatialVector vel = pf.computeVelocityAtPosition(point);
  EXPECT_GT(vel.getX(), 0);  // push away from origin
  EXPECT_NEAR(vel.getY(), 0.0f, 1e-5);
}
