#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include "pfield/pfield.hpp"
#include "pfield/pf_obstacle.hpp"
#include "pfield/spatial_vector.hpp"
// Use the simple IK from the NullMotionPlugin for planPath tests
#include "robot_plugins/null_motion_plugin.hpp"

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
  EXPECT_EQ(pf.getObstacles()[0].getInfluenceDistance(), 2.5f);
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
  PotentialField pf(goal, 3.0, 0.0, 10.0, 1.0, 5.0, 5.0);
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
  // New semantics: absolute influence distance from surface. Pick a point farther than 2.0m from the box surface
  EXPECT_FALSE(box.withinInfluenceZone(Eigen::Vector3d(2.6, 0.0, 0.0)));
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
  // Far enough that distance to side surface > 3.0 (surface distance = 4.1 - 1.0 = 3.1)
  EXPECT_FALSE(cyl.withinInfluenceZone(Eigen::Vector3d(4.1, 0, 0)));
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
  // Construct with full limits: attractiveGain=0, rotationalGain=10,
  // maxLinearVel=0, maxAngularVel=15, maxLinearAccel=0, maxAngularAccel=5.
  // Raw angular velocity magnitude (gain * angle) = 10 * (pi/2) ≈ 15.7079 which will be velocity-capped to 15.
  PotentialField pf(goal, 0.0, 10.0, 0.0, 15.0, 0.0, 5.0);
  TaskSpaceWrench rawWrench = pf.evaluateWrenchAtPose(query);
  double rawOmegaMag = rawWrench.torque.norm();
  EXPECT_NEAR(rawOmegaMag, 10.0 * goal.angularDistance(query), 1e-6); // pre-constraint check
  // Use limited velocity to reflect soft saturation constraint (dt=0 => velocity soft-cap only)
  TaskSpaceTwist vel = pf.evaluateLimitedVelocityAtPose(query);
  // Linear velocity should be ~0 (positions are equal)
  EXPECT_NEAR(vel.getLinearVelocity().x(), 0.0, 1e-3);
  EXPECT_NEAR(vel.getLinearVelocity().y(), 0.0, 1e-3);
  EXPECT_NEAR(vel.getLinearVelocity().z(), 0.0, 1e-3);
  // Angular velocity axis should align with the quaternion error axis (up to sign)
  Eigen::Quaterniond q_err = query.getOrientation().conjugate() * goal.getOrientation();
  q_err.normalize();
  Eigen::Vector3d axis = q_err.vec();
  if (axis.norm() > 1e-12) axis.normalize();
  const double omega_norm = vel.getAngularVelocity().norm();
  // Check colinearity: |axis x omega| ~ 0 (when axis ~ 0 for zero angle, omega_norm should also be ~0)
  if (axis.norm() < 1e-12) {
    EXPECT_NEAR(omega_norm, 0.0, 1e-6);
  }
  else {
    Eigen::Vector3d omega_dir = vel.getAngularVelocity() / (omega_norm + 1e-18);
    EXPECT_NEAR((axis.cross(omega_dir)).norm(), 0.0, 1e-3);
  }
  // Magnitude should be soft-saturated: max * tanh(raw/max)
  const double expectedOmega = pf.getMaxAngularVelocity() * std::tanh(rawOmegaMag / pf.getMaxAngularVelocity());
  EXPECT_NEAR(omega_norm, expectedOmega, 1e-6);
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

TEST(PotentialFieldTest, WrenchToTwistConversion) {
  // wrenchToTwist uses unit gains; twist should equal wrench components
  PotentialField pf; // default gains
  TaskSpaceWrench w(Eigen::Vector3d(1.0, -2.0, 3.5), Eigen::Vector3d(0.25, -0.5, 1.0));
  TaskSpaceTwist t = pf.wrenchToTwist(w);
  EXPECT_NEAR((t.getLinearVelocity() - w.force).norm(), 0.0, 1e-12);
  EXPECT_NEAR((t.getAngularVelocity() - w.torque).norm(), 0.0, 1e-12);
}

TEST(PotentialFieldTest, ApplyVelocityLimitsAccelerationDominates) {
  // When going from 0 to a large twist in a short dt, acceleration limits should dominate
  PotentialField pf; // defaults: vmax=5 m/s, amax=1 m/s^2; wmax=1 rad/s, alpha_max=1 rad/s^2
  TaskSpaceTwist prev(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  TaskSpaceTwist cmd(Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d(0.0, 2.0, 0.0));
  const double dt = 0.1; // s
  TaskSpaceTwist lim = pf.applyMotionConstraints(cmd, prev, dt);
  // Implementation clamps velocity first, then acceleration if still exceeding accel caps
  // From zero, commanded 10 m/s gets clamped to vmax=5 m/s; accel check then reduces to prev + amax*dt = 0.1 m/s
  // With soft saturation re-applied, expect a value slightly below 0.1; allow small tolerance
  EXPECT_NEAR(lim.getLinearVelocity().x(), 1.0 * dt, 1e-4); // ~0.1 m/s
  EXPECT_NEAR(lim.getLinearVelocity().y(), 0.0, 1e-12);
  EXPECT_NEAR(lim.getLinearVelocity().z(), 0.0, 1e-12);
  // After rate limit to 0.1 rad/s, soft saturation is applied again -> |w| = wmax * tanh(0.1/wmax) = tanh(0.1)
  EXPECT_NEAR(lim.getAngularVelocity().y(), std::tanh(1.0 * dt), 1e-6);
  EXPECT_NEAR(lim.getAngularVelocity().x(), 0.0, 1e-12);
  EXPECT_NEAR(lim.getAngularVelocity().z(), 0.0, 1e-12);
}

TEST(PotentialFieldTest, ApplyVelocityLimitsVelocityCapOnly) {
  // If previous twist is already at velocity cap in same direction, velocity cap should apply without accel limiting
  // Construct a PF and set previous to max values; request a larger command, expect capped at max
  PotentialField pf; // defaults
  // Previous at vmax along +X and wmax along +Z
  TaskSpaceTwist prev(Eigen::Vector3d(5.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0));
  TaskSpaceTwist cmd(Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 2.0));
  const double dt = 0.2; // acceleration from 5->5 after cap is zero
  TaskSpaceTwist lim = pf.applyMotionConstraints(cmd, prev, dt);
  // First soft-saturation on cmd gives v1 = vmax*tanh(10/5) then re-soft-saturation yields v = vmax*tanh(v1/vmax)
  {
    const double vmax = pf.getMaxLinearVelocity();
    const double v1 = vmax * std::tanh(10.0 / vmax);
    const double expected = vmax * std::tanh(v1 / vmax);
    EXPECT_NEAR(lim.getLinearVelocity().x(), expected, 1e-6);
  }
  EXPECT_NEAR(lim.getLinearVelocity().y(), 0.0, 1e-12);
  EXPECT_NEAR(lim.getLinearVelocity().z(), 0.0, 1e-12);
  // Angular: w1 = wmax*tanh(2/wmax) and final = wmax*tanh(w1/wmax) = tanh(tanh(2.0)) since wmax=1
  {
    const double wmax = pf.getMaxAngularVelocity();
    const double w1 = wmax * std::tanh(2.0 / wmax);
    const double expected = wmax * std::tanh(w1 / wmax);
    EXPECT_NEAR(lim.getAngularVelocity().z(), expected, 1e-6);
  }
  EXPECT_NEAR(lim.getAngularVelocity().x(), 0.0, 1e-12);
  EXPECT_NEAR(lim.getAngularVelocity().y(), 0.0, 1e-12);
}

TEST(PotentialFieldTest, IntegrateLinearVelocity) {
  PotentialField pf;
  Eigen::Vector3d p0(1.0, 2.0, 3.0);
  Eigen::Vector3d v(1.0, 0.0, -2.0);
  double dt = 0.5;
  Eigen::Vector3d p1 = pf.integrateLinearVelocity(p0, v, dt);
  EXPECT_NEAR(p1.x(), 1.5, 1e-12);
  EXPECT_NEAR(p1.y(), 2.0, 1e-12);
  EXPECT_NEAR(p1.z(), 2.0, 1e-12);
}

TEST(PotentialFieldTest, IntegrateAngularVelocityAboutZ) {
  PotentialField pf;
  Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  Eigen::Vector3d w(0.0, 0.0, 1.0); // rad/s about Z
  double dt = M_PI / 4.0; // 45 degrees
  Eigen::Quaterniond q1 = pf.integrateAngularVelocity(q0, w, dt);
  // Expected rotation of 45 deg about Z
  Eigen::Quaterniond q_expected(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()));
  // Compare up to sign: |dot| ~ 1
  double dot = std::abs(q1.dot(q_expected));
  EXPECT_NEAR(dot, 1.0, 1e-6);
}

TEST(PotentialFieldTest, InterpolateNextPoseTranslationalStep) {
  // With rotational gain zero, pose should move toward goal along -x with dt*speed
  SpatialVector goal(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  // Construct with all limits (attractiveGain=1, rotationalGain=0, vmax_lin=5, vmax_ang=1, amax_lin=1, amax_ang=1)
  PotentialField pf(1.0, 0.0, 5.0, 1.0, 1.0, 1.0);
  pf.setGoalPose(goal);
  SpatialVector current(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
  double dt = 0.1; // raw speed at x=1 would be 1 m/s; accel limit (1 m/s^2 * 0.1 s) => 0.1 m/s applied
  SpatialVector next = pf.interpolateNextPose(current, TaskSpaceTwist(), dt);
  // Expected new x: 1.0 + (-0.1 m/s)*0.1 s = 0.99
  EXPECT_NEAR(next.getPosition().x(), 0.99, 1e-5);
  EXPECT_NEAR(next.getPosition().y(), 0.0, 1e-6);
  EXPECT_NEAR(next.getPosition().z(), 0.0, 1e-6);
  // Orientation should remain identity (no angular velocity)
  double qdot = std::abs(next.getOrientation().dot(Eigen::Quaterniond::Identity()));
  EXPECT_NEAR(qdot, 1.0, 1e-6);
}

TEST(PotentialFieldTest, EvaluateWrenchAtPosePureTorque) {
  // Same position but 90deg yaw error -> zero force, non-zero torque
  SpatialVector goal;
  goal.setOrientationEuler(0, 0, 0);
  SpatialVector query;
  query.setOrientationEuler(0, 0, M_PI_2);
  // No translational attraction, only rotational
  PotentialField pf(goal, 0.0, 2.0);
  TaskSpaceWrench w = pf.evaluateWrenchAtPose(query);
  EXPECT_NEAR(w.force.norm(), 0.0, 1e-9);
  // Torque axis should match quaternion error axis up to sign
  Eigen::Quaterniond q_err = query.getOrientation().conjugate() * goal.getOrientation();
  q_err.normalize();
  Eigen::Vector3d axis = q_err.vec();
  if (axis.norm() > 1e-12) axis.normalize();
  double tau_norm = w.torque.norm();
  if (axis.norm() < 1e-12) {
    EXPECT_NEAR(tau_norm, 0.0, 1e-6);
  }
  else {
    Eigen::Vector3d tau_dir = w.torque / (tau_norm + 1e-18);
    EXPECT_NEAR((axis.cross(tau_dir)).norm(), 0.0, 1e-3);
  }
  EXPECT_NEAR(tau_norm, 2.0 * goal.angularDistance(query), 1e-2);
}

TEST(PotentialFieldTest, NoOvershootApproachGoalMonotonic) {
  // Verify that with a reasonable timestep the interpolated poses approach the goal monotonically
  // without crossing to the opposite side (no sign change) and without increasing distance.
  SpatialVector goal(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  // Use attractive gain = 1, no rotation component
  PotentialField pf(goal, 1.0, 0.0);
  // Start 1 m along +X
  SpatialVector current(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
  const double dt = 0.1; // dt < 1/gain ensures (1 - k dt) > 0 so analytical solution has no overshoot
  const unsigned int steps = 50; // sufficient steps to converge near tolerance
  TaskSpaceTwist prevTwist; // starts at zero
  double previousDistance = current.getPosition().norm();
  for (unsigned int i = 0; i < steps; ++i) {
    SpatialVector next = pf.interpolateNextPose(current, prevTwist, dt);
    // Reconstruct the constrained twist that would be used for the next step (matching interpolateNextPose logic)
    TaskSpaceTwist rawTwist = pf.wrenchToTwist(pf.evaluateWrenchAtPose(next));
    TaskSpaceTwist twistNow = pf.applyMotionConstraints(rawTwist, prevTwist, dt);
    // Distance to goal should not increase
    double dist = next.getPosition().norm();
    EXPECT_LE(dist, previousDistance + 1e-9) << "Distance increased at step " << i;
    // X position should remain non-negative (no overshoot past goal) and non-increasing
    EXPECT_GE(next.getPosition().x(), -1e-9) << "Overshoot past goal at step " << i;
    EXPECT_LE(next.getPosition().x(), current.getPosition().x() + 1e-9) << "X increased at step " << i;
    // Update for next loop
    previousDistance = dist;
    current = next;
    prevTwist = twistNow; // supply previous twist for potential acceleration limiting (if dt>0 used)
    if (dist <= 1e-3) { // within translational tolerance
      break;
    }
  }
  // Final pose should be within a small neighborhood of the goal and not negative on X
  EXPECT_GE(current.getPosition().x(), -1e-6);
  EXPECT_LE(current.getPosition().x(), 1.0);
  EXPECT_NEAR(current.getPosition().y(), 0.0, 1e-6);
  EXPECT_NEAR(current.getPosition().z(), 0.0, 1e-6);
}

TEST(PotentialFieldTest, PlanPathSinglePointWhenAlreadyAtGoal) {
  SpatialVector goal(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  // Construct PF with goal in constructor
  PotentialField pf(goal, 1.0, 0.0);
  // Provide a simple IK solver from the NullMotionPlugin
  NullMotionPlugin nullPlugin;
  auto ik = nullPlugin.getIKSolver();
  const double dt = 0.05;
  const double tol = 1e-3; // matches translationalTolerance
  PlannedPath path = pf.planPath(goal, dt, tol, ik); // startPose == goal
  ASSERT_EQ(path.numPoints, 1u);
  ASSERT_EQ(path.poses.size(), 1u);
  ASSERT_EQ(path.twists.size(), 1u);
  ASSERT_EQ(path.timeStamps.size(), 1u);
  EXPECT_NEAR(path.timeStamps[0], 0.0, 1e-9);
  EXPECT_NEAR(path.duration, 0.0, 1e-9);
  // Velocity at goal should be zero (already tested separately, re-check here for path integrity)
  EXPECT_NEAR(path.twists[0].getLinearVelocity().norm(), 0.0, 1e-9);
  EXPECT_NEAR(path.twists[0].getAngularVelocity().norm(), 0.0, 1e-9);
}

TEST(PotentialFieldTest, PlanPathMonotonicConvergenceToGoal) {
  SpatialVector goal(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  PotentialField pf(goal, 1.0, 0.0); // pure translation attraction
  SpatialVector start(Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
  NullMotionPlugin nullPlugin;
  auto ik = nullPlugin.getIKSolver();
  const double dt = 0.1;
  const double tol = 1e-3;
  const size_t maxIters = 500;
  PlannedPath path = pf.planPath(start, dt, tol, ik, maxIters);
  ASSERT_GT(path.numPoints, 1u); // should have progressed
  // Distances should be non-increasing
  double prevDist = (path.poses.front().getPosition() - goal.getPosition()).norm();
  for (size_t i = 1; i < path.poses.size(); ++i) {
    double dist = (path.poses[i].getPosition() - goal.getPosition()).norm();
    EXPECT_LE(dist, prevDist + 1e-9) << "Distance increased at index " << i;
    prevDist = dist;
  }
  // Final distance within tolerance
  double finalDist = (path.poses.back().getPosition() - goal.getPosition()).norm();
  EXPECT_LE(finalDist, tol + 1e-9);
  // Duration should correspond to last timestamp (starts at 0.0)
  if (!path.timeStamps.empty()) {
    EXPECT_NEAR(path.duration, path.timeStamps.back(), 1e-9);
  }
}

TEST(PotentialFieldTest, PlanPathTimeStampConsistency) {
  SpatialVector goal(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  PotentialField pf(goal, 1.0, 0.0);
  SpatialVector start(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
  NullMotionPlugin nullPlugin;
  auto ik = nullPlugin.getIKSolver();
  const double dt = 0.05;
  const double tol = 1e-3;
  const size_t maxIters = 400;
  PlannedPath path = pf.planPath(start, dt, tol, ik, maxIters);
  ASSERT_EQ(path.dt, dt); // stored dt
  // timeStamps[i] should be approximately i*dt
  for (size_t i = 0; i < path.timeStamps.size(); ++i) {
    EXPECT_NEAR(path.timeStamps[i], static_cast<double>(i) * dt, 1e-9);
  }
  // duration should equal last timestamp
  if (!path.timeStamps.empty()) {
    EXPECT_NEAR(path.duration, path.timeStamps.back(), 1e-9);
  }
}

TEST(PotentialFieldTest, PlanPathTwistMatchesEvaluateVelocityAtPose) {
  // Since planPath logs evaluateVelocityAtPose for each step, verify consistency
  SpatialVector goal(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  PotentialField pf(goal, 1.0, 0.0);
  SpatialVector start(Eigen::Vector3d(1.5, 0.0, 0.0), Eigen::Quaterniond::Identity());
  NullMotionPlugin nullPlugin;
  auto ik = nullPlugin.getIKSolver();
  const double dt = 0.1;
  const double tol = 1e-3;
  const size_t maxIters = 200;
  PlannedPath path = pf.planPath(start, dt, tol, ik, maxIters);
  ASSERT_EQ(path.poses.size(), path.twists.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    TaskSpaceTwist expectedTwist = pf.evaluateVelocityAtPose(path.poses[i]);
    // compare components
    EXPECT_NEAR((expectedTwist.getLinearVelocity() - path.twists[i].getLinearVelocity()).norm(), 0.0, 1e-9);
    EXPECT_NEAR((expectedTwist.getAngularVelocity() - path.twists[i].getAngularVelocity()).norm(), 0.0, 1e-9);
  }
}
