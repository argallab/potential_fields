#include <gtest/gtest.h>
#include "spatial_vector.hpp"
#include <eigen3/Eigen/Dense>


TEST(SpatialVectorTest, ConstructorAndAccessors) {
  // SpatialVector v(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 0.9f);
  SpatialVector v(Eigen::Vector3d(1.0f, 2.0f, 3.0f), Eigen::Quaterniond(0.1f, 0.2f, 0.3f, 0.9f));
  EXPECT_EQ(v.getPosition(), Eigen::Vector3d(1.0f, 2.0f, 3.0f));
  EXPECT_EQ(v.getOrientation(), Eigen::Quaterniond(0.1f, 0.2f, 0.3f, 0.9f));
  v.setOrientation(Eigen::Quaterniond(0.0f, 0.0f, 0.0f, 1.0f));
  EXPECT_EQ(v.getOrientation(), Eigen::Quaterniond(0.0f, 0.0f, 0.0f, 1.0f));
  v.setPosition(Eigen::Vector3d(4.0f, 5.0f, 6.0f));
  EXPECT_EQ(v.getPosition(), Eigen::Vector3d(4.0f, 5.0f, 6.0f));
}

TEST(SpatialVectorTest, EuclideanDistanceSamePosition) {
  SpatialVector a;
  SpatialVector b;
  EXPECT_FLOAT_EQ(a.euclideanDistance(b), 0.0f);
}

TEST(SpatialVectorTest, EuclideanDistance) {
  SpatialVector a;
  SpatialVector b(Eigen::Vector3d(3.0f, 4.0f, 0.0f));
  EXPECT_FLOAT_EQ(a.euclideanDistance(b), 5.0f);
  b.setPosition(Eigen::Vector3d(-4.5f, -2.1f, 3.0f));
  EXPECT_FLOAT_EQ(a.euclideanDistance(b), 5.801724f);
}

TEST(SpatialVectorTest, NormalizePosition) {
  SpatialVector v(Eigen::Vector3d(3.0f, 4.0f, 0.0f));
  v.normalizePosition();
  EXPECT_FLOAT_EQ(v.getPosition().norm(), 1.0f);
}

TEST(SpatialVectorTest, SetAndGetEulerAngles) {
  // Angles are [yaw, pitch, roll]
  std::vector<Eigen::Vector3d> angles = {
    {M_PI_2 / 2,  0.0,  0.0},
    {0.0,  -M_PI / 4,  0.0},
    {0.0,  0.0, M_PI_2},
    {M_PI / 4,  -M_PI / 8, 0.0},
    {0.0,      0.0,     0.0}
  };
  for (auto&& ang : angles) {
    SpatialVector v;
    v.setOrientationEuler(ang.x(), ang.y(), ang.z());
    Eigen::Vector3d out = v.getOrientationEuler();
    EXPECT_NEAR(out.x(), ang.x(), 1e-5);
    EXPECT_NEAR(out.y(), ang.y(), 1e-5);
    EXPECT_NEAR(out.z(), ang.z(), 1e-5);
  }
}

TEST(SpatialVectorTest, AngularDistance) {
  SpatialVector a;
  SpatialVector b;
  a.setOrientationEuler(0, 0, 0);
  b.setOrientationEuler(M_PI_2, 0, 0); // Yaw 90 degrees
  EXPECT_NEAR(a.angularDistance(b), M_PI_2, 1e-5);
  // Check the reverse operation since angles should be bounded by [0, pi]
  EXPECT_NEAR(b.angularDistance(a), M_PI_2, 1e-5);
  a.setOrientationEuler(0, 0, 0);
  b.setOrientationEuler(0, 0, 0);
  EXPECT_NEAR(a.angularDistance(b), 0.0f, 1e-5);
}

TEST(SpatialVectorTest, OrientationNormalization) {
  Eigen::Quaterniond q(2.0, 0.0, 0.0, 0.0);
  SpatialVector v;
  v.setOrientation(q);
  // Should normalize internally to unit quaternion
  EXPECT_NEAR(v.getOrientation().norm(), 1.0, 1e-6);
}

