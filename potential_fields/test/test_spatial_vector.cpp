#include <gtest/gtest.h>
#include "spatial_vector.hpp"
#include <eigen3/Eigen/Dense>

TEST(SpatialVectorTest, EuclideanDistance) {
  SpatialVector a;
  SpatialVector b(Eigen::Vector3d(3.0f, 4.0f, 0.0f));
  EXPECT_FLOAT_EQ(a.euclideanDistance(b), 5.0f);
}

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

TEST(SpatialVectorTest, GeodesicDistance) {
  SpatialVector a;
  SpatialVector b;
  a.setOrientationEuler(0, 0, 0);
  b.setOrientationEuler(0, 0, M_PI_2); // Yaw 90 degrees
  EXPECT_NEAR(a.geodesicDistance(b), M_PI_2, 1e-5);
  // Check the reverse operation since angles should be bounded by [0, pi]
  EXPECT_NEAR(b.geodesicDistance(a), M_PI_2, 1e-5);
}
