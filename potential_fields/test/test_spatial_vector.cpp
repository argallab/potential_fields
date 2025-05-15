#include <gtest/gtest.h>
#include "spatial_vector.hpp"

TEST(SpatialVectorTest, EuclideanDistance) {
  SpatialVector a(0.0f, 0.0f, 0.0f);
  SpatialVector b(3.0f, 4.0f, 0.0f);
  EXPECT_FLOAT_EQ(a.euclideanDistance(b), 5.0f);
}

TEST(SpatialVectorTest, ConstructorAndAccessors) {
  SpatialVector v(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 0.9f);
  EXPECT_FLOAT_EQ(v.getX(), 1.0f);
  EXPECT_FLOAT_EQ(v.getY(), 2.0f);
  EXPECT_FLOAT_EQ(v.getZ(), 3.0f);
  EXPECT_FLOAT_EQ(v.getQX(), 0.1f);
  EXPECT_FLOAT_EQ(v.getQY(), 0.2f);
  EXPECT_FLOAT_EQ(v.getQZ(), 0.3f);
  EXPECT_FLOAT_EQ(v.getQW(), 0.9f);
}

TEST(SpatialVectorTest, NormalizePositionVector) {
  SpatialVector v(3.0f, 4.0f, 0.0f);
  v.normalizePosition();
  EXPECT_NEAR(v.getX(), 0.6f, 1e-5);
  EXPECT_NEAR(v.getY(), 0.8f, 1e-5);
}

TEST(SpatialVectorTest, OperatorAddition) {
  SpatialVector a(1, 2, 3);
  SpatialVector b(4, 5, 6);
  SpatialVector c = a + b;
  EXPECT_EQ(c.getX(), 5);
  EXPECT_EQ(c.getY(), 7);
  EXPECT_EQ(c.getZ(), 9);
}

TEST(SpatialVectorTest, OperatorSubtraction) {
  SpatialVector a(5, 7, 9);
  SpatialVector b(1, 2, 3);
  SpatialVector c = a - b;
  EXPECT_EQ(c.getX(), 4);
  EXPECT_EQ(c.getY(), 5);
  EXPECT_EQ(c.getZ(), 6);
}

TEST(SpatialVectorTest, OperatorMultiply) {
  SpatialVector a(1, -2, 3);
  SpatialVector c = a * 2.0f;
  EXPECT_EQ(c.getX(), 2);
  EXPECT_EQ(c.getY(), -4);
  EXPECT_EQ(c.getZ(), 6);
}

TEST(SpatialVectorTest, OperatorDivide) {
  SpatialVector a(2, 4, 6);
  SpatialVector c = a / 2.0f;
  EXPECT_EQ(c.getX(), 1);
  EXPECT_EQ(c.getY(), 2);
  EXPECT_EQ(c.getZ(), 3);
}

TEST(SpatialVectorTest, OperatorDivideByZero) {
  SpatialVector a(1, 1, 1);
  EXPECT_THROW(a / 0.0f, std::invalid_argument);
}

TEST(SpatialVectorTest, QuaternionDifferenceZero) {
  SpatialVector a(1, 2, 3, 0, 0, 0, 1);
  SpatialVector b(1, 2, 3, 0, 0, 0, 1);
  SpatialVector diff = a.quaternionDifference(b);
  EXPECT_NEAR(diff.getQX(), 0.0f, 1e-5);
  EXPECT_NEAR(diff.getQY(), 0.0f, 1e-5);
  EXPECT_NEAR(diff.getQZ(), 0.0f, 1e-5);
}

TEST(SpatialVectorTest, QuaternionGeodesic) {
  SpatialVector a(0, 0, 0);
  SpatialVector b(0, 0, 0);
  a.setOrientationEuler(0, 0, 0);
  b.setOrientationEuler(0, 0, M_PI_2); // Yaw 90 degrees
  EXPECT_NEAR(a.geodesicDistance(b), M_PI_2, 1e-5);
  // Check the reverse operation since angles should be bounded by [0, pi]
  EXPECT_NEAR(b.geodesicDistance(a), M_PI_2, 1e-5);
}

TEST(SpatialVectorTest, QuaternionDifference) {
  SpatialVector a(0, 0, 0);
  SpatialVector b(0, 0, 0);
  a.setOrientationEuler(0, 0, 0);
  b.setOrientationEuler(0, 0, M_PI_2); // Yaw 90 degrees
  SpatialVector diff = a.quaternionDifference(b); // Should be 90 degrees
  EXPECT_NEAR(diff.getQX(), 0.0f, 1e-5);
  EXPECT_NEAR(diff.getQY(), 0.0f, 1e-5);
  EXPECT_NEAR(diff.getQZ(), 0.7071f, 1e-4); // sqrt(2)/2
  EXPECT_NEAR(diff.getQW(), 0.7071f, 1e-4); // sqrt(2)/2
  EXPECT_NEAR(a.geodesicDistance(b), M_PI_2, 1e-5);
  EXPECT_NEAR(b.geodesicDistance(a), M_PI_2, 1e-5);
  SpatialVector c(0, 0, 0);
  SpatialVector d(0, 0, 0);
  c.setOrientationEuler(0, 0, M_PI_2);
  d.setOrientationEuler(0, 0, M_PI);
  SpatialVector diff2 = c.quaternionDifference(d);
  EXPECT_NEAR(diff2.getQX(), 0.0f, 1e-5);
  EXPECT_NEAR(diff2.getQY(), 0.0f, 1e-5);
  EXPECT_NEAR(diff2.getQZ(), 0.7071f, 1e-4); // sqrt(2)/2
  EXPECT_NEAR(diff2.getQW(), -0.7071f, 1e-4); // -sqrt(2)/2 
  EXPECT_NEAR(c.geodesicDistance(d), M_PI_2, 1e-5);
  EXPECT_NEAR(d.geodesicDistance(c), M_PI_2, 1e-5);
}

TEST(SpatialVectorTest, QuaternionFromEuler) {
  SpatialVector a(0, 0, 0);
  a.setOrientationEuler(0, 0, 0);
  EXPECT_NEAR(a.getQX(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQY(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQZ(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQW(), 1.0f, 1e-5);
  a.setOrientationEuler(M_PI_2, 0, 0);
  EXPECT_NEAR(a.getQX(), 0.7071f, 1e-4); // sqrt(2)/2
  EXPECT_NEAR(a.getQY(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQZ(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQW(), 0.7071f, 1e-4); // sqrt(2)/2
  a.setOrientationEuler(0, M_PI_2, 0);
  EXPECT_NEAR(a.getQX(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQY(), 0.7071f, 1e-4); // sqrt(2)/2
  EXPECT_NEAR(a.getQZ(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQW(), 0.7071f, 1e-4); // sqrt(2)/2
  a.setOrientationEuler(0, 0, M_PI_2);
  EXPECT_NEAR(a.getQX(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQY(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQZ(), 0.7071f, 1e-4); // sqrt(2)/2
  EXPECT_NEAR(a.getQW(), 0.7071f, 1e-4); // sqrt(2)/2
  a.setOrientationEuler(M_PI_2, M_PI_2, M_PI_2);
  EXPECT_NEAR(a.getQX(), 0.7071f, 1e-4); // sqrt(2)/2
  EXPECT_NEAR(a.getQY(), 0.7071f, 1e-4); // sqrt(2)/2
  EXPECT_NEAR(a.getQZ(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQW(), 0.0f, 1e-5);
  a.setOrientationEuler(M_PI, M_PI, M_PI);
  EXPECT_NEAR(a.getQX(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQY(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQZ(), 0.0f, 1e-5);
  EXPECT_NEAR(a.getQW(), -1.0f, 1e-5);
}

TEST(SpatialVectorTest, EulerRoundTrip) {
  SpatialVector a;
  float roll_in = M_PI_4, pitch_in = M_PI / 6.0f, yaw_in = M_PI_4;
  a.setOrientationEuler(roll_in, pitch_in, yaw_in);

  float roll_out, pitch_out, yaw_out;
  a.getEulerAngles(roll_out, pitch_out, yaw_out);

  EXPECT_NEAR(roll_in, roll_out, 1e-4);
  EXPECT_NEAR(pitch_in, pitch_out, 1e-4);
  EXPECT_NEAR(yaw_in, yaw_out, 1e-4);
}
