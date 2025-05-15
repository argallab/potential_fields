#include <gtest/gtest.h>
#include "spatial_vector.hpp"

TEST(SpatialVectorTest, EuclideanDistance) {
  SpatialVector a(0.0f, 0.0f, 0.0f);
  SpatialVector b(3.0f, 4.0f, 0.0f);
  EXPECT_FLOAT_EQ(a.euclideanDistance(b), 5.0f);
}

TEST(SpatialVectorTest, QuaternionGeodesic) {
  SpatialVector a(0, 0, 0, 0, 0, 0, 1);
  SpatialVector b(0, 0, 0, 0, 0, std::sin(M_PI / 4), std::cos(M_PI / 4));
  float angle = a.geodesicDistance(b);
  EXPECT_NEAR(angle, M_PI / 2, 1e-5);
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
  v.normalize();
  EXPECT_NEAR(v.getX(), 0.6f, 1e-5);
  EXPECT_NEAR(v.getY(), 0.8f, 1e-5);
}

TEST(SpatialVectorTest, QuaternionDifferenceZero) {
  SpatialVector a(1, 2, 3, 0, 0, 0, 1);
  SpatialVector b(1, 2, 3, 0, 0, 0, 1);
  SpatialVector diff = a.quaternionDifference(b);
  EXPECT_NEAR(diff.getQX(), 0.0f, 1e-5);
  EXPECT_NEAR(diff.getQY(), 0.0f, 1e-5);
  EXPECT_NEAR(diff.getQZ(), 0.0f, 1e-5);
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


