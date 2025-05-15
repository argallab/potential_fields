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
