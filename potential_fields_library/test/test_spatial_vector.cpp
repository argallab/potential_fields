// Copyright 2025 Sharwin Patil
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include "pfield/spatial_vector.hpp"

TEST(SpatialVectorTest, ConstructorAndAccessors) {
  // pfield::SpatialVector v(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 0.9f);
  pfield::SpatialVector v(Eigen::Vector3d(1.0f, 2.0f, 3.0f), Eigen::Quaterniond(0.1f, 0.2f, 0.3f, 0.9f));
  EXPECT_EQ(v.getPosition(), Eigen::Vector3d(1.0f, 2.0f, 3.0f));
  EXPECT_EQ(v.getOrientation(), Eigen::Quaterniond(0.1f, 0.2f, 0.3f, 0.9f));
  v.setOrientation(Eigen::Quaterniond(0.0f, 0.0f, 0.0f, 1.0f));
  EXPECT_EQ(v.getOrientation(), Eigen::Quaterniond(0.0f, 0.0f, 0.0f, 1.0f));
  v.setPosition(Eigen::Vector3d(4.0f, 5.0f, 6.0f));
  EXPECT_EQ(v.getPosition(), Eigen::Vector3d(4.0f, 5.0f, 6.0f));
}

TEST(SpatialVectorTest, EuclideanDistanceSamePosition) {
  pfield::SpatialVector a;
  pfield::SpatialVector b;
  EXPECT_FLOAT_EQ(a.euclideanDistance(b), 0.0f);
}

TEST(SpatialVectorTest, EuclideanDistance) {
  pfield::SpatialVector a;
  pfield::SpatialVector b(Eigen::Vector3d(3.0f, 4.0f, 0.0f));
  EXPECT_FLOAT_EQ(a.euclideanDistance(b), 5.0f);
  b.setPosition(Eigen::Vector3d(-4.5f, -2.1f, 3.0f));
  EXPECT_FLOAT_EQ(a.euclideanDistance(b), 5.801724f);
}

TEST(SpatialVectorTest, NormalizePosition) {
  pfield::SpatialVector v(Eigen::Vector3d(3.0f, 4.0f, 0.0f));
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
    pfield::SpatialVector v;
    v.setOrientationEuler(ang.x(), ang.y(), ang.z());
    Eigen::Vector3d out = v.getOrientationEuler();
    EXPECT_NEAR(out.x(), ang.x(), 1e-5);
    EXPECT_NEAR(out.y(), ang.y(), 1e-5);
    EXPECT_NEAR(out.z(), ang.z(), 1e-5);
  }
}

TEST(SpatialVectorTest, AngularDistance) {
  pfield::SpatialVector a;
  pfield::SpatialVector b;
  a.setOrientationEuler(0, 0, 0);
  b.setOrientationEuler(M_PI_2, 0, 0); // Yaw 90 degrees
  EXPECT_NEAR(a.angularDistance(b), M_PI_2, 1e-5);
  // Check the reverse operation since angles should be bounded by [0, pi]
  EXPECT_NEAR(b.angularDistance(a), M_PI_2, 1e-5);
  a.setOrientationEuler(0, 0, 0);
  b.setOrientationEuler(0, 0, 0);
  EXPECT_NEAR(a.angularDistance(b), 0.0f, 1e-5);
  b.setOrientationEuler(0, M_PI, 0); // Pitch 180 degrees
  EXPECT_NEAR(a.angularDistance(b), M_PI, 1e-5);
  a.setOrientationEuler(0, 0, M_PI_2); // Roll 90 degrees
  b.setOrientationEuler(0, M_PI_2, M_PI_2); // Roll and Pitch 90 degrees
  // Check both directions (should be the same)
  EXPECT_NEAR(a.angularDistance(b), M_PI_2, 1e-5);
  EXPECT_NEAR(b.angularDistance(a), M_PI_2, 1e-5);
}

TEST(SpatialVectorTest, OrientationNormalization) {
  Eigen::Quaterniond q(2.0, 0.0, 0.0, 0.0);
  pfield::SpatialVector v;
  v.setOrientation(q);
  // Should normalize internally to unit quaternion
  EXPECT_NEAR(v.getOrientation().norm(), 1.0, 1e-6);
}
