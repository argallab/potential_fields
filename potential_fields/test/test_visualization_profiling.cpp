#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "pfield_library/pfield/pfield.hpp"
#include "pfield_library/pfield/pf_obstacle.hpp"
#include "pfield_library/pfield/spatial_vector.hpp"

class VisualizationProfilingTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Setup a typical scenario
    goalPose = SpatialVector(Eigen::Vector3d(0.5, 0.0, 0.5), Eigen::Quaterniond::Identity());
    pf = std::make_shared<PotentialField>(goalPose);

    // Add some obstacles
    pf->addObstacle(PotentialFieldObstacle(
      "obs1",
      Eigen::Vector3d(0.3, 0.1, 0.3),
      Eigen::Quaterniond::Identity(),
      ObstacleType::SPHERE,
      ObstacleGroup::STATIC,
      ObstacleGeometry{0.1, 0.0, 0.0, 0.0}
    ));

    pf->addObstacle(PotentialFieldObstacle(
      "obs2",
      Eigen::Vector3d(0.6, -0.2, 0.6),
      Eigen::Quaterniond::Identity(),
      ObstacleType::BOX,
      ObstacleGroup::STATIC,
      ObstacleGeometry{0.0, 0.2, 0.2, 0.2}
    ));

    // Parameters matching PotentialFieldManager defaults
    visualizerBufferArea = 1.0;
    fieldResolution = 0.1; // Using a finer resolution to stress test, or match default 0.5
  }

  std::shared_ptr<PotentialField> pf;
  SpatialVector goalPose;
  double visualizerBufferArea;
  double fieldResolution;
};

TEST_F(VisualizationProfilingTest, ProfileVectorFieldGeneration) {
  SpatialVector queryPose(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
  PFLimits limits = pf->computeFieldBounds(queryPose, visualizerBufferArea);

  std::cout << "Profiling Vector Field Generation..." << std::endl;
  std::cout << "Limits: X[" << limits.minX << ", " << limits.maxX << "] "
    << "Y[" << limits.minY << ", " << limits.maxY << "] "
    << "Z[" << limits.minZ << ", " << limits.maxZ << "]" << std::endl;
  std::cout << "Resolution: " << fieldResolution << std::endl;

  auto start = std::chrono::high_resolution_clock::now();
  int pointCount = 0;
  int evaluatedCount = 0;

  for (double x = limits.minX; x <= limits.maxX; x += fieldResolution) {
    for (double y = limits.minY; y <= limits.maxY; y += fieldResolution) {
      for (double z = limits.minZ; z <= limits.maxZ; z += fieldResolution) {
        pointCount++;
        Eigen::Vector3d point(x, y, z);
        if (pf->isPointInsideObstacle(point)) { continue; }

        SpatialVector position{point};
        TaskSpaceTwist velocity = pf->evaluateLimitedVelocityAtPose(position);
        (void)velocity; // Suppress unused variable warning
        evaluatedCount++;

        // Simulate marker creation overhead (minimal)
        // Marker vectorMarker;
        // vectorMarker.pose.position.x = position.getPosition().x();
      }
    }
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  std::cout << "Total Points: " << pointCount << std::endl;
  std::cout << "Evaluated Points: " << evaluatedCount << std::endl;
  std::cout << "Time Elapsed: " << elapsed.count() * 1000 << " milliseconds" << std::endl;
  std::cout << "Average Time per Point: " << (elapsed.count() / pointCount) * 1e6 << " microseconds" << std::endl;

  // Assert that it runs within a reasonable time frame (e.g., < 0.1s for 10Hz)
  // Note: This assertion might be flaky depending on the machine and resolution
  // For a unit test, we might just want to ensure it completes within a reasonable time.
  EXPECT_LT(elapsed.count(), 0.4); // Visualization loop should take less than 400ms
}
