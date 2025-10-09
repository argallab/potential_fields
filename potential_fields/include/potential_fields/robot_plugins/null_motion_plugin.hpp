#ifndef NULL_MOTION_PLUGIN_HPP
#define NULL_MOTION_PLUGIN_HPP

#include "motion_plugin.hpp"
#include <vector>

// A minimal IK solver that returns the seed as the solution (useful for offline testing)
class SimpleIKSolver : public IKSolver {
public:
  SimpleIKSolver() = default;
  ~SimpleIKSolver() override = default;

  bool solve([[maybe_unused]] const Eigen::Isometry3d& targetPose, const std::vector<double>& seed,
    std::vector<double>& solution, Eigen::Matrix<double, 6, Eigen::Dynamic>& J) override {
    if (seed.empty()) {
      // default to 7 zeros
      solution = std::vector<double>(7, 0.0);
    }
    else {
      solution = seed;
    }
    J.resize(6, solution.size());
    J.setZero();
    return true;
  }

  bool computeJacobian(const std::vector<double>& jointPositions, Eigen::Matrix<double, 6, Eigen::Dynamic>& J) override {
    J.resize(6, jointPositions.size());
    J.setZero();
    return true;
  }

  std::vector<std::string> getJointNames() const override {
    return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  }
};

// A null MotionPlugin which performs no real robot IO and is suitable for offline testing
class NullMotionPlugin : public MotionPlugin {
public:
  NullMotionPlugin() {
    this->assignIKSolver(std::make_shared<SimpleIKSolver>());
    this->assignSharedClock(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  }
  ~NullMotionPlugin() override = default;

  bool sendCartesianTwist(const geometry_msgs::msg::Twist& /*endEffectorTwist*/) override {
    // No-op for offline testing
    return true;
  }

  bool sendJointStates(const sensor_msgs::msg::JointState& /*js*/) override {
    // No-op
    return true;
  }

  bool readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) override {
    // Provide a neutral robot state for testing
    auto names = this->getIKSolver()->getJointNames();
    js.name = names;
    js.position = std::vector<double>(names.size(), 0.0);
    js.header.stamp = this->clock->now();

    endEffectorPose.header.stamp = this->clock->now();
    endEffectorPose.header.frame_id = "world";
    endEffectorPose.pose.orientation.w = 1.0;
    endEffectorPose.pose.position.x = 0.0;
    endEffectorPose.pose.position.y = 0.0;
    endEffectorPose.pose.position.z = 0.0;
    return true;
  }
};

#endif // NULL_MOTION_PLUGIN_HPP
