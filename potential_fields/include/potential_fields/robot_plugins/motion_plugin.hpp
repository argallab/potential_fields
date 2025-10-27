#ifndef MOTION_PLUGIN_HPP
#define MOTION_PLUGIN_HPP

#include <memory>
#include <utility>
#include <string>
#include <vector>

#include "rclcpp/clock.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ik_solver.hpp"


class MotionPlugin {
public:
  explicit MotionPlugin(const std::string& name) : name(name) {}
  virtual ~MotionPlugin() = default;
  virtual bool sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) = 0;
  virtual bool sendJointStates(const sensor_msgs::msg::JointState& js) = 0;
  virtual bool readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) = 0;
  virtual void assignSharedClock(rclcpp::Clock::SharedPtr clock) { this->clock = clock; }
  virtual void assignIKSolver(std::shared_ptr<IKSolver> ikSolver) { this->ikSolver = ikSolver; }
  virtual std::shared_ptr<IKSolver> getIKSolver() const { return ikSolver; }
  virtual std::string getName() const { return name; }
  virtual std::vector<double> getCurrentJointAngles() const { return currentJointAngles; }
protected:
  rclcpp::Clock::SharedPtr clock;
  std::shared_ptr<IKSolver> ikSolver;
  std::string name;
  std::vector<double> currentJointAngles; // Current joint angles of the robot
};

#endif // !MOTION_PLUGIN_HPP
