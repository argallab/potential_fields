#ifndef MOTION_INTERFACE_HPP
#define MOTION_INTERFACE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_plugins/motion_plugin.hpp"
#include "potential_fields_interfaces/srv/plan_path.hpp"
#include "potential_fields_interfaces/srv/p_field_step.hpp"

using JointState = sensor_msgs::msg::JointState;
using PlanPath = potential_fields_interfaces::srv::PlanPath;
using PFieldStep = potential_fields_interfaces::srv::PFieldStep;

struct PlanningResult {
  bool success;
  nav_msgs::msg::Path path;
  trajectory_msgs::msg::JointTrajectory jointTrajectory;
};

class MotionInterface : public rclcpp::Node {
public:
  MotionInterface();
  ~MotionInterface() = default;

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr humanTwistSub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr autonomyTwistSub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr fusedTwistPub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalPoseSub;
  rclcpp::Publisher<JointState>::SharedPtr planningJointStatePub; // Publisher for planning joint states
  rclcpp::Service<PlanPath>::SharedPtr pathPlanningService; // Now hosted here
  rclcpp::Client<PFieldStep>::SharedPtr pfieldStepClient; // Client to request single PF steps

  double fusionAlpha;
  std::unique_ptr<MotionPlugin> motionPlugin;

  geometry_msgs::msg::Twist::SharedPtr twistLimits;
  geometry_msgs::msg::PoseStamped::SharedPtr currentGoal;
  geometry_msgs::msg::TwistStamped::SharedPtr lastHumanTwist;
  geometry_msgs::msg::TwistStamped::SharedPtr lastAutonomyTwist;

  // Service callback for PlanPath requests
  void handlePlanPath(const PlanPath::Request::SharedPtr request, PlanPath::Response::SharedPtr response);

  geometry_msgs::msg::TwistStamped fuseTwists(
    const geometry_msgs::msg::TwistStamped::SharedPtr humanTwist,
    const geometry_msgs::msg::TwistStamped::SharedPtr autonomyTwist);

  geometry_msgs::msg::TwistStamped clampTwist(const geometry_msgs::msg::TwistStamped& twist);
};

#endif // !MOTION_INTERFACE_HPP
