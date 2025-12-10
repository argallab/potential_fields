#ifndef PFIELD_TELEOP_DEMO_HPP
#define PFIELD_TELEOP_DEMO_HPP

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cstddef>
#include "potential_fields_interfaces/srv/compute_autonomy_vector.hpp"
#include "potential_fields_interfaces/msg/obstacle_array.hpp"
#include "potential_fields_interfaces/msg/obstacle.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using ComputePFTwist = potential_fields_interfaces::srv::ComputeAutonomyVector;

class PFTeleopDemo : public rclcpp::Node {
public:
  PFTeleopDemo();
  ~PFTeleopDemo() = default;
private:
  std::string fixedFrame; // RViz fixed frame for visualization
  std::string eeLinkName; // End-effector link name
  double fuseAlpha; // Alpha for fusing Teleop and PF Twists

  // Create a service client for planning paths
  rclcpp::Client<ComputePFTwist>::SharedPtr pfTwistClient;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr teleopTwistSub; // Subscriber for the Joystick
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goalPosePub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr queryPosePub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr eeVelocityPub;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr jointVelocityPub;
  rclcpp::Publisher<potential_fields_interfaces::msg::ObstacleArray>::SharedPtr obstaclePub;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer; // TF buffer for listening to Robot TFs
  std::shared_ptr<tf2_ros::TransformListener> tfListener; // TF listener for Robot TFs to populate buffer
  rclcpp::TimerBase::SharedPtr timer; // Timer to periodically update the potential field

  geometry_msgs::msg::Pose queryPose;
  geometry_msgs::msg::TwistStamped latestTeleopTwist;
  geometry_msgs::msg::TwistStamped latestPFTwist;

  void timerCallback();
  geometry_msgs::msg::Twist fuseTwists(
    const geometry_msgs::msg::Twist twist1,
    const geometry_msgs::msg::Twist twist2,
    const double alpha);

  void createAndPublishObstacles();
  geometry_msgs::msg::Pose getEndEffectorPose();

  void handleComputeAutonomyVectorResponse(rclcpp::Client<ComputePFTwist>::SharedFuture future);

  // Timer-based streaming of end-effector velocity commands
  void startEEVelocityStreaming(const std::vector<geometry_msgs::msg::TwistStamped>& eeVels, double dt);
  void stopEEVelocityStreaming();
  void eeVelocityTimerCallback();
  // Streaming state
  bool isStreamingEEVel{false};
  std::vector<geometry_msgs::msg::TwistStamped> eeVelocityBuffer; // queued trajectory
  std::size_t eeVelocityIndex{0};
  double eeVelocityDt{0.1};
  rclcpp::TimerBase::SharedPtr eeVelocityTimer; // publishes at eeVelocityDt


  // Timer-based streaming of joint velocity commands
  void startJointVelocityStreaming(const trajectory_msgs::msg::JointTrajectory& jointTrajectory, double dt);
  void stopJointVelocityStreaming();
  void jointVelocityTimerCallback();
  // Streaming state
  bool isStreamingJointVel{false};
  trajectory_msgs::msg::JointTrajectory jointVelocityBuffer; // queued trajectory
  std::size_t jointVelocityIndex{0};
  double jointVelocityDt{0.1};
  rclcpp::TimerBase::SharedPtr jointVelocityTimer; // publishes at jointVelocityDt
};

#endif // !TELEOP_
