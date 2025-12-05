#ifndef PFIELD_DEMO_HPP
#define PFIELD_DEMO_HPP

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cstddef>
#include "potential_fields_interfaces/srv/plan_path.hpp"
#include "potential_fields_interfaces/msg/obstacle_array.hpp"
#include "potential_fields_interfaces/msg/obstacle.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using PlanPath = potential_fields_interfaces::srv::PlanPath;

class PFDemo : public rclcpp::Node {
public:
  PFDemo();
  ~PFDemo() = default;
private:
  std::string fixedFrame; // RViz fixed frame for visualization
  std::string eeLinkName; // End-effector link name

  // Create a service client for planning paths
  rclcpp::Client<PlanPath>::SharedPtr planPathClient;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goalPosePub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr queryPosePub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr eeVelocityPub;
  rclcpp::Publisher<potential_fields_interfaces::msg::ObstacleArray>::SharedPtr obstaclePub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr runPlanPathDemoService;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer; // TF buffer for listening to Robot TFs
  std::shared_ptr<tf2_ros::TransformListener> tfListener; // TF listener for Robot TFs to populate buffer

  void createAndPublishObstacles();
  geometry_msgs::msg::Pose getEndEffectorPose();

  // Service callback handlers
  void handleRunPlanPathDemo(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response);

  void handlePlanPathResponse(
    rclcpp::Client<PlanPath>::SharedFuture future,
    double dt);

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
};

#endif // PFIELD_DEMO_HPP
