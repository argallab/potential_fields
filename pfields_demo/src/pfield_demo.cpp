#include "pfield_demo.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "tf2_ros/static_transform_broadcaster.h"
#include "potential_fields_interfaces/srv/plan_path.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <filesystem>

template<typename T>
using ServiceResponseFuture = rclcpp::Client<T>::SharedFuture;

PFDemo::PFDemo() : Node("pfield_demo") {
  RCLCPP_INFO(this->get_logger(), "PFDemo Initialized");

  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  this->eeLinkName = this->declare_parameter("ee_link_name", "link_tcp"); // End-effector link name
  // Get parameters from yaml file
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();
  this->eeLinkName = this->get_parameter("ee_link_name").as_string();

  this->goalPosePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pfield/planning_goal_pose", 10);

  // Wait for the service to be available
  this->planPathClient = this->create_client<PlanPath>("pfield/plan_path");
  while (!this->planPathClient->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for the plan_path service to be available...");
  }
  RCLCPP_INFO(this->get_logger(), "Plan path service is available.");

  this->eeVelocityPub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/robot_action", 10);
  this->obstaclePub = this->create_publisher<potential_fields_interfaces::msg::ObstacleArray>("/pfield/obstacles", 10);

  this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer, this);

  this->createAndPublishObstacles();

  // Initialize demo service
  this->runPlanPathDemoService = this->create_service<std_srvs::srv::Empty>(
    "/pfield_demo/run_plan_path_demo",
    std::bind(&PFDemo::handleRunPlanPathDemo, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void PFDemo::handleRunPlanPathDemo(
  [[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr request,
  [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(), "Running plan path demo...");

  // Create a request for the plan_path service
  auto pathPlanRequest = std::make_shared<PlanPath::Request>();
  // Define start and goal poses
  geometry_msgs::msg::PoseStamped startPose;
  startPose.header.stamp = this->now();
  startPose.header.frame_id = this->fixedFrame;
  // startPose.pose = this->getEndEffectorPose();
  startPose.pose.position.x = 0.6;
  startPose.pose.position.y = 0.0;
  startPose.pose.position.z = 0.3;
  // Z axis pointing down, X axis forward
  startPose.pose.orientation.x = 1.0;
  startPose.pose.orientation.y = 0.0;
  startPose.pose.orientation.z = 0.0;
  startPose.pose.orientation.w = 0.0;

  geometry_msgs::msg::PoseStamped goalPose;
  goalPose.header.stamp = this->now();
  goalPose.header.frame_id = this->fixedFrame;
  goalPose.pose = startPose.pose;
  goalPose.pose.position.x = 0.5;
  goalPose.pose.position.y = 0.2;
  goalPose.pose.position.z = 0.3;

  pathPlanRequest->start = startPose;
  // Use a non-singular start configuration (slightly bent) instead of all zeros
  // xArm7: [J1, J2, J3, J4, J5, J6, J7]
  pathPlanRequest->starting_joint_angles = {0.0, -0.5, 0.0, 0.5, 0.0, 0.5, 0.0}; 
  pathPlanRequest->goal = goalPose;
  pathPlanRequest->delta_time = 0.02; // 2 ms between waypoints
  pathPlanRequest->goal_tolerance = 0.001; // 1 mm tolerance
  pathPlanRequest->max_iterations = 30000; // Max iterations for planning
  pathPlanRequest->planning_method = "whole_body"; // "task_space" or "whole_body"
  const double dt = pathPlanRequest->delta_time;

  // Publish the goal pose
  this->goalPosePub->publish(goalPose);

  RCLCPP_INFO(this->get_logger(), "Sending plan_path request (async)...");

  // Send request asynchronously and attach a callback to process the result
  this->planPathClient->async_send_request(
    pathPlanRequest,
    [this, dt](ServiceResponseFuture<PlanPath> future) { this->handlePlanPathResponse(future, dt); }
  );
}

void PFDemo::handlePlanPathResponse(rclcpp::Client<PlanPath>::SharedFuture future, double dt) {
  auto pathPlanResponse = future.get();
  if (!pathPlanResponse) {
    RCLCPP_ERROR(this->get_logger(), "plan_path service returned an empty response (async)");
    return;
  }

  // Log a small summary of the returned trajectories to help debug crashes
  size_t ee_path_len = pathPlanResponse->end_effector_path.poses.size();
  size_t jt_points = pathPlanResponse->joint_trajectory.points.size();
  size_t ee_vels = pathPlanResponse->end_effector_velocity_trajectory.size();
  RCLCPP_INFO(this->get_logger(), "(async) Received plan_path response: success=%s, end_effector_path.len=%zu, joint_trajectory.points=%zu, ee_velocity_traj=%zu",
    pathPlanResponse->success ? "true" : "false", ee_path_len, jt_points, ee_vels);

  if (ee_path_len > 0) {
    const auto& p = pathPlanResponse->end_effector_path.poses.front().pose.position;
    RCLCPP_INFO(this->get_logger(), "(async) First EE pose: (%.4f, %.4f, %.4f)", p.x, p.y, p.z);
  }
  if (ee_vels > 0) {
    const auto& v = pathPlanResponse->end_effector_velocity_trajectory.front().twist.linear;
    RCLCPP_INFO(this->get_logger(), "(async) First EE linear velocity: (%.6f, %.6f, %.6f)", v.x, v.y, v.z);
  }

  // Begin streaming EE velocity commands to follow the path
  // this->startEEVelocityStreaming(pathPlanResponse->end_effector_velocity_trajectory, dt);
}

void PFDemo::createAndPublishObstacles() {
  potential_fields_interfaces::msg::ObstacleArray allObstacles;

  // Virtual sphere in the way
  potential_fields_interfaces::msg::Obstacle sphere;
  sphere.frame_id = "virtual_sphere";
  sphere.type = "Sphere";
  sphere.group = "Static";
  sphere.pose.position.x = 0.39 + 0.15;
  sphere.pose.position.y = -0.01;
  sphere.pose.position.z = 0.2935;
  sphere.pose.orientation.x = 0.0;
  sphere.pose.orientation.y = 0.0;
  sphere.pose.orientation.z = 0.0;
  sphere.pose.orientation.w = 1.0;
  sphere.radius = 0.01;
  allObstacles.obstacles.push_back(sphere);

  this->obstaclePub->publish(allObstacles);
}

geometry_msgs::msg::Pose PFDemo::getEndEffectorPose() {
  while (!this->tfBuffer->canTransform(this->fixedFrame, this->fixedFrame, tf2::TimePointZero, tf2::durationFromSec(0.1))) {}
  // Use the TF Listener to get the Pose of the `link_tcp` frame from the `world` frame
  try {
    auto tf = this->tfBuffer->lookupTransform(this->fixedFrame, this->eeLinkName, tf2::TimePointZero);
    geometry_msgs::msg::Pose eePose;
    eePose.position.x = tf.transform.translation.x;
    eePose.position.y = tf.transform.translation.y;
    eePose.position.z = tf.transform.translation.z;
    eePose.orientation.x = tf.transform.rotation.x;
    eePose.orientation.y = tf.transform.rotation.y;
    eePose.orientation.z = tf.transform.rotation.z;
    eePose.orientation.w = tf.transform.rotation.w;
    RCLCPP_INFO(this->get_logger(),
      "Found TF (%s -> %s): (%.2f, %.2f, %.2f) [mm]", this->fixedFrame.c_str(), this->eeLinkName.c_str(),
      eePose.position.x, eePose.position.y, eePose.position.z
    );
    return eePose;
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to find TF (%s -> %s): %s", this->fixedFrame.c_str(), this->eeLinkName.c_str(), ex.what()
    );
    return geometry_msgs::msg::Pose();
  }
}

void PFDemo::startEEVelocityStreaming(const std::vector<geometry_msgs::msg::TwistStamped>& eeVels, double dt) {
  // If an existing stream is in progress, stop it first
  this->stopEEVelocityStreaming();
  if (eeVels.empty() || dt <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Requested EE velocity stream has no points or non-positive dt (dt=%.6f)", dt);
    return;
  }

  this->eeVelocityBuffer = eeVels;
  // Append the zero command to stop the robot at the end
  geometry_msgs::msg::TwistStamped zeroCmd;
  zeroCmd.twist.linear.x = 0.0;
  zeroCmd.twist.linear.y = 0.0;
  zeroCmd.twist.linear.z = 0.0;
  zeroCmd.twist.angular.x = 0.0;
  zeroCmd.twist.angular.y = 0.0;
  zeroCmd.twist.angular.z = 0.0;
  this->eeVelocityBuffer.push_back(zeroCmd);
  this->eeVelocityIndex = 0;
  this->eeVelocityDt = dt;
  this->isStreamingEEVel = true;

  // Create and start a timer to publish at the requested period
  this->eeVelocityTimer = this->create_wall_timer(
    std::chrono::duration<double>(this->eeVelocityDt),
    std::bind(&PFDemo::eeVelocityTimerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Started EE velocity streaming: %zu points @ %.3f Hz", eeVelocityBuffer.size(), 1.0 / eeVelocityDt);
}

void PFDemo::stopEEVelocityStreaming() {
  if (this->eeVelocityTimer) {
    this->eeVelocityTimer->cancel();
    this->eeVelocityTimer.reset();
  }
  if (this->isStreamingEEVel) {
    RCLCPP_INFO(this->get_logger(), "Stopped EE velocity streaming after %zu/%zu points", eeVelocityIndex, eeVelocityBuffer.size());
  }
  this->isStreamingEEVel = false;
  this->eeVelocityBuffer.clear();
  this->eeVelocityIndex = 0;
}

void PFDemo::eeVelocityTimerCallback() {
  if (!this->isStreamingEEVel) {
    // Nothing to stream, defensive cancel
    if (this->eeVelocityTimer) {
      this->eeVelocityTimer->cancel();
      this->eeVelocityTimer.reset();
    }
    return;
  }

  if (this->eeVelocityIndex >= this->eeVelocityBuffer.size()) {
    // Finished streaming
    this->stopEEVelocityStreaming();
    return;
  }

  auto cmd = this->eeVelocityBuffer[this->eeVelocityIndex++];
  // Re-stamp for more accurate stamps
  cmd.header.stamp = this->now();
  this->eeVelocityPub->publish(cmd);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PFDemo>());
  rclcpp::shutdown();
  return 0;
}
