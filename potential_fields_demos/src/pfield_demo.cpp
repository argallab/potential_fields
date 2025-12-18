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

  this->goalPosePub = this->create_publisher<geometry_msgs::msg::Pose>("pfield/planning_goal_pose", 10);
  this->queryPosePub = this->create_publisher<geometry_msgs::msg::Pose>("pfield/query_pose", 10);

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
  this->jointVelocityPub = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/joint_vel_control", 10);
  this->obstaclePub = this->create_publisher<potential_fields_interfaces::msg::ObstacleArray>("/pfield/obstacles", 10);

  this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer, this);

  // Before publishing obstacles wait for the pfield/obstacles topic to exist
  RCLCPP_INFO(this->get_logger(), "Waiting for subscribers to the /pfield/obstacles topic...");
  while (this->obstaclePub->get_subscription_count() == 0) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(this->get_logger(), "Subscribers detected on the /pfield/obstacles topic. Publishing obstacles...");
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
  geometry_msgs::msg::Pose startPose;
  startPose = this->getEndEffectorPose();
  if (startPose.position.x == 0.0 &&
    startPose.position.y == 0.0 &&
    startPose.position.z == 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get current end-effector pose. Using default");
    startPose.position.x = 0.227;
    startPose.position.y = 0.00;
    startPose.position.z = 0.2935;
    startPose.orientation.x = 0.70709;
    startPose.orientation.y = 9.8864e-05;
    startPose.orientation.z = 0.70712;
    startPose.orientation.w = -2.1146e-05;
  }
  // Use Eigen to build quaternion from Euler angles (roll, pitch, yaw)
  // Eigen::AngleAxisd roll_ang(0.0, Eigen::Vector3d::UnitX());
  // Eigen::AngleAxisd pitch_ang(M_PI_2, Eigen::Vector3d::UnitY());
  // Eigen::AngleAxisd yaw_ang(M_PI, Eigen::Vector3d::UnitZ());
  // Compose as R = Rz(yaw) * Ry(pitch) * Rx(roll)
  // Eigen::Quaterniond q = roll_ang * pitch_ang * yaw_ang;
  // Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  // startPose.orientation.w = q.w();
  // startPose.orientation.x = q.x();
  // startPose.orientation.y = q.y();
  // startPose.orientation.z = q.z();

  geometry_msgs::msg::Pose goalPose;
  // goalPose = startPose;
  goalPose.position.x = 0.5;
  goalPose.position.y = 0.1;
  goalPose.position.z = 0.5;
  goalPose.orientation.x = 0.70709;
  goalPose.orientation.y = 9.8864e-05;
  goalPose.orientation.z = 0.70712;
  goalPose.orientation.w = -2.1146e-05;

  pathPlanRequest->start = startPose;
  pathPlanRequest->starting_joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, -M_PI_2, 0.0};
  pathPlanRequest->goal = goalPose;
  pathPlanRequest->delta_time = 0.02; // 20 ms between waypoints
  pathPlanRequest->goal_tolerance = 0.01; // 10 mm tolerance
  pathPlanRequest->max_iterations = 25000; // Max iterations for planning
  pathPlanRequest->planning_method = PlanPath::Request::PLANNING_METHOD_TASK_SPACE; // "task_space" or "whole_body"
  const double dt = pathPlanRequest->delta_time;

  // Publish the goal pose
  this->queryPosePub->publish(startPose);
  // this->goalPosePub->publish(goalPose);

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
  RCLCPP_INFO(this->get_logger(),
    "Received plan_path response: success=%s, end_effector_path.len=%zu, joint_trajectory.points=%zu, ee_velocity_traj=%zu",
    pathPlanResponse->success ? "true" : "false", ee_path_len, jt_points, ee_vels
  );

  if (ee_path_len > 0) {
    const auto& firstEEPose = pathPlanResponse->end_effector_path.poses.front().pose;
    const auto& lastEEPose = pathPlanResponse->end_effector_path.poses.back().pose;
    RCLCPP_INFO(this->get_logger(), "First EE pose: (%.4f, %.4f, %.4f) [m] (%.4f, %.4f, %.4f, %.4f)",
      firstEEPose.position.x, firstEEPose.position.y, firstEEPose.position.z,
      firstEEPose.orientation.x, firstEEPose.orientation.y, firstEEPose.orientation.z, firstEEPose.orientation.w
    );
    RCLCPP_INFO(this->get_logger(), "Last EE pose: (%.4f, %.4f, %.4f) [m] (%.4f, %.4f, %.4f, %.4f)",
      lastEEPose.position.x, lastEEPose.position.y, lastEEPose.position.z,
      lastEEPose.orientation.x, lastEEPose.orientation.y, lastEEPose.orientation.z, lastEEPose.orientation.w
    );
  }
  if (ee_vels > 0) {
    const auto& firstEEVel = pathPlanResponse->end_effector_velocity_trajectory.front().twist.linear;
    const auto& lastEEVel = pathPlanResponse->end_effector_velocity_trajectory.back().twist.linear;
    RCLCPP_INFO(this->get_logger(), "First EE linear velocity: (%.6f, %.6f, %.6f)", firstEEVel.x, firstEEVel.y, firstEEVel.z);
    RCLCPP_INFO(this->get_logger(), "Last EE linear velocity: (%.6f, %.6f, %.6f)", lastEEVel.x, lastEEVel.y, lastEEVel.z);
  }

  if (pathPlanResponse->success) {
    RCLCPP_INFO(this->get_logger(), "Planning succeeded");
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", pathPlanResponse->error_message.c_str());
  }

  // Begin streaming EE velocity commands to follow the path
  this->startEEVelocityStreaming(pathPlanResponse->end_effector_velocity_trajectory, dt);
  // this->startJointVelocityStreaming(pathPlanResponse->joint_trajectory, dt);
}

void PFDemo::createAndPublishObstacles() {
  potential_fields_interfaces::msg::ObstacleArray allObstacles;

  // // Real Box in the way
  // potential_fields_interfaces::msg::Obstacle box;
  // box.frame_id = "box";
  // box.type = "Box";
  // box.group = "Static";
  // box.pose.position.x = 0.66;
  // box.pose.position.y = 0.00;
  // box.pose.position.z = (0.6 / 2);
  // box.pose.orientation.x = 0.0;
  // box.pose.orientation.y = 0.0;
  // box.pose.orientation.z = 0.0;
  // box.pose.orientation.w = 1.0;
  // box.length = 0.15;
  // box.width = 0.7;
  // box.height = 0.6;
  // allObstacles.obstacles.push_back(box);

  // Real Cylinder in the way
  potential_fields_interfaces::msg::Obstacle cyl;
  cyl.frame_id = "pitcher";
  cyl.type = "Cylinder";
  cyl.group = "Static";
  cyl.radius = 0.18 / 2.0;
  cyl.height = 0.3;
  cyl.pose.position.x = 0.62;
  cyl.pose.position.y = 0.00;
  cyl.pose.position.z = (cyl.height / 2.0);
  cyl.pose.orientation.x = 0.0;
  cyl.pose.orientation.y = 0.0;
  cyl.pose.orientation.z = 0.0;
  cyl.pose.orientation.w = 1.0;
  allObstacles.obstacles.push_back(cyl);

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
      "Found TF (%s -> %s): (%.2f, %.2f, %.2f) [m]", this->fixedFrame.c_str(), this->eeLinkName.c_str(),
      eePose.position.x, eePose.position.y, eePose.position.z
    );
    return eePose;
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(),
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

void PFDemo::startJointVelocityStreaming(const trajectory_msgs::msg::JointTrajectory& jointVels, double dt) {
  // If an existing stream is in progress, stop it first
  this->stopJointVelocityStreaming();
  if (jointVels.points.empty() || dt <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Requested Joint velocity stream has no points or non-positive dt (dt=%.6f)", dt);
    return;
  }

  this->jointVelocityBuffer = jointVels;
  trajectory_msgs::msg::JointTrajectoryPoint zeroCmd;
  const size_t numJoints = jointVels.points[0].velocities.size();
  zeroCmd.positions = jointVels.points.back().positions;
  zeroCmd.velocities = std::vector<double>(numJoints, 0.0);
  // Append the zero command to stop the robot at the end
  this->jointVelocityBuffer.points.push_back(zeroCmd);
  this->jointVelocityIndex = 0;
  this->jointVelocityDt = dt;
  this->isStreamingJointVel = true;

  // Create and start a timer to publish at the requested period
  this->jointVelocityTimer = this->create_wall_timer(
    std::chrono::duration<double>(this->jointVelocityDt),
    std::bind(&PFDemo::jointVelocityTimerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Started Joint velocity streaming: %zu points @ %.3f Hz", jointVelocityBuffer.points.size(), 1.0 / jointVelocityDt);
}

void PFDemo::stopJointVelocityStreaming() {
  if (this->jointVelocityTimer) {
    this->jointVelocityTimer->cancel();
    this->jointVelocityTimer.reset();
  }
  if (this->isStreamingJointVel) {
    RCLCPP_INFO(this->get_logger(), "Stopped joint velocity streaming after %zu/%zu points", jointVelocityIndex - 1, jointVelocityBuffer.points.size());
  }
  this->isStreamingJointVel = false;
  this->jointVelocityBuffer.points.clear();
  this->jointVelocityIndex = 0;
}

void PFDemo::jointVelocityTimerCallback() {
  if (!this->isStreamingJointVel) {
    // Nothing to stream, defensive cancel
    if (this->jointVelocityTimer) {
      this->jointVelocityTimer->cancel();
      this->jointVelocityTimer.reset();
    }
    return;
  }

  if (this->jointVelocityIndex >= this->jointVelocityBuffer.points.size()) {
    // Finished streaming
    this->stopJointVelocityStreaming();
    return;
  }

  auto cmd = this->jointVelocityBuffer.points[this->jointVelocityIndex++];
  this->jointVelocityPub->publish(cmd);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PFDemo>());
  rclcpp::shutdown();
  return 0;
}
