#include "pfield_teleop_demo.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <filesystem>

template<typename T>
using ServiceResponseFuture = rclcpp::Client<T>::SharedFuture;

PFTeleopDemo::PFTeleopDemo() : Node("pfield_teleop_demo") {
  RCLCPP_INFO(this->get_logger(), "PFTeleopDemo Initialized");

  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  this->eeLinkName = this->declare_parameter("ee_link_name", "link_tcp"); // End-effector link name
  this->fuseAlpha = this->declare_parameter("fuse_alpha", 0.5); // Fusion Alpha, 0 is teleop twist, 1 is PFTwist
  // Get parameters from yaml file
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();
  this->eeLinkName = this->get_parameter("ee_link_name").as_string();
  this->fuseAlpha = this->get_parameter("fuse_alpha").as_double();

  this->goalPosePub = this->create_publisher<geometry_msgs::msg::Pose>("pfield/planning_goal_pose", 10);
  this->queryPosePub = this->create_publisher<geometry_msgs::msg::Pose>("pfield/query_pose", 10);

  // Wait for the service to be available
  this->pfTwistClient = this->create_client<ComputePFTwist>("pfield/plan_path");
  while (!this->pfTwistClient->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for the ComputeAutonomyVector service to be available...");
  }
  RCLCPP_INFO(this->get_logger(), "ComputeAutonomyVector service is available.");

  this->eeVelocityPub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/robot_action", 10);
  this->jointVelocityPub = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/joint_vel_control", 10);
  this->obstaclePub = this->create_publisher<potential_fields_interfaces::msg::ObstacleArray>("/pfield/obstacles", 10);

  this->teleopTwistSub = this->create_subscription<geometry_msgs::msg::TwistStamped>("/pfield_demo/teleop_twist",
    10, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { this->latestTeleopTwist = *msg; }
  );

  this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer, this);

  this->createAndPublishObstacles();

  const double teleopFreq = 50.0; // [Hz]
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / teleopFreq),
    std::bind(&PFTeleopDemo::timerCallback, this)
  );
}

void PFTeleopDemo::timerCallback() {
  // Call ComputePfTwist service
  // Create a request for the ComputeAutonomyVector service
  auto computePFTwistRequest = std::make_shared<ComputePFTwist::Request>();
  geometry_msgs::msg::PoseStamped queryPoseStamped;
  queryPoseStamped.header.frame_id = this->fixedFrame;
  queryPoseStamped.header.stamp = this->now();
  queryPoseStamped.pose = this->getEndEffectorPose();
  computePFTwistRequest->query_pose = queryPoseStamped;
  // computePFTwistRequest->joint_angles = std::vector<double>(7, 0.0); // Optional
  // computePFTwistRequest->delta_time = 1.0 / 50.0; // Optional
  // computePFTwistRequest->prev_joint_velocities = std::vector<double>(7, 0.0); // Optional
  // computePFTwistRequest->planning_method = ComputePFTwist::Request::PLANNING_METHOD_WHOLE_BODY; // Optional

  // Send request asynchronously and attach a callback to process the result
  this->pfTwistClient->async_send_request(computePFTwistRequest,
    [this](ServiceResponseFuture<ComputePFTwist> future) { this->handleComputeAutonomyVectorResponse(future); }
  );
  // Handle callback will internally update this->latestPFTwist
  // Joystick Twist will automatically update latestTeleopTwist from subscriber
  // Fuse both twists together using specified alpha
  geometry_msgs::msg::Twist fusedTwist = this->fuseTwists(
    this->latestTeleopTwist.twist, this->latestPFTwist.twist, this->fuseAlpha
  );
  geometry_msgs::msg::TwistStamped eeTwist;
  eeTwist.header.frame_id = this->fixedFrame;
  eeTwist.header.stamp = this->now();
  eeTwist.twist = fusedTwist;
  // Publish to Robot Action
  this->eeVelocityPub->publish(eeTwist);
}

geometry_msgs::msg::Twist PFTeleopDemo::fuseTwists(
  const geometry_msgs::msg::Twist twist1,
  const geometry_msgs::msg::Twist twist2,
  const double alpha) {
  geometry_msgs::msg::Twist fusedTwist;

  // Alpha = 0 -> A, Alpha = 1 -> B
  auto fuseValue = [](double A, double B, double alpha) {
    return alpha * B + (1.0 - alpha) * A;
  };

  // Simple linear fusion based on alpha parameter. Missing inputs are treated as zeros.
  fusedTwist.linear.x = fuseValue(twist1.linear.x, twist2.linear.x, alpha);
  fusedTwist.linear.y = fuseValue(twist1.linear.y, twist2.linear.y, alpha);
  fusedTwist.linear.z = fuseValue(twist1.linear.z, twist2.linear.z, alpha);
  fusedTwist.angular.x = fuseValue(twist1.angular.x, twist2.angular.x, alpha);
  fusedTwist.angular.y = fuseValue(twist1.angular.y, twist2.angular.y, alpha);
  fusedTwist.angular.z = fuseValue(twist1.angular.z, twist2.angular.z, alpha);

  return fusedTwist;
}


void PFTeleopDemo::handleComputeAutonomyVectorResponse(rclcpp::Client<ComputePFTwist>::SharedFuture future) {
  auto computePFTwistResponse = future.get();
  if (!computePFTwistResponse) {
    RCLCPP_ERROR(this->get_logger(), "plan_path service returned an empty response (async)");
    return;
  }
  this->latestPFTwist = computePFTwistResponse->autonomy_vector;
}

void PFTeleopDemo::createAndPublishObstacles() {
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

geometry_msgs::msg::Pose PFTeleopDemo::getEndEffectorPose() {
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

void PFTeleopDemo::startEEVelocityStreaming(const std::vector<geometry_msgs::msg::TwistStamped>& eeVels, double dt) {
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
    std::bind(&PFTeleopDemo::eeVelocityTimerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Started EE velocity streaming: %zu points @ %.3f Hz", eeVelocityBuffer.size(), 1.0 / eeVelocityDt);
}

void PFTeleopDemo::stopEEVelocityStreaming() {
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

void PFTeleopDemo::eeVelocityTimerCallback() {
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

void PFTeleopDemo::startJointVelocityStreaming(const trajectory_msgs::msg::JointTrajectory& jointVels, double dt) {
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
    std::bind(&PFTeleopDemo::jointVelocityTimerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Started Joint velocity streaming: %zu points @ %.3f Hz", jointVelocityBuffer.points.size(), 1.0 / jointVelocityDt);
}

void PFTeleopDemo::stopJointVelocityStreaming() {
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

void PFTeleopDemo::jointVelocityTimerCallback() {
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
