#include "robot_plugins/franka_plugin.hpp"
#include "geofik.h"

FrankaIKSolver::FrankaIKSolver() : IKSolver() {
  this->homeTransformOE = franka_fk(this->homeJointAngles);
}

bool FrankaIKSolver::computeIK(const geometry_msgs::msg::PoseStamped& targetPose,
  sensor_msgs::msg::JointState& jointState) {
  return false;
}

FrankaPlugin::FrankaPlugin(const std::string& hostname) : MotionPlugin() {
  this->assignIKSolver(std::make_unique<FrankaIKSolver>());
  this->initializeRobot(hostname);
}

bool FrankaPlugin::initializeRobot(const std::string& hostname) {
  try {
    this->robot = std::make_unique<franka::Robot>(hostname);
    setDefaultBehavior(*this->robot);
  }
  catch (const franka::Exception& e) {
    std::cerr << "Failed to connect to Franka robot at " << hostname << ": " << e.what() << std::endl;
    return false;
  }
}

bool FrankaPlugin::startControlLoop(const franka::Duration& movementDuration) {
  if (!this->currentEEVelocity) {
    std::cerr << "No end-effector velocity command set" << std::endl;
    return false;
  }

  auto controlTime = franka::Duration(0.0);
  auto controlCallback = [this, &controlTime, &movementDuration](const franka::RobotState& robotState,
    franka::Duration period) -> franka::CartesianVelocities {
    // Move along the controlTime to see if we should finish the motion
    controlTime += period;
    if (controlTime >= movementDuration) {
      return franka::MotionFinished(*this->currentEEVelocity);
    }
    return *this->currentEEVelocity;
  };

  try {
    // Start the control using joint impedance mode
    bool motionFinished = false;
    auto activeControl = this->robot->startCartesianVelocityControl(
      research_interface::robot::Move::ControllerMode::kJointImpedance
    );
    while (!motionFinished) {
      auto robotStateAndDuration = activeControl->readOnce();
      const franka::RobotState& robotState = robotStateAndDuration.first;
      const franka::Duration& period = robotStateAndDuration.second;
      auto cartesianVelocities = controlCallback(robotState, period);
      motionFinished = cartesianVelocities.motion_finished;
      activeControl->writeOnce(cartesianVelocities);
    }
  }
  catch (const franka::Exception& e) {
    std::cerr << "Control loop failed: " << e.what() << std::endl;
    return false;
  }
  return true;
}

bool FrankaPlugin::sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) {
  const auto velocityCommand = franka::CartesianVelocities({
    endEffectorTwist.linear.x,
    endEffectorTwist.linear.y,
    endEffectorTwist.linear.z,
    endEffectorTwist.angular.x,
    endEffectorTwist.angular.y,
    endEffectorTwist.angular.z
    });
  this->currentEEVelocity = std::make_unique<franka::CartesianVelocities>(velocityCommand);
  return true;
}

bool FrankaPlugin::sendJointStates(const sensor_msgs::msg::JointState& js) { return false; }

bool FrankaPlugin::readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) {
  try {
    franka::RobotState robotState = this->robot->readOnce();
    // Convert franka::RobotState to sensor_msgs::msg::JointState
    if (!this->clock) {
      js.header.stamp = this->clock->now();
    }
    else {
      js.header.stamp = rclcpp::Time(0);
    }
    js.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    js.position = std::vector<double>(robotState.q.data(), robotState.q.data() + 7);
    js.velocity = std::vector<double>(robotState.dq.data(), robotState.dq.data() + 7);
    js.effort = std::vector<double>(robotState.tau_J.data(), robotState.tau_J.data() + 7);

    // Set end-effector pose
    if (this->clock) {
      endEffectorPose.header.stamp = this->clock->now();
    }
    else {
      endEffectorPose.header.stamp = rclcpp::Time(0);
    }
    endEffectorPose.header.frame_id = "panda_link0";
    endEffectorPose.pose.position.x = robotState.O_T_EE[12];
    endEffectorPose.pose.position.y = robotState.O_T_EE[13];
    endEffectorPose.pose.position.z = robotState.O_T_EE[14];
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix <<
      robotState.O_T_EE[0], robotState.O_T_EE[1], robotState.O_T_EE[2],
      robotState.O_T_EE[4], robotState.O_T_EE[5], robotState.O_T_EE[6],
      robotState.O_T_EE[8], robotState.O_T_EE[9], robotState.O_T_EE[10];
    Eigen::Quaterniond quaternion(rotationMatrix);
    endEffectorPose.pose.orientation.x = quaternion.x();
    endEffectorPose.pose.orientation.y = quaternion.y();
    endEffectorPose.pose.orientation.z = quaternion.z();
    endEffectorPose.pose.orientation.w = quaternion.w();

    return true;
  }
  catch (const franka::Exception& e) {
    std::cerr << "Failed to read Franka robot state: " << e.what() << std::endl;
    return false;
  }
}
