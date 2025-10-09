#include "robot_plugins/franka_plugin.hpp"
#include "weighted_ik.h"
#include <cmath>

// struct WeightedIKResult {
//   bool success;
//   std::array<double, 7> joint_angles;
//   double q7_optimal;
//   double score;
//   double manipulability;
//   double neutral_distance;
//   double current_distance;
//   int solution_index;
//   std::array<std::array<double, 6>, 7> jacobian;

//   int total_solutions_found;
//   int valid_solutions_count;
//   int q7_values_tested;
//   int optimization_iterations;  // Number of iterations used by optimization algorithm
//   long duration_microseconds;
// };

FrankaIKSolver::FrankaIKSolver(IKSolverSearchParameters params) : IKSolver(), ikParams(params) {
  this->homeTransformOE = franka_fk(this->homeJointAngles);
  this->solver = std::make_unique<WeightedIKSolver>(
    /*neutral_pose=*/ this->homeJointAngles,
    /*weight_manip=*/ ikParams.manipulabilityWeight,
    /*weight_neutral=*/ ikParams.neutralWeight,
    /*weight_current=*/ ikParams.currentWeight,
    /*verbose=*/ false
  );
}

bool FrankaIKSolver::solve(
  const Eigen::Isometry3d& targetPose,
  const std::vector<double>& seed,
  std::vector<double>& solution,
  Eigen::Matrix<double, 6, Eigen::Dynamic>& J) {
  const std::array<double, 3> targetPosition = {
    targetPose.translation().x(), targetPose.translation().y(), targetPose.translation().z()
  };
  const std::array<double, 9> targetOrientation = {
    targetPose.rotation()(0, 0), targetPose.rotation()(0, 1), targetPose.rotation()(0, 2),
    targetPose.rotation()(1, 0), targetPose.rotation()(1, 1), targetPose.rotation()(1, 2),
    targetPose.rotation()(2, 0), targetPose.rotation()(2, 1), targetPose.rotation()(2, 2)
  };
  const std::array<double, 7> currentConfiguration = {
    seed[0], seed[1], seed[2], seed[3], seed[4], seed[5], seed[6]
  };
  WeightedIKResult result = this->solver->solve_q7_optimized(
    /*target_position=*/targetPosition,
    /*target_orientation=*/targetOrientation,
    /*current_pose=*/currentConfiguration,
    /*q7_min=*/ this->ikParams.q7Min,
    /*q7_max=*/ this->ikParams.q7Max,
    /*tolerance=*/ this->ikParams.ikTolerance,
    /*max_iterations=*/ this->ikParams.ikMaxIterations
  );
  // Copy the result joint angles to the output solution vector
  solution = std::vector<double>(result.joint_angles.begin(), result.joint_angles.end());
  // Copy the result Jacobian to the output J matrix, resizing to 6x7 since  Franka is 7-DOF
  J.resize(6, 7);
  for (size_t col = 0; col < J.cols(); ++col) {
    for (size_t row = 0; row < J.rows(); ++row) {
      J(row, col) = result.jacobian.at(col).at(row);
    }
  }
  return result.success;
}

FrankaPlugin::FrankaPlugin(const std::string& hostname) : MotionPlugin() {
  this->assignIKSolver(std::make_shared<FrankaIKSolver>());
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
