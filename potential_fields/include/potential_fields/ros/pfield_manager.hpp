// Copyright 2025 argallab
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

#ifndef PFIELD_MANAGER_HPP
#define PFIELD_MANAGER_HPP

#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <cctype>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "potential_fields_interfaces/msg/obstacle.hpp"
#include "potential_fields_interfaces/msg/obstacle_array.hpp"
#include "potential_fields_interfaces/srv/plan_path.hpp"
#include "potential_fields_interfaces/srv/compute_autonomy_vector.hpp"

#include "pfield/pfield.hpp"
#include "pfield/pf_obstacle.hpp"
#include "pfield/pf_kinematics.hpp"
#include "solvers/ik_solver.hpp"
#include "robot_plugins/motion_plugin.hpp"

#include "urdf/model.h"
#include "urdf_parser/urdf_parser.h"

#include "tf2_eigen/tf2_eigen.hpp"

#include "rclcpp/rclcpp.hpp"

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Pose = geometry_msgs::msg::Pose;
using Point = geometry_msgs::msg::Point;
using Quaternion = geometry_msgs::msg::Quaternion;
using Path = nav_msgs::msg::Path;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Obstacle = potential_fields_interfaces::msg::Obstacle;
using ObstacleArray = potential_fields_interfaces::msg::ObstacleArray;
using PlanPath = potential_fields_interfaces::srv::PlanPath;
using ComputeAutonomyVector = potential_fields_interfaces::srv::ComputeAutonomyVector;
using JointState = sensor_msgs::msg::JointState;

class PotentialFieldManager : public rclcpp::Node {
public:
  PotentialFieldManager();
  ~PotentialFieldManager() = default;

  static Quaternion getQuaternionFromYaw(double yaw) {
    Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw / 2.0);
    q.w = cos(yaw / 2.0);
    return q;
  }

private:
  double visualizerFrequency; // frequency for updating visualization [Hz]
  double attractiveGain; // Attractive gain [Ns/m]
  double rotationalAttractiveGain; // Rotational attractive gain [Ns/m]
  double repulsiveGain; // Repulsive gain [Ns/m]
  double maxLinearVelocity; // Maximum Linear Velocity [m/s]
  double maxAngularVelocity; // Maximum Angular Velocity [rad/s]
  double maxLinearAcceleration; // Maximum Linear Acceleration [m/s^2]
  double maxAngularAcceleration; // Maximum Angular Acceleration [rad/s^2]
  double influenceDistance; // Influence distance for obstacle repulsion [m]
  double visualizerBufferArea; // Extra area around obstacles and goal to visualize the PF [m]
  double fieldResolution; // Resolution of the potential field grid [m]
  bool visualizeFieldVectors; // Whether to visualize the potential field vectors
  std::string fixedFrame; // RViz fixed frame for visualization and PF computation
  std::string eeFrame; // End-effector frame name for planning and kinematics
  std::string urdfFileName; // URDF file name
  std::string motionPluginType; // Motion Plugin Type [e.g., "null", "franka", etc.]
  std::string visualizationMethod; // Visualization method: "task_space" or "whole_body_velocity"
  std::shared_ptr<pfield::PotentialField> pField; // Potential Field Instance containing main PF functionality
  std::unique_ptr<MotionPlugin> motionPlugin; // The MotionPlugin containing robot-specific functions like IK and state reading
  std::shared_ptr<pfield::IKSolver> ikSolver; // The IKSolver obtained from the MotionPlugin
  pfield::SpatialVector queryPose; // Current Query Pose for autonomy vector computation
  pfield::TaskSpaceTwist prevQueryTwist; // Previous query twist for acceleration limiting
  rclcpp::Time lastQueryUpdate;  // Timestamp of last query pose integration step
  pfield::TaskSpaceTwist prevTwist; // Previous twist for acceleration limiting
  std::vector<double> currentJointAngles; // Current joint angles for robot obstacle visualization

  // Visualization Constants
  const double TASK_SPACE_POSE_SPHERE_DIAMETER = 0.05; // Scale for goal marker [m]
  const double TASK_SPACE_AXIS_LENGTH = 0.2; // Length of axis indicators [m]
  const double TASK_SPACE_AXIS_SHAFT_DIAMETER = 0.01; // Diameter of axis shafts [m]
  const double TASK_SPACE_AXIS_HEAD_DIAMETER = 0.025; // Diameter of axis heads [m]

  rclcpp::TimerBase::SharedPtr timer; // Timer to periodically update the potential field
  rclcpp::Publisher<MarkerArray>::SharedPtr pFieldMarkerPub; // Publisher for PF Markers
  rclcpp::Publisher<JointState>::SharedPtr planningJointStatePub; // Publisher for planning joint states
  rclcpp::Publisher<Path>::SharedPtr plannedEndEffectorPathPub; // Publisher for planned end-effector path
  rclcpp::Subscription<Pose>::SharedPtr goalPoseSub; // Subscriber for the goal pose
  rclcpp::Subscription<Pose>::SharedPtr queryPoseSub; // Subscriber for query poses
  rclcpp::Subscription<ObstacleArray>::SharedPtr obstacleSub; // Subscriber for obstacles
  rclcpp::Service<PlanPath>::SharedPtr pathPlanningService; // Now hosted here
  rclcpp::Service<ComputeAutonomyVector>::SharedPtr autonomyVectorService; // Service to compute velocity vector at a given pose

  void timerCallback();

  // Service callbacks
  void handlePlanPath(const PlanPath::Request::SharedPtr request, PlanPath::Response::SharedPtr response);
  void handleComputeAutonomyVector(
    const ComputeAutonomyVector::Request::SharedPtr request, ComputeAutonomyVector::Response::SharedPtr response);

  /**
   * @brief Given a potential field, generate all visualization markers
   *        (goal, obstacles, obstacle influence zones, velocity vectors)
   *
   * @param pf The potential field instance
   * @return MarkerArray The marker array containing all visualization markers
   */
  MarkerArray visualizePF(std::shared_ptr<pfield::PotentialField> pf);

  MarkerArray createQueryPoseMarker();

  MarkerArray createThresholdMarkers(std::shared_ptr<pfield::PotentialField> pf);

  /**
   * @brief Get Obstacle and Obstacle Influence Zone markers from a potential field
   *
   * @param pf The potential field instance
   * @return MarkerArray The marker array containing all obstacle markers
   */
  MarkerArray createObstacleMarkers(std::shared_ptr<pfield::PotentialField> pf);

  /**
   * @brief Build obstacle shape markers and their semi-transparent influence-zone overlays
   *        for all environment and robot obstacles.
   *
   * @param obstacles Combined list of environment and robot obstacles.
   * @param influenceDistance Repulsive influence radius [m].
   * @return MarkerArray Markers in namespaces "robot_obstacles", "environment_obstacles",
   *         and "environment_influence_zones".
   */
  MarkerArray createObstaclesWithInfluenceZoneMarkerArray(
    const std::vector<pfield::PotentialFieldObstacle>& obstacles,
    double influenceDistance);

  /**
   * @brief Build semi-transparent ghost mesh markers for CAPSULE robot obstacles that
   *        were fitted from a mesh, so the original mesh is visible alongside the capsule
   *        approximation in RViz.
   *
   * @param robotObstacles Robot link obstacles.
   * @return MarkerArray Markers in namespace "robot_mesh_ghost".
   */
  MarkerArray createGhostMeshOverlayMarkerArray(
    const std::vector<pfield::PotentialFieldObstacle>& robotObstacles);

  /**
   * @brief Build small sphere markers at each robot link control point used by the
   *        whole-body velocity repulsion, for visualization in RViz.
   *
   * @param robotObstacles Robot link obstacles.
   * @return MarkerArray Markers in namespace "robot_control_points".
   */
  MarkerArray createRobotLinkControlPointsMarkerArray(
    const std::vector<pfield::PotentialFieldObstacle>& robotObstacles);

  /**
   * @brief Create a marker for the goal position in the potential field
   *
   * @param pf The potential field instance
   * @return MarkerArray The marker array containing the goal marker and orientation indicator
   */
  MarkerArray createGoalMarker(std::shared_ptr<pfield::PotentialField> pf);

  /**
   * @brief Create velocity vector markers for visualization in RViz
   *
   * @note The density of the vectors is determined by the field resolution parameter
   *       and the vector sizes are normalized but the color intensity represents the magnitude
   *
   * @param pf The potential field instance
   * @return MarkerArray The marker array containing all velocity vector markers
   */
  MarkerArray createPotentialVectorMarkers(std::shared_ptr<pfield::PotentialField> pf);

  /**
   * @brief Fuses the two twists using a weighted alpha parameter
   *
   * @note alpha = 0.0 -> only twist1, alpha = 1.0 -> only twist2
   *
   * @param twist1 The first twist to fuse
   * @param twist2 The second twist to fuse
   * @param alpha The weight for the second twist [0.0, 1.0]
   * @return geometry_msgs::msg::Twist The fused twist
   */
  geometry_msgs::msg::Twist fuseTwists(
    const geometry_msgs::msg::Twist::SharedPtr twist1,
    const geometry_msgs::msg::Twist::SharedPtr twist2,
    const double alpha);

  /**
   * @brief Clamps the twist to the specified limits.
   *
   * @param twist The twist to clamp
   * @param limits A twist message where linear and angular components represent the maximum allowed values
   * @return geometry_msgs::msg::Twist The clamped twist as a new message
   */
  geometry_msgs::msg::Twist clampTwist(const geometry_msgs::msg::Twist& twist, const geometry_msgs::msg::Twist& limits);

  /**
   * @brief Exports the potential field data to a CSV file for external analysis or visualization.
   *
   * @param pf The potential field instance
   * @param filename The name of the CSV file to export the data to
   */
  void exportFieldDataToCSV(std::shared_ptr<pfield::PotentialField> pf, const std::string& filename);

  /**
   * @brief Integrates the internal query pose forward one timestep using the
   *        potential field wrench, motion constraints, and velocity integration.
   *
   * @note Updates members: lastQueryUpdate, queryPose, prevQueryTwist.
   */
  void integrateQueryPoseFromField();
};

#endif // PFIELD_MANAGER_HPP
