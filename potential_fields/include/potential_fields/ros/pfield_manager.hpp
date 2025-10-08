#ifndef PFIELD_MANAGER_HPP
#define PFIELD_MANAGER_HPP

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "potential_fields_interfaces/msg/obstacle.hpp"
#include "potential_fields_interfaces/msg/obstacle_array.hpp"
#include "potential_fields_interfaces/srv/plan_path.hpp"
#include "potential_fields_interfaces/srv/compute_autonomy_vector.hpp"
#include "potential_fields_interfaces/srv/p_field_step.hpp"

#include "pfield/pfield.hpp"
#include "robot_plugins/ik_solver.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::Point;
using Quaternion = geometry_msgs::msg::Quaternion;
using Path = nav_msgs::msg::Path;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Obstacle = potential_fields_interfaces::msg::Obstacle;
using ObstacleArray = potential_fields_interfaces::msg::ObstacleArray;
using PlanPath = potential_fields_interfaces::srv::PlanPath;
using ComputeAutonomyVector = potential_fields_interfaces::srv::ComputeAutonomyVector;
using PFieldStep = potential_fields_interfaces::srv::PFieldStep;

struct PFLimits {
  double minX; // Minimum X coordinate of the bounding box [m]
  double maxX; // Maximum X coordinate of the bounding box [m]
  double minY; // Minimum Y coordinate of the bounding box [m]
  double maxY; // Maximum Y coordinate of the bounding box [m]
  double minZ; // Minimum Z coordinate of the bounding box [m]
  double maxZ; // Maximum Z coordinate of the bounding box [m]
};

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
  double maxForce; // Maximum force [N]
  double influenceZoneScale; // Influence zone scaling factor
  std::string fixedFrame; // RViz fixed frame for visualization and PF computation
  // Potential field instances (shared_ptr to avoid inadvertent copying when passing to visualization helpers)
  std::shared_ptr<PotentialField> pField; // Primary Potential Field
  std::shared_ptr<PotentialField> planningPField; // Planning Potential Field
  double visualizerBufferArea; // Extra area around obstacles and goal to visualize the PF [m]
  double fieldResolution; // Resolution of the potential field grid [m]

  rclcpp::TimerBase::SharedPtr timer; // Timer to periodically update the potential field
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamicTfBroadcaster; // Dynamic transform broadcaster
  std::shared_ptr<tf2_ros::Buffer> tfBuffer; // TF buffer for transform lookups
  std::shared_ptr<tf2_ros::TransformListener> tfListener; // TF Listener for populating the TF buffer
  rclcpp::Publisher<MarkerArray>::SharedPtr pFieldMarkerPub; // Publisher for PF Markers
  rclcpp::Publisher<MarkerArray>::SharedPtr planningPFieldMarkerPub; // Publisher for Planning PF Markers
  rclcpp::Subscription<PoseStamped>::SharedPtr goalPoseSub; // Subscriber for the goal pose
  rclcpp::Subscription<ObstacleArray>::SharedPtr obstacleSub; // Subscriber for obstacles
  rclcpp::Subscription<ObstacleArray>::SharedPtr planningObstacleSub; // Subscriber for planning obstacles
  // Path planning service was moved to MotionInterface; PFieldManager now offers a single-step PF evaluation
  rclcpp::Service<PFieldStep>::SharedPtr pfieldStepService; // Service to perform one planning step (evaluate PF and return next pose)
  rclcpp::Service<ComputeAutonomyVector>::SharedPtr autonomyVectorService; // Service to compute velocity vector at a given pose

  // Service callbacks
  void handlePFieldStep(const PFieldStep::Request::SharedPtr request, PFieldStep::Response::SharedPtr response);

  /**
   * @brief Given a potential field, computes its spatial limits for visualization (bounding box)
   *
   * @note Could be used for adaptive computations in the future
   *
   * @param pf The potential field instance
   * @return PFLimits The spatial limits of the potential field
   */
  PFLimits getPFLimits(std::shared_ptr<PotentialField> pf);

  /**
   * @brief Given a potential field, generate all visualization markers
   *        (goal, obstacles, obstacle influence zones, velocity vectors)
   *
   * @param pf The potential field instance
   * @return MarkerArray The marker array containing all visualization markers
   */
  MarkerArray visualizePF(std::shared_ptr<PotentialField> pf);

  /**
   * @brief Get Obstacle and Obstacle Influence Zone markers from a potential field
   *
   * @param pf The potential field instance
   * @return MarkerArray The marker array containing all obstacle markers
   */
  MarkerArray createObstacleMarkers(std::shared_ptr<PotentialField> pf);

  /**
   * @brief Create a marker for the goal position in the potential field
   *
   * @param pf The potential field instance
   * @return MarkerArray The marker array containing the goal marker and orientation indicator
   */
  MarkerArray createGoalMarker(std::shared_ptr<PotentialField> pf);

  /**
   * @brief Create velocity vector markers for visualization in RViz
   *
   * @note The density of the vectors is determined by the field resolution parameter
   *       and the vector sizes are normalized but the color intensity represents the magnitude
   *
   * @param pf The potential field instance
   * @return MarkerArray The marker array containing all velocity vector markers
   */
  MarkerArray createPotentialVectorMarkers(std::shared_ptr<PotentialField> pf);

  /**
   * @brief Exports the potential field data to a CSV file for external analysis or visualization.
   *
   * @param pf The potential field instance
   * @param filename The name of the CSV file to export the data to
   */
  void exportFieldDataToCSV(std::shared_ptr<PotentialField> pf, const std::string& filename);
};

#endif // PFIELD_MANAGER_HPP
