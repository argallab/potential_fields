#ifndef ROBOT_PARSER_HP
#define ROBOT_PARSER_HP

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "potential_fields_interfaces/msg/obstacle.hpp"
#include "potential_fields_interfaces/msg/obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include "urdf_parser/urdf_parser.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/parameter_client.hpp"
#include "pfield/pf_kinematics.hpp"
#include "pfield/pf_obstacle.hpp"

using Obstacle = potential_fields_interfaces::msg::Obstacle;
using ObstacleArray = potential_fields_interfaces::msg::ObstacleArray;
using JointState = sensor_msgs::msg::JointState;

struct CollisionCatalogEntry {
  std::string id;         // unique obstacle id (link::name or link::colN)
  std::string linkName;  // link this collision belongs to
  urdf::CollisionSharedPtr col; // collision element
};

class RobotParser : public rclcpp::Node {
public:
  RobotParser();
  ~RobotParser() = default;

private:
  double robotUpdateFrequency; // Frequency for updating Robot's TF frames [Hz]
  std::string urdfFileName; // URDF file name
  std::string fixedFrame; // RViz fixed frame
  PFKinematics kinematicModel; // Pinocchio kinematic model for FK
  urdf::Model robotModel; // URDF model
  std::vector<CollisionCatalogEntry> collisionCatalog; // Catalog of collision objects from URDF
  std::vector<std::string> collisionLinkNames; // Names of links with collision geometry
  // Cached obstacle geometry templates aligned to collisionCatalog/collisionLinkNames
  std::vector<Obstacle> obstacleGeometryTemplates; // pose will be updated per callback
  bool kinematicsCachesInitialized = false;

  rclcpp::Subscription<JointState>::SharedPtr jointStateSub; // Subscriber for live JointStates
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstaclePub; // Publisher for Obstacles from Robot Collision Geometry

  void jointStateCallback(const JointState::SharedPtr msg);

  /**
   * @brief Using cached transforms vector aligned to collisionLinkNames,
   *        build obstacles using updated poses
   *
   * @param transforms The new poses of collision links, aligned to collisionLinkNames
   * @return ObstacleArray Obstacle array message with updated poses
   */
  ObstacleArray buildObstaclesFromTransforms(const std::vector<Eigen::Affine3d>& transforms);

  /**
   * @brief Builds the collision catalog from the URDF model, holding info about
   *        each collision object's link, name, and Collision pointer.
   *
   * @param model The URDF model
   * @return std::vector<CollisionCatalogEntry> The built collision catalog
   */
  std::vector<CollisionCatalogEntry> buildCollisionCatalog(urdf::Model& model);

  /**
   * @brief Builds an Obstacle message from a URDF Collision object and given pose
   *
   * @param frameID The frame ID for the obstacle's pose to be defined in
   * @param collisionObject The URDF Collision object, defining geometry and type
   * @param position The position of the obstacle
   * @param orientation The orientation of the obstacle
   * @return Obstacle The built Obstacle message
   */
  Obstacle obstacleFromCollisionObject(const std::string& frameID, const urdf::Collision& collisionObject,
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
};

#endif // ROBOT_PARSER_HP
