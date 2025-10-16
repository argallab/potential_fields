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

  rclcpp::Subscription<JointState>::SharedPtr jointStateSub; // Subscriber for live JointStates
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstaclePub; // Publisher for Obstacles from Robot Collision Geometry

  void jointStateCallback(const JointState::SharedPtr msg);

  ObstacleArray extractObstaclesFromCatalog(const std::vector<CollisionCatalogEntry>& catalog,
    const std::unordered_map<std::string, Eigen::Affine3d>& linkToTransformMap);
  std::vector<CollisionCatalogEntry> buildCollisionCatalog(urdf::Model& model);
  Obstacle obstacleFromCollisionObject(const std::string& frameID, const urdf::Collision& collisionObject,
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
};

#endif // ROBOT_PARSER_HP
