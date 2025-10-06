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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "potential_fields_interfaces/msg/obstacle.hpp"
#include "potential_fields_interfaces/msg/obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "urdf/model.h"
#include "urdf_parser/urdf_parser.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/parameter_client.hpp"

using Obstacle = potential_fields_interfaces::msg::Obstacle;
using ObstacleArray = potential_fields_interfaces::msg::ObstacleArray;
using TransformStamped = geometry_msgs::msg::TransformStamped;
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

  // Test-only lightweight constructor (skips heavy ROS/TF setup & URDF parsing)
  explicit RobotParser(bool skipInitialization) : Node("robot_parser_test"), modelLoaded(false) {
    if (!skipInitialization) {
      // Fallback to full initialization if false passed inadvertently
      robotUpdateFrequency = 50.0;
    }
  }

private:
  friend class RobotParserTestHelper; // Grants unit tests controlled access to private members
  double robotUpdateFrequency; // Frequency for updating Robot's TF frames [Hz]
  std::string robotDescription; // Robot Description
  std::string fixedFrame; // RViz fixed frame
  std::string planningTFPrefix; // Prefix applied to duplicated planning TF frames (e.g. "planning::")
  urdf::Model robotModel; // Parsed URDF model
  urdf::Model planningRobotModel; // Parsed URDF model with modified names for planning
  bool modelLoaded = false; // Whether URDF parsed successfully
  bool planningRSPLoaded = false;
  bool planningJSPParamSet = false; // true once we successfully set robot_description on any planning JSP node
  bool useJSPGui;
  const std::string planningRSPNodeName = "planning_robot_state_publisher";
  const std::string planningJSPNodeName = "planning_joint_state_publisher";
  std::string cachedPlanningRobotDescription; // cached planning URDF we generated
  size_t cachedPlanningRobotDescriptionHash = 0; // hash to avoid redundant param sets

  std::vector<CollisionCatalogEntry> collisionCatalog;
  std::vector<CollisionCatalogEntry> planningCollisionCatalog;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstaclePub;
  rclcpp::Publisher<ObstacleArray>::SharedPtr planningObstaclePub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planningRobotDescriptionPub;
  std::shared_ptr<rclcpp::SyncParametersClient> syncParamClientRSP;
  std::shared_ptr<rclcpp::SyncParametersClient> syncParamClientJSP;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamicTfBroadcaster;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;

  void timerCallback();
  void trySetPlanningRobotDescriptionParam();
  void trySetPlanningJointStatePublisherRDParam();

  std::vector<Obstacle> extractObstaclesFromCatalog(const std::vector<CollisionCatalogEntry>& catalog);
  std::vector<CollisionCatalogEntry> buildCollisionCatalog(urdf::Model& model, bool forPlanning);

  Obstacle obstacleFromCollisionObject(const std::string& frameID, const urdf::Collision& collisionObject,
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

  std::string createPlanningRobotDescription(const std::string& originalRobotDescription);
};

#endif // ROBOT_PARSER_HP
