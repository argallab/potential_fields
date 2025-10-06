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
#include "tf2_ros/static_transform_broadcaster.h"
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
  explicit RobotParser(bool testConstructor) : Node("robot_parser_test"), modelLoaded(false) {}

private:
  friend class RobotParserTestHelper; // Grants unit tests controlled access to private members
  double robotUpdateFrequency; // Frequency for updating Robot's TF frames [Hz]
  std::string robotDescription; // Robot Description
  std::string fixedFrame; // RViz fixed frame
  std::string planningTFPrefix; // Prefix applied to duplicated planning TF frames (e.g. "planning::")
  urdf::Model robotModel; // Parsed URDF model
  urdf::Model planningRobotModel; // Parsed URDF model with modified names for planning
  bool modelLoaded = false; // Whether URDF parsed successfully
  bool useJSPGui; // Parameter for GUI vs non-GUI Joint State Publisher
  std::vector<CollisionCatalogEntry> collisionCatalog; // Catalog of collision objects from URDF (Real Robot)
  std::vector<CollisionCatalogEntry> planningCollisionCatalog; // Catalog of collision objects from URDF (Planning Robot)

  rclcpp::TimerBase::SharedPtr timer; // Timer for updating Robot and Planning TFs
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstaclePub; // Publisher for Obstacles from Robot Collision Geometry
  rclcpp::Publisher<ObstacleArray>::SharedPtr planningObstaclePub; // Publisher for Planning Obstacles
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planningRobotDescriptionPub; // Publishes planning robot_description
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamicTfBroadcaster; // Broadcaster for dynamic TFs from Robot
  std::shared_ptr<tf2_ros::Buffer> tfBuffer; // TF buffer for listening to Robot TFs
  std::shared_ptr<tf2_ros::TransformListener> tfListener; // TF listener for Robot TFs to populate buffer
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster; // Broadcaster for static TFs from Robot

  void timerCallback();

  /**
   * @brief Given a catalog of collision objects, convert to a list of Obstacle msgs
   *
   * @param catalog Catalog of collision objects containing link names, collision objects, etc.
   * @return std::vector<Obstacle> The extracted obstacles, ready to publish
   */
  std::vector<Obstacle> extractObstaclesFromCatalog(const std::vector<CollisionCatalogEntry>& catalog);

  /**
   * @brief Given a URDF model, build a catalog of cached collision objects for quick access later
   *
   * @param model The URDF model to extract collision objects from
   * @param forPlanning Whether to build the catalog for planning purposes (adds planning prefix if so)
   * @return std::vector<CollisionCatalogEntry> The collision catalog which only needs to be built once
   */
  std::vector<CollisionCatalogEntry> buildCollisionCatalog(urdf::Model& model, bool forPlanning);

  /**
   * @brief From a collision object and extra data, create an Obstacle message for publishing
   *
   * @param frameID The frameID the obstacle belongs to (also the frame for the pose)
   * @param collisionObject The collision object to convert
   * @param position The position of the obstacle
   * @param orientation The orientation of the obstacle
   * @return Obstacle The created Obstacle message
   */
  Obstacle obstacleFromCollisionObject(const std::string& frameID, const urdf::Collision& collisionObject,
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

  /**
   * @brief Given an original robot description, create a modified version for planning with
   *        the appropriate TF prefixes and modified joint names.
   *
   * @note An example planningRobotDescription can be found in the data/ directory
   *
   * @param originalRobotDescription The original robot_description XML string
   * @return std::string The modified robot_description XML string for planning
   */
  std::string createPlanningRobotDescription(const std::string& originalRobotDescription);

  // Helper: write text content to a file (logs success/failure)
  void writeTextFile(const std::string& path, const std::string& contents, const std::string& label);
  // Helper: dump both original & planning robot descriptions for inspection
  void dumpRobotDescriptions(const std::string& original, const std::string& planning,
    const std::string& originalPath, const std::string& planningPath);
};

#endif // ROBOT_PARSER_HP
