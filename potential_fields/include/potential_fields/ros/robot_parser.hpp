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

using Obstacle = potential_fields_interfaces::msg::Obstacle;
using ObstacleArray = potential_fields_interfaces::msg::ObstacleArray;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class RobotParser : public rclcpp::Node {
public:
  RobotParser();
  ~RobotParser() = default;

private:
  double robotUpdateFrequency; // Frequency for updating Robot's TF frames [Hz]
  std::string robotDescription; // Robot Description
  std::string fixedFrame; // RViz fixed frame
  urdf::Model robotModel; // Parsed URDF model
  bool modelLoaded = false; // Whether URDF parsed successfully

  struct CollisionEntry {
    std::string id;         // unique obstacle id (link::name or link::colN)
    std::string link_name;  // link this collision belongs to
    urdf::CollisionSharedPtr col; // collision element
  };
  std::vector<CollisionEntry> collisions; // pre-collected collisions


  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstaclePub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamicTfBroadcaster;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;

  void timerCallback();

  std::vector<Obstacle> extractObstaclesFromCatalog();
  void buildCollisionCatalog();

  Obstacle obstacleFromCollisionObject(const std::string& frameID, const urdf::Collision& collisionObject,
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
};

#endif // ROBOT_PARSER_HP
