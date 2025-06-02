#include "pfield_demo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

PFDemo::PFDemo() : Node("pfield_demo") {
  RCLCPP_INFO(this->get_logger(), "PFDemo Initialized");

  static tf2_ros::StaticTransformBroadcaster staticBroadcaster = tf2_ros::StaticTransformBroadcaster(this);
  // Publish a static transform at the origin of the world frame
  geometry_msgs::msg::TransformStamped worldTransform;
  worldTransform.header.stamp = this->now();
  worldTransform.header.frame_id = "world";
  worldTransform.child_frame_id = "map";
  worldTransform.transform.translation.x = 0.0;
  worldTransform.transform.translation.y = 0.0;
  worldTransform.transform.translation.z = 0.0;
  worldTransform.transform.rotation.x = 0.0;
  worldTransform.transform.rotation.y = 0.0;
  worldTransform.transform.rotation.z = 0.0;
  worldTransform.transform.rotation.w = 1.0;
  staticBroadcaster.sendTransform(worldTransform);

  // Publish a static transform from world to base_link
  geometry_msgs::msg::TransformStamped baseLinkTransform;
  baseLinkTransform.header.stamp = this->now();
  baseLinkTransform.header.frame_id = "world";
  baseLinkTransform.child_frame_id = "base_link";
  baseLinkTransform.transform.translation.x = 0.0;
  baseLinkTransform.transform.translation.y = 0.0;
  baseLinkTransform.transform.translation.z = 0.0;
  baseLinkTransform.transform.rotation.x = 0.0;
  baseLinkTransform.transform.rotation.y = 0.0;
  baseLinkTransform.transform.rotation.z = 0.0;
  baseLinkTransform.transform.rotation.w = 1.0;
  staticBroadcaster.sendTransform(baseLinkTransform);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PFDemo>());
  rclcpp::shutdown();
  return 0;
}