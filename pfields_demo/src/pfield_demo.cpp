#include "pfield_demo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

PFDemo::PFDemo() : Node("pfield_demo") {
  RCLCPP_INFO(this->get_logger(), "PFDemo Initialized");

  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  // Get parameters from yaml file
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();

  static tf2_ros::StaticTransformBroadcaster staticBroadcaster = tf2_ros::StaticTransformBroadcaster(this);

  // Publish a static transform from world to base_link
  geometry_msgs::msg::TransformStamped baseLinkTransform;
  baseLinkTransform.header.stamp = this->now();
  baseLinkTransform.header.frame_id = this->fixedFrame;
  baseLinkTransform.child_frame_id = "base_link";
  baseLinkTransform.transform.translation.x = 0.0;
  baseLinkTransform.transform.translation.y = 0.0;
  baseLinkTransform.transform.translation.z = 0.0;
  baseLinkTransform.transform.rotation.x = 0.0;
  baseLinkTransform.transform.rotation.y = 0.0;
  baseLinkTransform.transform.rotation.z = 0.0;
  baseLinkTransform.transform.rotation.w = 1.0;
  staticBroadcaster.sendTransform(baseLinkTransform);

  // Wait for the service to be available
  this->planPathClient = this->create_client<PlanPath>("pfield/plan_path");
  while (!this->planPathClient->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for the plan_path service to be available...");
  }
  RCLCPP_INFO(this->get_logger(), "Plan path service is available.");
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PFDemo>());
  rclcpp::shutdown();
  return 0;
}