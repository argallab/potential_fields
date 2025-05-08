#include "rclcpp/rclcpp.hpp"
#include "pfield_manager.hpp"

PotentialFieldManager::PotentialFieldManager() : Node("potential_field_manager") {
  RCLCPP_INFO(this->get_logger(), "PotentialFieldManager Initialized");

  // Declare parameters
  this->timerFreq = this->declare_parameter("timer_frequency", 10.0f); // [Hz]
  this->attractiveGain = this->declare_parameter("attractive_gain", 1.0f); // [N]
  this->repulsiveGain = this->declare_parameter("repulsive_gain", 1.0f); // [N]
  this->max_force = this->declare_parameter("max_force", 10.0f); // [N]
  this->influence_radius_scale = this->declare_parameter("influence_radius_scale", 2.0f); // Scale for influence radius
  // Get parameters from yaml file
  this->timerFreq = this->get_parameter("timer_frequency").as_double();
  this->attractiveGain = this->get_parameter("attractive_gain").as_double();
  this->repulsiveGain = this->get_parameter("repulsive_gain").as_double();
  this->max_force = this->get_parameter("max_force").as_double();
  this->influence_radius_scale = this->get_parameter("influence_radius_scale").as_double();


  this->timer = this->create_wall_timer(
    std::chrono::seconds(static_cast<int>(1.0f / this->timerFreq)),
    std::bind(&PotentialFieldManager::timerCallback, this)
  );
}

void PotentialFieldManager::timerCallback() {
  RCLCPP_INFO(this->get_logger(), "Timer triggered");
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialFieldManager>());
  rclcpp::shutdown();
  return 0;
}