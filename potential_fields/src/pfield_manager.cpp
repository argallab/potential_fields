#include "rclcpp/rclcpp.hpp"
#include "pfield_manager.hpp"

PotentialFieldManager::PotentialFieldManager() : Node("potential_field_manager") {
  RCLCPP_INFO(this->get_logger(), "PotentialFieldManager Initialized");

  this->timer = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PotentialFieldManager::timerCallback, this)
  );
}

void PotentialFieldManager::timerCallback() {
  RCLCPP_INFO(this->get_logger(), "Timer triggered");
}