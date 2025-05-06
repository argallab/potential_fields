#ifndef PFIELD_MANAGER_HPP
#define PFIELD_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>

class PotentialFieldManager : public rclcpp::Node {
public:
  PotentialFieldManager();
  ~PotentialFieldManager() = default;

private:
  rclcpp::TimerBase::SharedPtr timer;

  void timerCallback();
};

#endif // PFIELD_MANAGER_HPP