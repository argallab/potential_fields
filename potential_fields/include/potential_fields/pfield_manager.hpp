#ifndef PFIELD_MANAGER_HPP
#define PFIELD_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>

class PotentialFieldManager : public rclcpp::Node {
public:
  PotentialFieldManager();
  ~PotentialFieldManager() = default;

private:
  float timerFreq; // Timer frequency [Hz]
  float attractiveGain; // Attractive gain [N]
  float repulsiveGain; // Repulsive gain [N]
  float max_force; // Maximum force [N]
  float influence_radius_scale; // Scale for influence radius

  rclcpp::TimerBase::SharedPtr timer;

  void timerCallback();
};

#endif // PFIELD_MANAGER_HPP