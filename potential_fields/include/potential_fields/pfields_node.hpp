#ifndef PFIELDS_NODE_HPP
#define PFIELDS_NODE_HPP

#include <rclcpp/rclcpp.hpp>

class PotentialFieldNode : public rclcpp::Node {
public:
  PotentialFieldNode();
  ~PotentialFieldNode() = default;

private:
  rclcpp::TimerBase::SharedPtr timer;
};

#endif // PFIELDS_NODE_HPP