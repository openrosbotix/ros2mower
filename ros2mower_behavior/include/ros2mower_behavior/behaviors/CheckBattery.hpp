#ifndef CHECKBATTERY_H
#define CHECKBATTERY_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/behavior_tree.h"
// #include "ros2mower_robot.hpp"
class ROS2Mower_Robot; // forward declaration to get rid of circular dependencies

// Check battery level
class CheckBattery : public BT::SyncActionNode
{
public:
  CheckBattery(const std::string &name, const BT::NodeConfig &config);
  BT::NodeStatus tick() override;

  std::shared_ptr<ROS2Mower_Robot> robot_ptr_;
// necessary to declare, even it no ports are provided
  static BT::PortsList providedPorts() { return {}; };

private:
  // std::deque<std::string> location_queue_;
};
#endif