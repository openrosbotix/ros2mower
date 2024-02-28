#ifndef _CHECK_BATTERY_VOLTAGE_HPP
#define _CHECK_BATTERY_VOLTAGE_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <sensor_msgs/msg/battery_state.hpp>

using namespace BT;

class CheckBatteryVoltage : public RosTopicSubNode<sensor_msgs::msg::BatteryState>
{
public:
  CheckBatteryVoltage(const std::string &name,
                      const NodeConfig &conf,
                      const RosNodeParams &params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<float>("expectedValue")});
  }

  NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::BatteryState> &last_msg) override;

private:
  sensor_msgs::msg::BatteryState last_msg_received;
};

#endif