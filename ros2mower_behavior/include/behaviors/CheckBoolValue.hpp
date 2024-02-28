#ifndef _CHECK_BOOL_VALUE_HPP
#define _CHECK_BOOL_VALUE_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/bool.hpp>

using namespace BT;

class CheckBoolValue : public RosTopicSubNode<std_msgs::msg::Bool>
{
public:
  CheckBoolValue(const std::string &name,
                 const NodeConfig &conf,
                 const RosNodeParams &params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<bool>("expectedValue")});
  }

  NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool> &last_msg) override;

  private:
  std_msgs::msg::Bool last_msg_received;
};

#endif