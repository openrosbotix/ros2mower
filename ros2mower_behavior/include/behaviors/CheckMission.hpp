#ifndef _CHECK_MISSION_HPP
#define _CHECK_MISSION_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "ros2mower_msgs/msg/mission.hpp"

using namespace BT;

class CheckMission : public RosTopicSubNode<ros2mower_msgs::msg::Mission>
{
public:
  CheckMission(const std::string &name,
                 const NodeConfig &conf,
                 const RosNodeParams &params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<int>("expectedMissionID"),
                               BT::OutputPort<int>("actualMissionID")});
  }

  NodeStatus onTick(const std::shared_ptr<ros2mower_msgs::msg::Mission> &last_msg) override;

  private:
  ros2mower_msgs::msg::Mission last_msg_received;
};

#endif