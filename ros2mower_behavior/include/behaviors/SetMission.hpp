#ifndef _SET_MISSION_HPP
#define _SET_MISSION_HPP

#include <behaviortree_ros2/bt_service_node.hpp>
#include "ros2mower_msgs/msg/mission.hpp"
#include "ros2mower_msgs/srv/set_mission.hpp"

using namespace BT;

class SetMission : public RosServiceNode<ros2mower_msgs::srv::SetMission>
{
public:
  SetMission(const std::string &name,
             const NodeConfig &conf,
             const RosNodeParams &params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<int>("newMissionID")});
  }

  bool setRequest(Request::SharedPtr &request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr &response) override;
};

#endif