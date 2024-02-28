#ifndef _MOW_AREA_HPP
#define _MOW_AREA_HPP

#include <behaviortree_ros2/bt_service_node.hpp>

using namespace BT;

class MowArea : public RosActionNode<ros2mower_msgs::srv::SetMission>
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