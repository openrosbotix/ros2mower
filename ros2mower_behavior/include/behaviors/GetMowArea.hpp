#ifndef _GET_MOW_AREA_HPP
#define _GET_MOW_AREA_HPP

#include <behaviortree_ros2/bt_service_node.hpp>
#include "std_msgs/msg/string.hpp"
#include "ros2mower_msgs/srv/get_area_list.hpp"

using namespace BT;

class GetMowArea : public RosServiceNode<ros2mower_msgs::srv::GetAreaList>
{
public:
  GetMowArea(const std::string &name,
             const NodeConfig &conf,
             const RosNodeParams &params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<std::string>("actualMowArea"),
                               BT::OutputPort<std::string>("newMowArea")});
  }

  bool setRequest(Request::SharedPtr &request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr &response) override;
};

#endif