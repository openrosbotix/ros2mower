#ifndef _GET_MOW_AREA_DATA_HPP
#define _GET_MOW_AREA_DATA_HPP

#include <behaviortree_ros2/bt_service_node.hpp>
#include "ros2mower_msgs/srv/get_area.hpp"
#include "ros2mower_msgs/msg/map_area.hpp"

using namespace BT;

class GetMowAreaData : public RosServiceNode<ros2mower_msgs::srv::GetArea>
{
public:
  GetMowAreaData(const std::string &name,
                  const NodeConfig &conf,
                  const RosNodeParams &params);

  static BT::PortsList providedPorts()
  {
    // compiles w/o error
    //return providedBasicPorts({BT::InputPort<std::string>("mowArea"),
    //                           BT::OutputPort<std::string>("path")});
    return providedBasicPorts({BT::InputPort<std::string>("mowArea"),
                               BT::OutputPort<ros2mower_msgs::srv::GetArea::Response>("mowAreaData")});
  }

  bool setRequest(Request::SharedPtr &request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr &response) override;
};

#endif