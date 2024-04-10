#ifndef _GET_COVERAGE_PATH_HPP
#define _GET_COVERAGE_PATH_HPP

#include <behaviortree_ros2/bt_service_node.hpp>
#include "slic3r_coverage_planner/srv/plan_path.hpp"
#include "slic3r_coverage_planner/msg/path.hpp"
#include "ros2mower_msgs/msg/map_area.hpp"

using namespace BT;

class GetCoveragePath : public RosServiceNode<slic3r_coverage_planner::srv::PlanPath>
{
public:
  GetCoveragePath(const std::string &name,
                  const NodeConfig &conf,
                  const RosNodeParams &params);

  static BT::PortsList providedPorts()
  {
    // compiles w/o error
//        return providedBasicPorts({BT::InputPort<ros2mower_msgs::msg::MapArea>("mowAreaData"),
 //                              BT::OutputPort<slic3r_coverage_planner::srv::PlanPath::Response>("path")});

    return providedBasicPorts({BT::InputPort<ros2mower_msgs::msg::MapArea>("mowAreaData"),
                               BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("path")});
  }

  bool setRequest(Request::SharedPtr &request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr &response) override;
};

#endif