#include "behaviors/GetCoveragePath.hpp"

GetCoveragePath::GetCoveragePath(const std::string &name,
                                 const NodeConfig &conf,
                                 const RosNodeParams &params)
    : RosServiceNode<slic3r_coverage_planner::srv::PlanPath>(name, conf, params)
{
}

bool GetCoveragePath::setRequest(Request::SharedPtr &request)
{
    auto mowArea = ros2mower_msgs::msg::MapArea();
    if (getInput("mowAreaData", mowArea))
    {
        RCLCPP_INFO(node_->get_logger(), "[%s]: got area: %s", name().c_str(), mowArea.name.data.c_str());

        // build request for slic3r coverage planner
        request->angle = 20.0;
        request->outline_count = 2;
        request->distance = 0.2;
        request->fill_type = 0;
        request->outline = mowArea.outer_polygon;
        request->holes = mowArea.keepout_zones;
        return true;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s]: no mow area defined", name().c_str());
    return false;
}

NodeStatus GetCoveragePath::onResponseReceived(const Response::SharedPtr &response)
{
    std::vector<geometry_msgs::msg::PoseStamped> result_path;

    for (auto &path : response->paths)
    {
        for (auto &pose : path.path.poses)
        {
            result_path.push_back(pose);
        }
    }
    setOutput("path", result_path);

    // if (response->actual_mission.mission == new_missionID)
    //{
    return NodeStatus::SUCCESS;
    // }
    // return NodeStatus::FAILURE;
}
