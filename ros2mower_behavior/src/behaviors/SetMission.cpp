#include "behaviors/SetMission.hpp"

SetMission::SetMission(const std::string &name,
                       const NodeConfig &conf,
                       const RosNodeParams &params)
    : RosServiceNode<ros2mower_msgs::srv::SetMission>(name, conf, params)
{
}

bool SetMission::setRequest(Request::SharedPtr &request)
{
    int new_mission;
    if (getInput("newMissionID", new_mission))
    {
        RCLCPP_INFO(node_->get_logger(), "[%s]: Try to change mission to %i", name().c_str(), new_mission);
        request->new_mission.mission = new_mission;
        return true;
    }
    return false;
}

NodeStatus SetMission::onResponseReceived(const Response::SharedPtr &response)
{
    int new_missionID;
    getInput("newMissionID", new_missionID);

    if (response->actual_mission.mission == new_missionID)
    {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}
