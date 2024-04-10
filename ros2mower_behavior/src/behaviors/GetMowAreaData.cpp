#include "behaviors/GetMowAreaData.hpp"

GetMowAreaData::GetMowAreaData(const std::string &name,
                               const NodeConfig &conf,
                               const RosNodeParams &params)
    : RosServiceNode<ros2mower_msgs::srv::GetArea>(name, conf, params)
{
}

bool GetMowAreaData::setRequest(Request::SharedPtr &request)
{
    std::string mowArea;
    if (getInput("mowArea", mowArea))
    {
        RCLCPP_INFO(node_->get_logger(), "[%s]: fetch data for: %s", name().c_str(), mowArea);
        request->name.data = mowArea;
        return true;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s]: no mow area defined", name().c_str());
    return false;
}

NodeStatus GetMowAreaData::onResponseReceived(const Response::SharedPtr &response)
{
    setOutput("mowAreaData", response->area);
    return NodeStatus::SUCCESS;
}
