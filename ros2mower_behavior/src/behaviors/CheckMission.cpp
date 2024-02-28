#include "behaviors/CheckMission.hpp"

CheckMission::CheckMission(const std::string &name,
                           const NodeConfig &conf,
                           const RosNodeParams &params)
    : RosTopicSubNode<ros2mower_msgs::msg::Mission>(name, conf, params)
{
}

NodeStatus CheckMission::onTick(const std::shared_ptr<ros2mower_msgs::msg::Mission> &last_msg)
{
    if (last_msg) // empty if no new message received, since the last tick
    {
        // store new value for further checks
        this->last_msg_received = *last_msg;
    }

    int expected;
    setOutput("actualMissionID", this->last_msg_received.mission);
    
    if (getInput("expectedMissionID", expected))
    {
        if (this->last_msg_received.mission == expected)
        {
            return NodeStatus::SUCCESS; // meets expected value
        }
        else
        {
            return NodeStatus::FAILURE; // doesn't meet expected value
        }
    }
    else
    {
        setOutput("actualMissionID", this->last_msg_received.mission);
        return NodeStatus::FAILURE; // unable to read input port
    }

    RCLCPP_INFO(logger(), "[%s]: no mission received", name().c_str());
    return NodeStatus::FAILURE;
}
