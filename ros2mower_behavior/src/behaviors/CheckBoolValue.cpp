#include "behaviors/CheckBoolValue.hpp"

CheckBoolValue::CheckBoolValue(const std::string &name,
                               const NodeConfig &conf,
                               const RosNodeParams &params)
    : RosTopicSubNode<std_msgs::msg::Bool>(name, conf, params)
{
}

NodeStatus CheckBoolValue::onTick(const std::shared_ptr<std_msgs::msg::Bool> &last_msg)
{
    if (last_msg) // empty if no new message received, since the last tick
    {
        this->last_msg_received = *last_msg;
    }
    // RCLCPP_INFO(logger(), "[%s]: %i", name().c_str(), last_msg->data);
    bool expected;
    if (getInput("expectedValue", expected))
    {
        if (this->last_msg_received.data == expected)
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
        return NodeStatus::FAILURE; // unable to read input port
    }
    return NodeStatus::FAILURE; // should never reach here
}
