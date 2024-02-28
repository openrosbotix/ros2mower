#include "behaviors/CheckBatteryVoltage.hpp"

CheckBatteryVoltage::CheckBatteryVoltage(const std::string &name,
                                         const NodeConfig &conf,
                                         const RosNodeParams &params)
    : RosTopicSubNode<sensor_msgs::msg::BatteryState>(name, conf, params)
{
}

NodeStatus CheckBatteryVoltage::onTick(const std::shared_ptr<sensor_msgs::msg::BatteryState> &last_msg)
{
    if (last_msg) // empty if no new message received, since the last tick
    {
        this->last_msg_received = *last_msg;
    }

    // if we didn't receive a message so far, return success
    if (this->last_msg_received.voltage < 1.0)
    {
        return NodeStatus::SUCCESS;
    }

    float expected;
    if (getInput("expectedValue", expected))
    {
        if (this->last_msg_received.voltage >= expected)
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

    return NodeStatus::SUCCESS;
}
