#include "behaviors/GetMowArea.hpp"

GetMowArea::GetMowArea(const std::string &name,
                       const NodeConfig &conf,
                       const RosNodeParams &params)
    : RosServiceNode<ros2mower_msgs::srv::GetAreaList>(name, conf, params)
{
}

bool GetMowArea::setRequest(Request::SharedPtr &request)
{
    return true;
}

NodeStatus GetMowArea::onResponseReceived(const Response::SharedPtr &response)
{
    std::string actualMowArea;
    getInput("actualMowArea", actualMowArea);

    if (response->count > 0)
    {
        // no actual mow area set? Start with the first one
        if (actualMowArea.empty())
        {
            setOutput("newMowArea", response->area_names[0].data);
            return NodeStatus::SUCCESS;
        }
        else
        {
            // find index of actual area
            for (uint i = 0; i < response->count; i++)
            {
                if (actualMowArea == response->area_names[i].data)
                {
                    // check if there exist another area
                    if (i + 1 < response->count)
                    {
                        // return next area
                        setOutput("newMowArea", response->area_names[i + 1].data);
                        return NodeStatus::SUCCESS;
                    }
                    else
                    {
                        // no more areas
                        setOutput("newMowArea", "");
                        return NodeStatus::FAILURE;
                    }
                }
            }
        }
    }
    else
    {
        setOutput("newMowArea", "");
        return NodeStatus::FAILURE;
    }
}
