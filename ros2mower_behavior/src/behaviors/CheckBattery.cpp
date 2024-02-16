#include "behaviors/CheckBattery.hpp"


CheckBattery::CheckBattery(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config)
{
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

// CheckBattery::CheckBattery(const std::string &name, const BT::NodeConfig &config,
//                            std::shared_ptr<ROS2Mower_Robot> robot_ptr) : BT::SyncActionNode(name, config), robot_ptr_{robot_ptr}
// {
//     std::cout << "[" << this->name() << "] Initialized" << std::endl;
// }

BT::NodeStatus CheckBattery::tick()
{
  //  if (robot_ptr_->battery_critical || robot_ptr_->battery_low)
    {
        std::cout << "[" << this->name() << "] Battery low" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
std::cout << "[" << this->name() << "] Battery OK" << std::endl;
    return BT::NodeStatus::SUCCESS;
}
