#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviors/CheckBoolValue.hpp"
#include "behaviors/CheckBatteryVoltage.hpp"
#include "behaviors/CheckMission.hpp"
#include "behaviors/SetMission.hpp"
#include "behaviors/GetMowArea.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("ros2mower_behavior");

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("ros2mower_behavior");

  std::string tree_xml = package_share_directory + "/config/ros2mower_main_tree.xml";

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;
  factory.registerNodeType<CheckBoolValue>("CheckBoolValue", params);
  factory.registerNodeType<CheckBatteryVoltage>("CheckBatteryVoltage", params);
  factory.registerNodeType<CheckMission>("CheckMission", params);
  factory.registerNodeType<SetMission>("SetMission", params);
  factory.registerNodeType<GetMowArea>("GetMowArea", params);

  auto tree = factory.createTreeFromFile(tree_xml);
  BT::Groot2Publisher publisher(tree);
  while (rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}
