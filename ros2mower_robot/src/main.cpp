#include "ros2mower_robot.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROS2Mower_Robot>("ros2mower_robot");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
