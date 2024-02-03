#include "ros2mower_robot.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto controller = std::make_shared<ROS2Mower_Robot>("ros2mower_robot");

    rclcpp::Rate rate(20.0);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(controller);
        rate.sleep();
    }

    return 0;
}
