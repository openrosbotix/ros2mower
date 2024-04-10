#include "ros2mower_area_recording.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROS2Mower_AreaRecording>("ros2mower_area_recording");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
