#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

class ROS2Mower_Simulation : public rclcpp::Node
{
public:
  ROS2Mower_Simulation(std::string name) : Node(name)
  {
    this->declare_parameter("battery_voltage", 40.0);
    this->declare_parameter("battery_current", -1.0);
    this->declare_parameter("isLocalized", true);

    battery_voltage = this->get_parameter("battery_voltage").as_double();
    battery_current = this->get_parameter("battery_current").as_double();
    isLocalized = this->get_parameter("isLocalized").as_bool();

    // create publisher
    pub_battery = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery", 10);
    pub_localized = this->create_publisher<std_msgs::msg::Bool>("ros2mower/isLocalized", 3);

    // Create a timer to tick the behavior tree.
    const auto timer_period = 100ms;
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&ROS2Mower_Simulation::publish, this));

    _callbackParameter = this->add_on_set_parameters_callback(
        std::bind(&ROS2Mower_Simulation::parametersCallback, this, std::placeholders::_1));
  }

private:
  void publish()
  {
    this->publishBattery();
    this->publishLocalized();
  }

  void publishBattery()
  {
    auto batteryMsg = sensor_msgs::msg::BatteryState();
    batteryMsg.voltage = this->battery_voltage;
    batteryMsg.current = this->battery_current;
    this->pub_battery->publish(batteryMsg);
  }

  void publishLocalized()
  {
    auto localized = std_msgs::msg::Bool();
    localized.data = this->isLocalized;
    this->pub_localized->publish(localized);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    for (const auto &param : parameters)
    { // TODO: format reconfigure nicely
      if (param.get_name() == "battery_voltage")
      {
        this->battery_voltage = param.as_double();
      }
      if (param.get_name() == "battery_current")
      {
        this->battery_current = param.as_double();
      }
      if (param.get_name() == "isLocalized")
      {
        this->isLocalized = param.as_bool();
      }
    }
    return result;
  }

  float battery_voltage;
  float battery_current;
  bool isLocalized;

  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr _callbackParameter;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_localized;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ROS2Mower_Simulation>("ros2mower_Simulation");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
