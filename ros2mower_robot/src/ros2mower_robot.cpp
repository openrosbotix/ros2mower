#include "ros2mower_robot.hpp"

ROS2Mower_Robot::ROS2Mower_Robot(std::string name) : Node(name)
{
  this->declare_node_parameters();

  // register subscribers
  this->_sub_battery = this->create_subscription<sensor_msgs::msg::BatteryState>("/battery", 1000, std::bind(&ROS2Mower_Robot::callbackBattery, this, std::placeholders::_1));
  this->_sub_Localized = this->create_subscription<std_msgs::msg::Bool>("/localization_valid", 1000, std::bind(&ROS2Mower_Robot::callbackLocalized, this, std::placeholders::_1));
}

ROS2Mower_Robot::~ROS2Mower_Robot() {}

void ROS2Mower_Robot::declare_node_parameters()
{
  // declare parameter
  this->declare_parameter("battery_low", 37.5);
  this->declare_parameter("battery_critical", 36.5);

  // register parameter change callback handle
  this->_callbackParameter = this->add_on_set_parameters_callback(
      std::bind(&ROS2Mower_Robot::parametersCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ROS2Mower_Robot::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  // Here update class attributes, do some actions, etc.
  for (const auto &param : parameters)
  { // TODO: format reconfigure nicely
    if (param.get_name() == "battery_low")
    {
      this->_battery_voltage_low = param.as_double();
    }
    if (param.get_name() == "battery_critical")
    {
      this->_battery_voltage_critical = param.as_double();
    }
  }
}

void ROS2Mower_Robot::callbackBattery(const std::shared_ptr<sensor_msgs::msg::BatteryState> msg)
{
  this->_actual_battery_voltage = msg->voltage;
  if (this->_actual_battery_voltage < this->_battery_voltage_low)
  {
    this->battery_low = true;
  }
  else
  {
    this->battery_low = false;
  }

  if (this->_actual_battery_voltage < this->_battery_voltage_critical)
  {
    this->battery_critical = true;
  }
  else
  {
    this->battery_critical = false;
  }

  if (msg->current > 0)
  {
    this->isCharging = true;
  }
  else
  {
    this->isCharging = false;
  }
}

void ROS2Mower_Robot::callbackLocalized(const std::shared_ptr<std_msgs::msg::Bool> msg)
{
  this->_isLocalized = msg->data;
}

bool ROS2Mower_Robot::isLocalized()
{
  return this->_isLocalized;
}

int ROS2Mower_Robot::get_task()
{
  return this->_actual_task;
}

int ROS2Mower_Robot::set_task(int task){
  this->_last_task = this->_actual_task;
  this->_actual_task = task;
  return this->_actual_task;

}

