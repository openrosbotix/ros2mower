#include "ros2mower_robot.hpp"

using namespace std::chrono_literals;

ROS2Mower_Robot::ROS2Mower_Robot(std::string name) : rclcpp::Node(name)
{

  this->declare_node_parameters();

  // register subscribers
  this->_sub_battery = this->create_subscription<sensor_msgs::msg::BatteryState>("/battery", 1000, std::bind(&ROS2Mower_Robot::callbackBattery, this, std::placeholders::_1));

  // register publisher
  this->_pub_mission = this->create_publisher<ros2mower_msgs::msg::Mission>("ros2mower/mission", 3);

  // register services
  this->_service_mission = this->create_service<ros2mower_msgs::srv::SetMission>("ros2mower/set_mission", std::bind(&ROS2Mower_Robot::set_mission, this, std::placeholders::_1, std::placeholders::_2));
  // set default values
  this->_actual_mission.mission = ROS2Mower_Robot::missions::idle;

  // define timer callback for publishing state
  this->_timer_publisher = this->create_wall_timer(
      100ms, std::bind(&ROS2Mower_Robot::timer_callback_publisher, this));

  // register service clients for map provider
}

ROS2Mower_Robot::~ROS2Mower_Robot() {}

void ROS2Mower_Robot::declare_node_parameters()
{
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
    // if (param.get_name() == "battery_low")
    // {
    //   this->_battery_voltage_low = param.as_double();
    // }
    // if (param.get_name() == "battery_critical")
    // {
    //   this->_battery_voltage_critical = param.as_double();
    // }
  }
  return result;
}

// TODO: switch to station state, if charge voltage is above given threshold
void ROS2Mower_Robot::callbackBattery(const std::shared_ptr<sensor_msgs::msg::BatteryState> msg)
{
  this->_actual_battery_voltage = msg->voltage;
  // if (this->_actual_battery_voltage < this->_battery_voltage_low)
  // {
  //   this->battery_low = true;
  //   this->set_mission_internal(ROS2Mower_Robot::missions::docking);
  // }
  // else
  // {
  //   this->battery_low = false;
  // }

  // if (this->_actual_battery_voltage < this->_battery_voltage_critical)
  // {
  //   this->battery_critical = true;
  //   this->set_mission_internal(ROS2Mower_Robot::missions::error);
  // }
  // else
  // {
  //   this->battery_critical = false;
  // }

  if (msg->current > 0.1)
  {
    this->isCharging = true;
    this->set_mission_internal(ROS2Mower_Robot::missions::station);
  }
  else
  {
    this->isCharging = false;
  }
}

void ROS2Mower_Robot::set_mission_internal(int mission)
{
  auto new_mission = ros2mower_msgs::msg::Mission();
  new_mission.mission = mission;
  this->_last_mission = this->_actual_mission;
  this->_actual_mission = new_mission;
}

void ROS2Mower_Robot::timer_callback_publisher()
{
  this->_pub_mission->publish(this->_actual_mission);
}

void ROS2Mower_Robot::set_mission(const std::shared_ptr<ros2mower_msgs::srv::SetMission::Request> request,
                                  std::shared_ptr<ros2mower_msgs::srv::SetMission::Response> response)
{

  if (request->new_mission.mission != this->_actual_mission.mission)
  {

    // check if mission value is valid
    if (request->new_mission.mission == ROS2Mower_Robot::missions::idle ||
        request->new_mission.mission == ROS2Mower_Robot::missions::area_recording ||
        request->new_mission.mission == ROS2Mower_Robot::missions::mow ||
        request->new_mission.mission == ROS2Mower_Robot::missions::single_mow ||
        request->new_mission.mission == ROS2Mower_Robot::missions::docking ||
        request->new_mission.mission == ROS2Mower_Robot::missions::station ||
        request->new_mission.mission == ROS2Mower_Robot::missions::error)
    {
      if (request->new_mission.mission == ROS2Mower_Robot::missions::single_mow &&
          request->new_mission.mission_data.data.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "ROS2Mower: invalid mission data, keep old mission");
      }
      else
      {
        this->_last_mission = this->_actual_mission;
        this->_actual_mission = request->new_mission;
        RCLCPP_INFO(this->get_logger(), "ROS2Mower: change mission to" + this->_actual_mission.mission);
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "ROS2Mower: invalid mission, keep old mission");
    }
  }

  response->actual_mission = this->_actual_mission;
}
