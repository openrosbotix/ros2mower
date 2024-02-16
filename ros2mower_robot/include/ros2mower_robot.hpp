#ifndef _ROS2MOWER_ROBOT_H
#define _ROS2MOWER_ROBOT_H

#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ros2mower_msgs/msg/mission.hpp"
#include "ros2mower_msgs/srv/set_mission.hpp"

class ROS2Mower_Robot : public rclcpp::Node
{
public:
    /* pubic methods*/
    ROS2Mower_Robot(std::string name);
    ~ROS2Mower_Robot();

    /// @brief change mission of robot
    /// @param request new mission to set
    /// @return response actual mission after service call
    void set_mission(const std::shared_ptr<ros2mower_msgs::srv::SetMission::Request> request,
                     std::shared_ptr<ros2mower_msgs::srv::SetMission::Response> response);

    /// @brief parameter callback method
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    /* public types*/

    /// @brief available robot mission
    enum missions : uint8_t
    {
        idle = 0,
        area_recording = 10,
        mow = 20,
        single_mow = 21,
        docking = 30,
        station = 80,
        error = 99
    };

    /* public attributes*/

    /// @brief indicates if battery reaches low level
    bool battery_low;

    /// @brief indicator if battery reaches critical level
    bool battery_critical;

    /// @brief indicates if robot ic charging
    bool isCharging;

private:
    /// @brief what robot actually does
    ros2mower_msgs::msg::Mission _actual_mission;

    /// @brief what was the last mission of robot
    ros2mower_msgs::msg::Mission _last_mission;

    /// @brief actual battery level
    float _actual_battery_voltage;

    /// @brief battery voltage taken as low if below this value
    float _battery_voltage_low;

    /// @brief battery voltage taken as critical if below this value
    float _battery_voltage_critical;

    /// @brief Timer for publishing status messages like actual mission
    rclcpp::TimerBase::SharedPtr _timer_publisher;

    /// @brief Parameter Callback handle
    OnSetParametersCallbackHandle::SharedPtr _callbackParameter;

    /// @brief subscriber to battery topic
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr _sub_battery;

    /// @brief publisher of actual mission
    rclcpp::Publisher<ros2mower_msgs::msg::Mission>::SharedPtr _pub_mission;

    /// @brief Service server to set robot mission
    rclcpp::Service<ros2mower_msgs::srv::SetMission>::SharedPtr _service_mission;

    /// @brief define node parameters
    void declare_node_parameters();

    /// @brief callback method for battery messages
    void callbackBattery(const std::shared_ptr<sensor_msgs::msg::BatteryState> msg);

    /// @brief callback for publishing status messages
    void timer_callback_publisher();

    /// @brief set mission of robot. For internal use only
    /// @param mission new mission
    void set_mission_internal(int mission);
};

#endif