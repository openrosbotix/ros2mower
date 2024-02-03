#ifndef _ROS2MOWER_ROBOT_H
#define _ROS2MOWER_ROBOT_H

#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include <stdio.h>
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class ROS2Mower_Robot : public rclcpp::Node
{
public:
    /* pubis methods*/
    ROS2Mower_Robot(std::string name);
    ~ROS2Mower_Robot();

    /// @brief get the task the robot is actually processing
    int get_task();

    /// @brief change task of robot
    /// @param task new task
    /// @return id of new task
    int set_task(int task);

    /// @brief returns if robot is properly localized. I.e. if gps signal is valid
    bool isLocalized();

    /// @brief parameter callback method
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    /* public types*/

    /// @brief available robot tasks
    enum tasks : int
    {
        idle = 0,
        area_recording = 10,
        mow = 20,
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
    int _actual_task;

    /// @brief what was the last task of robot
    int _last_task;

    /// @brief indicates valid robot localization
    bool _isLocalized;

    /// @brief actual battery level
    float _actual_battery_voltage;

    /// @brief battery voltage taken as low if below this value
    float _battery_voltage_low;

    /// @brief battery voltage taken as critical if below this value
    float _battery_voltage_critical;

    /// @brief Parameter Callback handle
    OnSetParametersCallbackHandle::SharedPtr _callbackParameter;

    /// @brief subscriber to battery topic
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr _sub_battery;

    /// @brief subscriber to localize status
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_Localized;

    /// @brief define node parameters
    void declare_node_parameters();

    /// @brief callback method for battery messages
    void callbackBattery(const std::shared_ptr<sensor_msgs::msg::BatteryState> msg);

    /// @brief callback for valid localized messages
    void callbackLocalized(const std::shared_ptr<std_msgs::msg::Bool> msg);
};

#endif