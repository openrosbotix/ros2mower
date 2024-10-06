#ifndef _ROS2MOWER_AREA_REC_H
#define _ROS2MOWER_AREA_REC_H

#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/header.hpp"
#include "ros2mower_msgs/srv/set_area.hpp"
#include "ros2mower_msgs/srv/save_map.hpp"
#include "ros2mower_msgs/msg/map_area.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ROS2Mower_AreaRecording : public rclcpp::Node
{
public:
    /* pubic methods*/
    ROS2Mower_AreaRecording(std::string name);
    ~ROS2Mower_AreaRecording();

    /// @brief parameter callback method
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    /* public types*/

    /* public attributes*/

private:

    /// @brief  store recorded polygons here
    ros2mower_msgs::msg::MapArea _map_area;

    /// @brief last robot pose in map frame
    geometry_msgs::msg::PoseStamped _last_pose_map;

    /// @brief actual robot pose in map frame
    geometry_msgs::msg::PoseStamped _actual_pose_map;

    /// @brief robot pose only used for map transforms
    geometry_msgs::msg::PoseStamped _robot_pose;

    /// @brief last recorded polygon
    geometry_msgs::msg::Polygon _polygon;

    /// @brief is polygon recording currently active
    bool _polygon_recording;

    /// @brief define joystick buttons by parameter
    int _joy_btn_area;
    int _joy_btn_keepout;
    int _joy_btn_toggle_poly;
    int _joy_btn_clear;
    int _joy_btn_clear_all;
    int _joy_btn_save;
    float _distance_points;
    std::string _base_frame;
    std::string _map_frame;

    /// @brief publish polygon ar marker array
    bool _doPublishPolygon;

    /// @brief Parameter Callback handle
    OnSetParametersCallbackHandle::SharedPtr _callbackParameter;

    /// @brief Service client to store an area
    rclcpp::Client<ros2mower_msgs::srv::SetArea>::SharedPtr _srv_set_area;

    /// @brief Service client to save map
    rclcpp::Client<ros2mower_msgs::srv::SaveMap>::SharedPtr _srv_save_map;

    /// @brief publishers to visualize polygon and mow area in rviz2
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_polygon;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_mow_area;

    /// @brief subscriber for joystick messages
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _sub_joy;

    /// @brief Timer for publishing status messages like actual mission
    rclcpp::TimerBase::SharedPtr _timer_publisher;

    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};

    /// @brief define node parameters
    void declare_node_parameters();

    /// @brief callback method for joystick messages
    void callbackJoystick(const std::shared_ptr<sensor_msgs::msg::Joy> msg);

    /// @brief callback for publishing status messages
    void timer_callback_publisher();

    /// @brief get current pose related to map
    /// @param global_frame
    /// @param robot_frame
    /// @param transform_timeout
    /// @return
    bool getCurrentPose(
        const std::string global_frame,
        const std::string robot_frame, const double transform_timeout);

    /// @brief calculate distance between poses
    /// @return distance in meters
    float getDistanceToLastPose();

};

#endif