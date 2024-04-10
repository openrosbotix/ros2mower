#include "ros2mower_area_recording.hpp"

using namespace std::chrono_literals;

ROS2Mower_AreaRecording::ROS2Mower_AreaRecording(std::string name) : rclcpp::Node(name)
{

  this->declare_node_parameters();

  // register services
  _srv_set_area = this->create_client<ros2mower_msgs::srv::SetArea>("ros2mower/set_area");
  _srv_save_map = this->create_client<ros2mower_msgs::srv::SaveMap>("ros2mower/save_map");
}

ROS2Mower_AreaRecording::~ROS2Mower_AreaRecording() {}

void ROS2Mower_AreaRecording::declare_node_parameters()
{

  declare_parameter("joy_toggle_polygon", 0);
  declare_parameter("joy_keepout", 1);
  declare_parameter("joy_area", 2);
  declare_parameter("joy_clear", 3);
  declare_parameter("joy_clear_all", 4);
  declare_parameter("joy_save", 5);
  declare_parameter("distance_points", 0.1);
  declare_parameter("publish_polygon", true);

  get_parameter("joy_toggle_polygon", this->_joy_btn_toggle_poly);
  get_parameter("joy_keepout", this->_joy_btn_keepout);
  get_parameter("joy_area", this->_joy_btn_area);
  get_parameter("joy_clear", this->_joy_btn_clear);
  get_parameter("joy_clear_all", this->_joy_btn_clear_all);
  get_parameter("joy_save", this->_joy_btn_save);
  get_parameter("distance_points", this->_distance_points);
  get_parameter("pubish_polygon", this->_doPublishPolygon);

  // register parameter change callback handle
  this->_callbackParameter = this->add_on_set_parameters_callback(
      std::bind(&ROS2Mower_AreaRecording::parametersCallback, this, std::placeholders::_1));

  this->_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->_tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  // register publisher
  this->_pub_mow_area = this->create_publisher<visualization_msgs::msg::MarkerArray>("ros2mower_area_rec/mow_area", 3);
  this->_pub_polygon = this->create_publisher<visualization_msgs::msg::Marker>("ros2mower_area_rec/polygon", 3);

  // define timer callback for publishing state
  this->_timer_publisher = this->create_wall_timer(
      100ms, std::bind(&ROS2Mower_AreaRecording::timer_callback_publisher, this));
}

rcl_interfaces::msg::SetParametersResult ROS2Mower_AreaRecording::parametersCallback(
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

void ROS2Mower_AreaRecording::callbackJoystick(const std::shared_ptr<sensor_msgs::msg::Joy> msg)
{
  // toggle polygon recording on/off
  if (msg->buttons[this->_joy_btn_toggle_poly] == 1)
  {
    // get actual pose from map when activating area recording
    if (this->getCurrentPose("map", "base_frame", 1.0) && this->_polygon_recording)
    {
      this->_polygon_recording = true;
      RCLCPP_INFO(this->get_logger(), "Area recording: toggle polygon recording to status %i", this->_polygon_recording);

      // store actual pose as last pose
      this->_last_pose_map = this->_actual_pose_map;
    }
    else
    {
      this->_polygon_recording = false;
      RCLCPP_INFO(this->get_logger(), "Area recording: toggle polygon recording to status %i", this->_polygon_recording);
    }
  }

  // save last polygon as outer polygon
  if (msg->buttons[this->_joy_btn_area] == 1)
  {
    this->_map_area.outer_polygon = this->_polygon;
  }

  // save last polygon as keepout zone
  if (msg->buttons[this->_joy_btn_keepout] == 1)
  {
    this->_map_area.keepout_zones.push_back(this->_polygon);
  }

  // save map as area
  if (msg->buttons[this->_joy_btn_save] == 1)
  {
    auto request_save_map = std::make_shared<ros2mower_msgs::srv::SaveMap::Request>();
    auto result = this->_srv_save_map->async_send_request(request_save_map);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "save ok: %i", result.get()->success);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service save map of map provider");
    }
  }

  // save new point to polygon
  if (this->_polygon_recording)
  {
    // get actual pose related to map
    if (this->getCurrentPose("map", "base_frame", 1.0))
    {
      // calculate distance between last pose and actual pose
      if (this->getDistanceToLastPose() >= this->_distance_points)
      {
        // add point to polygon
        auto new_point = geometry_msgs::msg::Point32();
        new_point.x = this->_actual_pose_map.pose.position.x;
        new_point.y = this->_actual_pose_map.pose.position.y;

        this->_polygon.points.push_back(new_point);
      }
    }
  }
}

bool ROS2Mower_AreaRecording::getCurrentPose(
    const std::string global_frame,
    const std::string robot_frame, const double transform_timeout)
{
  static rclcpp::Logger logger = this->get_logger();

  tf2::toMsg(tf2::Transform::getIdentity(), this->_last_pose_map.pose);
  tf2::toMsg(tf2::Transform::getIdentity(), this->_robot_pose.pose);
  this->_robot_pose.header.frame_id = robot_frame;
  this->_robot_pose.header.stamp = rclcpp::Time();

  try
  {
    // this->_actual_pose_map
    auto transformStamped = this->_tf_buffer->lookupTransform(
        robot_frame, global_frame,
        tf2::TimePointZero, tf2::durationFromSec(transform_timeout));

    this->_actual_pose_map.pose.position.x = transformStamped.transform.translation.x;
    this->_actual_pose_map.pose.position.y = transformStamped.transform.translation.y;
    this->_actual_pose_map.pose.position.z = transformStamped.transform.translation.z;
    this->_actual_pose_map.pose.orientation = transformStamped.transform.rotation;
    return true;
  }
  catch (tf2::LookupException &ex)
  {
    RCLCPP_ERROR(
        logger,
        "No Transform available Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::ConnectivityException &ex)
  {
    RCLCPP_ERROR(
        logger,
        "Connectivity Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::ExtrapolationException &ex)
  {
    RCLCPP_ERROR(
        logger,
        "Extrapolation Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::TimeoutException &ex)
  {
    RCLCPP_ERROR(
        logger,
        "Transform timeout with tolerance: %.4f", transform_timeout);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(
        logger, "Failed to transform from %s to %s",
        global_frame.c_str(), robot_frame.c_str());
  }

  return false;
}

float ROS2Mower_AreaRecording::getDistanceToLastPose()
{
  tf2::Vector3 last_point(this->_last_pose_map.pose.position.x, this->_last_pose_map.pose.position.y, 0.0);
  tf2::Vector3 current_point(this->_actual_pose_map.pose.position.x, this->_actual_pose_map.pose.position.y, 0.0);

  return (current_point - last_point).length();
}

void ROS2Mower_AreaRecording::timer_callback_publisher()
{
  if (this->_doPublishPolygon)
  {
    auto polygon_marker = new visualization_msgs::msg::Marker();
    auto polygon_header = new std_msgs::msg::Header();
    for (auto poly_point : this->_polygon.points)
    {
      auto marker_point = new geometry_msgs::msg::Point();
      marker_point->x = poly_point.x;
      marker_point->y = poly_point.y;
      marker_point->z = poly_point.z;
      polygon_marker->points.push_back(*marker_point);
    }
    polygon_header->frame_id = "map";
    polygon_marker->header = *polygon_header;
    polygon_marker->ns = "Polygon";
    polygon_marker->type = visualization_msgs::msg::Marker::LINE_STRIP; // line
    polygon_marker->action = visualization_msgs::msg::Marker::ADD;
    polygon_marker->lifetime = rclcpp::Duration(100ms); // delete after 100ms
    polygon_marker->color.a = 1.0;                       // Don't forget to set the alpha!
    polygon_marker->color.r = 0.0;
    polygon_marker->color.g = 0.0;
    polygon_marker->color.b = 1.0;

    // create marker array containing entire Map data
    auto map_marker = new visualization_msgs::msg::MarkerArray();
    auto outline_marker = new visualization_msgs::msg::Marker();

    // create marker for outer polygon
    for (auto outline_point : this->_map_area.outer_polygon.points)
    {
      auto marker_point = new geometry_msgs::msg::Point();
      marker_point->x = outline_point.x;
      marker_point->y = outline_point.y;
      marker_point->z = outline_point.z;
      outline_marker->points.push_back(*marker_point);
    }

    // polygon_header->frame_id = "map";
    outline_marker->header = *polygon_header;
    outline_marker->ns = "Polygon";
    outline_marker->type = visualization_msgs::msg::Marker::LINE_STRIP; // line
    outline_marker->action = visualization_msgs::msg::Marker::ADD;
    outline_marker->lifetime = rclcpp::Duration(100ms); // delete after 100ms
    outline_marker->color.a = 1.0;                       // Don't forget to set the alpha!
    outline_marker->color.r = 0.0;
    outline_marker->color.g = 1.0;
    outline_marker->color.b = 0.0;

    // create marker for keepout zones
    for (auto keepout_zone : this->_map_area.keepout_zones)
    {
      auto keepout_marker = new visualization_msgs::msg::Marker();

      // create marker for single keepout zone
      for (auto keepout_point : keepout_zone.points)
      {
        auto marker_point = new geometry_msgs::msg::Point();
        marker_point->x = keepout_point.x;
        marker_point->y = keepout_point.y;
        marker_point->z = keepout_point.z;
        keepout_marker->points.push_back(*marker_point);
      }

      // polygon_header->frame_id = "map";
      keepout_marker->header = *polygon_header;
      keepout_marker->ns = "Polygon";
      keepout_marker->type = visualization_msgs::msg::Marker::LINE_STRIP; // line
      keepout_marker->action = visualization_msgs::msg::Marker::ADD;
      keepout_marker->lifetime = rclcpp::Duration(100ms); // delete after 100ms
      keepout_marker->color.a = 1.0;                       // Don't forget to set the alpha!
      keepout_marker->color.r = 1.0;
      keepout_marker->color.g = 0.0;
      keepout_marker->color.b = 0.0;
    }
  }
}
