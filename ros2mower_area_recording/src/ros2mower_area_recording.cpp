#include "ros2mower_area_recording.hpp"

using namespace std::chrono_literals;

ROS2Mower_AreaRecording::ROS2Mower_AreaRecording(std::string name) : rclcpp::Node(name)
{

  this->declare_node_parameters();

  // register services
  _srv_set_area = this->create_client<ros2mower_msgs::srv::SetArea>("map_provider/set_area");
  _srv_save_map = this->create_client<ros2mower_msgs::srv::SaveMap>("map_provider/save_map");

  // wait for services to become available
  while (!this->_srv_set_area->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "interrupt while waiting for service");
    }
    RCLCPP_INFO(this->get_logger(), "SetArea service not available, wait for service");
  }

  while (!this->_srv_save_map->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "interrupt while waiting for service");
    }
    RCLCPP_INFO(this->get_logger(), "SaveMap service not available, wait for service");
  }

  // register parameter change callback handle
  this->_callbackParameter = this->add_on_set_parameters_callback(
      std::bind(&ROS2Mower_AreaRecording::parametersCallback, this, std::placeholders::_1));

  this->_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->_tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  // register publisher
  this->_pub_mow_area = this->create_publisher<visualization_msgs::msg::MarkerArray>("ros2mower_area_rec/mow_area", 3);
  this->_pub_polygon = this->create_publisher<visualization_msgs::msg::Marker>("ros2mower_area_rec/polygon", 3);

  // register subscriber
  this->_sub_joy = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 100, std::bind(&ROS2Mower_AreaRecording::callbackJoystick, this, std::placeholders::_1));

  // define timer callback for publishing state
  this->_timer_publisher = this->create_wall_timer(
      100ms, std::bind(&ROS2Mower_AreaRecording::timer_callback_publisher, this));

  rclcpp::Time now = this->get_clock()->now();
  this->_map_area.name.data = std::to_string(now.seconds());
  RCLCPP_INFO(this->get_logger(), "Area recording started. Ready for recording area named: %s", this->_map_area.name.data.c_str());
}

ROS2Mower_AreaRecording::~ROS2Mower_AreaRecording() {}

void ROS2Mower_AreaRecording::declare_node_parameters()
{

  declare_parameter("joy_toggle_polygon", 3);
  declare_parameter("joy_keepout", 0);
  declare_parameter("joy_area", 2);
  declare_parameter("joy_clear", 11);
  declare_parameter("joy_clear_all", 12);
  declare_parameter("joy_save", 1);
  declare_parameter("distance_points", 0.1);
  declare_parameter("publish_polygon", true);
  declare_parameter("base_frame", "base_footprint");
  declare_parameter("map_frame", "map");

  get_parameter("joy_toggle_polygon", this->_joy_btn_toggle_poly);
  get_parameter("joy_keepout", this->_joy_btn_keepout);
  get_parameter("joy_area", this->_joy_btn_area);
  get_parameter("joy_clear", this->_joy_btn_clear);
  get_parameter("joy_clear_all", this->_joy_btn_clear_all);
  get_parameter("joy_save", this->_joy_btn_save);
  get_parameter("distance_points", this->_distance_points);
  get_parameter("publish_polygon", this->_doPublishPolygon);
  this->_base_frame = get_parameter("base_frame").as_string();
  this->_map_frame = get_parameter("map_frame").as_string();

  // info about all parameters set
  RCLCPP_INFO(this->get_logger(), "Area recording: distance between points %f", this->_distance_points);
  RCLCPP_INFO(this->get_logger(), "Area recording: publish polygon and map %i", this->_doPublishPolygon);
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
  bool button_action = false;
  // debounce joystick messages
  if (msg->buttons[this->_joy_btn_toggle_poly] == 1 ||
      msg->buttons[this->_joy_btn_area] == 1 ||
      msg->buttons[this->_joy_btn_keepout] == 1 ||
      msg->buttons[this->_joy_btn_clear] == 1 ||
      msg->buttons[this->_joy_btn_clear_all] == 1 ||
      msg->buttons[this->_joy_btn_save] == 1)
  {
    rclcpp::Duration last_toggle = this->get_clock()->now() - this->_last_time_joy_button;
    if (last_toggle.seconds() > 0.5)
    {
      this->_last_time_joy_button = this->get_clock()->now();
      button_action = true;
    }
  }

  // toggle polygon recording on/off
  if (msg->buttons[this->_joy_btn_toggle_poly] == 1 && button_action)
  {
    this->_last_time_joy_button = this->get_clock()->now();
    // get actual pose from map when activating area recording
    if (this->getCurrentPose(this->_map_frame, this->_base_frame, 1.0) && this->_polygon_recording == false)
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
  if (msg->buttons[this->_joy_btn_area] == 1 && button_action)
  {

    if (this->_polygon_recording == true || this->_polygon.points.size() < 3)
    {
      RCLCPP_INFO(this->get_logger(), "Area recording: unable to save. Either invalid polygon or toggle polygon recording off first");
    }
    else
    {

      // add first point as last point to close polygon
      this->_polygon.points.push_back(this->_polygon.points.front());
      this->_map_area.outer_polygon = this->_polygon;
      RCLCPP_INFO(this->get_logger(), "Area recording: outer polygon saved with %lu points", this->_polygon.points.size());
      this->_polygon.points.clear();
    }
  }

  // save last polygon as keepout zone
  if (msg->buttons[this->_joy_btn_keepout] == 1 && button_action)
  {
    if (this->_polygon_recording == true || this->_polygon.points.size() < 3)
    {
      RCLCPP_INFO(this->get_logger(), "Area recording: unable to save. either invalid polygon or toggle polygon recording off first");
    }
    else
    {
      // add first point as last point to close polygon
      this->_polygon.points.push_back(this->_polygon.points.front());
      this->_map_area.keepout_zones.push_back(this->_polygon);
      RCLCPP_INFO(this->get_logger(), "Area recording: keepout zone saved with %lu points", this->_polygon.points.size());
      this->_polygon.points.clear();
    }
  }

  // clear last polygon
  if (msg->buttons[this->_joy_btn_clear] == 1 && button_action)
  {
    RCLCPP_INFO(this->get_logger(), "Area recording: clearing last polygon with %lu points", this->_polygon.points.size());
    this->_polygon.points.clear();
  }

  // clear all polygons
  if (msg->buttons[this->_joy_btn_clear_all] == 1 && button_action)
  {
    RCLCPP_INFO(this->get_logger(), "Area recording: clearing all recorded areas");
    this->_polygon.points.clear();
    this->_map_area.keepout_zones.clear();
    this->_map_area.outer_polygon.points.clear();
  }

  // save map as area
  if (msg->buttons[this->_joy_btn_save] == 1 && button_action)
  {
    // first check if map is not empty
    if (this->_map_area.outer_polygon.points.size() == 0)
    {
      RCLCPP_INFO(this->get_logger(), "Area recording: map empty, unable to save");
    }
    else
    {
      // set new area to map provider
      auto request_setArea = std::make_shared<ros2mower_msgs::srv::SetArea::Request>();
      request_setArea->area = this->_map_area;

      auto result_setArea = this->_srv_set_area->async_send_request(request_setArea, std::bind(&ROS2Mower_AreaRecording::response_setArea_callback, this, std::placeholders::_1));

      // // Wait for the result of set_area.
      // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_setArea) ==
      //     rclcpp::FutureReturnCode::SUCCESS)
      // {
      //   RCLCPP_INFO(this->get_logger(), "Area recording: setArea ok. Count of areas %i", result_setArea.get()->count);

      //   auto request_save_map = std::make_shared<ros2mower_msgs::srv::SaveMap::Request>();
      //   auto result = this->_srv_save_map->async_send_request(request_save_map);

      //   // Wait for the result of save map.
      //   if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      //       rclcpp::FutureReturnCode::SUCCESS)
      //   {
      //     RCLCPP_INFO(this->get_logger(), "Area recording: save ok. Status %i", result.get()->success);
      //   }
      //   else
      //   {
      //     RCLCPP_ERROR(this->get_logger(), "Area recording: Failed to call service save map of map provider");
      //   }
      // }
      // else
      // {
      //   RCLCPP_ERROR(this->get_logger(), "Area recording: Failed to call service setArea of map provider");
      // }
    }
  }

  // save new point to polygon
  if (this->_polygon_recording)
  {
    // get actual pose related to map
    if (this->getCurrentPose(this->_map_frame, this->_base_frame, 1))
    {
      // calculate distance between last pose and actual pose
      if (this->getDistanceToLastPose() >= this->_distance_points)
      {
        // add point to polygon
        auto new_point = geometry_msgs::msg::Point32();
        new_point.x = this->_actual_pose_map.pose.position.x;
        new_point.y = this->_actual_pose_map.pose.position.y;

        this->_polygon.points.push_back(new_point);

        // take current pose as last pose
        this->_last_pose_map = this->_actual_pose_map;
      }
    }
  }
}

void ROS2Mower_AreaRecording::response_setArea_callback(rclcpp::Client<ros2mower_msgs::srv::SetArea>::SharedFuture future)
{

  try
  {
    auto check = future.get()->count;
    if (check > 0)
    {
      RCLCPP_INFO(this->get_logger(), "Area recording: setArea ok. Count of areas %i", future.get()->count);

      auto request_save_map = std::make_shared<ros2mower_msgs::srv::SaveMap::Request>();
      //auto result = this->_srv_save_map->async_send_request(request_save_map, std::bind(&ROS2Mower_AreaRecording::response_save_callback, this, std::placeholders::_1));
      auto result_future = this->_srv_save_map->async_send_request(request_save_map);

      // Do this instead of rclcpp::spin_until_future_complete()
      std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
      if (status == std::future_status::ready) {
          RCLCPP_INFO(this->get_logger(), "Area recording: save ok. Status %i", result_future.get()->success);
         
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Area recording: Failed to call service setArea of map provider");
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Area recording: Failed to call service setArea of map provider");
  }
}

void ROS2Mower_AreaRecording::response_save_callback(rclcpp::Client<ros2mower_msgs::srv::SaveMap>::SharedFuture future)
{

  try
  {
    auto check = future.get()->success;
    if (check == true)
    {
      RCLCPP_INFO(this->get_logger(), "Area recording: save ok. Status %i", future.get()->success);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Area recording: Failed to call service save map of map provider");
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Area recording: Failed to call service setArea of map provider");
  }
}

bool ROS2Mower_AreaRecording::getCurrentPose(
    const std::string global_frame,
    const std::string robot_frame, const double transform_timeout)
{
  static rclcpp::Logger logger = this->get_logger();

  try
  {
    // this->_actual_pose_map
    auto transformStamped = this->_tf_buffer->lookupTransform(
        global_frame, robot_frame,
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
  double new_x = this->_actual_pose_map.pose.position.x;
  double old_x = this->_last_pose_map.pose.position.x;
  double new_y = this->_actual_pose_map.pose.position.y;
  double old_y = this->_last_pose_map.pose.position.y;

  return sqrt(pow(new_x - old_x, 2) +
              pow(new_y - old_y, 2));
}

void ROS2Mower_AreaRecording::timer_callback_publisher()
{

  if (this->_doPublishPolygon)
  {
    // create polygon marker for actual polygon recording
    visualization_msgs::msg::Marker polygon_marker;

    polygon_marker.header.frame_id = "map";
    polygon_marker.ns = "new_poly";
    polygon_marker.id = 0;
    polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    polygon_marker.scale.x = 0.05; // line width
    polygon_marker.color.r = 1.0;  // red
    polygon_marker.color.g = 0.0;
    polygon_marker.color.b = 0.0;
    polygon_marker.color.a = 1.0; // alpha

    // build marker for actually recorded polygon
    for (auto poly_point : this->_polygon.points)
    {
      geometry_msgs::msg::Point marker_point;
      marker_point.x = poly_point.x;
      marker_point.y = poly_point.y;
      marker_point.z = poly_point.z;
      polygon_marker.points.push_back(marker_point);
    }

    //----------------------------------------------
    // create marker array containing entire Map data
    visualization_msgs::msg::MarkerArray map_marker;

    // create marker for outer polygon
    visualization_msgs::msg::Marker outline_marker;
    outline_marker.header.frame_id = "map";
    outline_marker.ns = "ouline";
    outline_marker.id = 0;
    outline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    outline_marker.action = visualization_msgs::msg::Marker::ADD;
    outline_marker.scale.x = 0.05; // line width
    outline_marker.color.r = 0.0;  // red
    outline_marker.color.g = 1.0;
    outline_marker.color.b = 0.0;
    outline_marker.color.a = 1.0; // alpha

    for (auto outline_point : this->_map_area.outer_polygon.points)
    {
      geometry_msgs::msg::Point marker_point;
      marker_point.x = outline_point.x;
      marker_point.y = outline_point.y;
      marker_point.z = outline_point.z;
      outline_marker.points.push_back(marker_point);
    }

    map_marker.markers.push_back(outline_marker);

    //----------------------------------------------
    // create marker for keepout zones
    int keepout_id = 0;

    for (auto keepout_zone : this->_map_area.keepout_zones)
    {
      visualization_msgs::msg::Marker keepout_marker;
      keepout_id += 1;

      keepout_marker.header.frame_id = "map";
      keepout_marker.ns = "keepout_zone";
      keepout_marker.id = keepout_id;
      keepout_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      keepout_marker.action = visualization_msgs::msg::Marker::ADD;
      keepout_marker.scale.x = 0.05; // line width
      keepout_marker.color.r = 0.0;  // red
      keepout_marker.color.g = 0.0;
      keepout_marker.color.b = 1.0;
      keepout_marker.color.a = 1.0; // alpha

      // create marker for single keepout zone
      for (auto keepout_point : keepout_zone.points)
      {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = keepout_point.x;
        marker_point.y = keepout_point.y;
        marker_point.z = keepout_point.z;
        keepout_marker.points.push_back(marker_point);
      }

      map_marker.markers.push_back(keepout_marker);
    }

    this->_pub_polygon->publish(polygon_marker);
    this->_pub_mow_area->publish(map_marker);
  }
}
