cmake_minimum_required(VERSION 3.5)
project(ros2mower_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/MapArea.msg"
   "srv/GetArea.srv"
   "srv/GetNumOfAreas.srv"
   DEPENDENCIES geometry_msgs builtin_interfaces
    LIBRARY_NAME ${PROJECT_NAME})

 


include_directories(include)
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_package()