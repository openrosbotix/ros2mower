cmake_minimum_required(VERSION 3.8)
project(ros2mower_area_recording)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2mower_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${Boost_INCLUDE_DIRS}
)

set(TARGET_DEPENDS
    ament_index_cpp rclcpp tf2_ros
    tf2
    tf2_geometry_msgs
    visualization_msgs
)

add_executable(${PROJECT_NAME} 
src/ros2mower_area_recording.cpp
src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs
  sensor_msgs ros2mower_msgs geometry_msgs tf2_geometry_msgs visualization_msgs tf2_ros
)

install(TARGETS
  ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
   launch
   config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()
