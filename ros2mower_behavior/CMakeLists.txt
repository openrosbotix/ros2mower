cmake_minimum_required(VERSION 3.8)
project(ros2mower_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2mower_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(${PROJECT_NAME}  
src/main.cpp
src/behaviors/CheckBoolValue.cpp
src/behaviors/CheckBatteryVoltage.cpp
src/behaviors/CheckMission.cpp
src/behaviors/SetMission.cpp
src/behaviors/GetMowArea.cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 include/behaviors
 ${Boost_INCLUDE_DIRS}
)

set(TARGET_DEPENDS
    ament_index_cpp rclcpp rclcpp_action
    behaviortree_ros2 std_msgs sensor_msgs ros2mower_msgs
)


ament_target_dependencies(${PROJECT_NAME} ${TARGET_DEPENDS} 
)

install(TARGETS
  ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
#   launch
   config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(behaviortree_ros2 sensor_msgs std_msgs sensor_msgs ros2mower_msgs)

ament_package()




