cmake_minimum_required(VERSION 3.8)
project(ros2mower_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
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
 include/ros2mower_robot
 ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
src/ros2mower_robot.cpp
src/main.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs
  sensor_msgs
  
)

install(TARGETS
  ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY
#  launch
#  conf
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()