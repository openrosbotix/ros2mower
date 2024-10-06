# ROS2Mower
A lawn mower operated with ROS2

This package implements the behavior of a lawn mower. I tried my best to keep it as hardware independent as possible. So it should make no difference if you use a real lawn mower or a simulated Turtlebot3 robot.

# Operation
## Mapping
First you need to introduce your robot to your garden. To do so, launch the xyz.launch.py file. This will start SLAM toolbox of nav2. Drive your robot around with keyboard or joystick and build your map.
Don't forget to save the map

## Defining mow areas
After map has been created, it's time to record the mowing areas. Start abc.launch.py file. This will start a teleop robot. By serivce calls, you can record an area by simply driving around. Every x cm, the actual pose will be saved and forms your polygon. After saving the outer polygon of an area, you can start to record the inner holes. This inner polygons will be excluded during mow and form some kind of keepout zone.

To ease necessary service calls, I higly recommend to use a joystick

This gets handeled by package map_provider

## Mow
After map creation and area recording, you can start mower by launch def.launch.py. At the beginning, mower will be in idle or station mode. Start the mower by call service abcxyz. The mower will start to mow the first mowing area. Therefore it first creates a path array and forward them to nav2 waypoint follower. After completion, the next mow area will be processed.

## Launch simulation
To launch ROS2Mower with a simulated HoverMower, you need to start these commands

```console
ros2 launch hovermower_simulation hm_simulation.launch.py
ros2 launch hovermower_navigation2 localization_launch.py use_sim_time:=true map:=garden_map.yaml
ros2 launch hovermower_navigation2 navigation2.launch.py use_sim_time:=True

ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: -6.201, y: -5.219, z: 0.0}, orientation: {w: -0.082}}, } }'
```