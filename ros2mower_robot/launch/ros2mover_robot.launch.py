from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_directory = os.path.join(
        get_package_share_directory('ros2mower_robot'),
        'config')
    params = os.path.join(config_directory, 'robot.yaml')

    ros2mower_robot = Node(
        package='ros2mower_robot',
        executable='ros2mower_robot',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        ros2mower_robot     
    ])