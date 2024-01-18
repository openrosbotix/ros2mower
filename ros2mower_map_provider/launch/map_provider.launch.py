
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2mower_map_provider')
    map_file = pkg_share + '/example/mow_area.yaml'
    
    map_provider = Node(
        package='ros2mower_map_provider',
        executable='map_provider',
        parameters=[
      #      os.path.join(
      #          get_package_share_directory('ros2mower_map_provider'),
      #          'param/test.param.yaml'
      #      ),
            {
                'map_file': map_file,
                'provider_plugin': 'ros2mowerMapProvider'
            }]
    )
 
   
    return LaunchDescription([
        map_provider,
   ])