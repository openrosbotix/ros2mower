from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
 
import os
import pytest
import unittest

@pytest.mark.launch_test
def generate_test_description():
 
    pkg_share = get_package_share_directory('ros2mower_map_provider')
    map_file = pkg_share + '/example/mow_area.yaml'
 
    map_provider = Node(
        package='ros2mower_map_provider',
        executable='provider_node',
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
 
    context = {'ros2mower_map_provider': map_provider}
 
    return LaunchDescription([
        map_provider,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
 
    def test_exit_code(self, proc_output, proc_info, ndt_mapper):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=ndt_mapper)
