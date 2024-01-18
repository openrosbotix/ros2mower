from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
 
import os
import pytest
import unittest
import rclpy

from ros2mower_msgs.srv import GetNumOfAreas, GetArea

@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory('ros2mower_map_provider')
    map_file = pkg_share + '/example/mow_area.yaml'
    
    map_provider = Node(
        package='ros2mower_map_provider',
        executable='map_provider',
        parameters=[
            {
                'map_file': map_file,
                'provider_plugin': 'ros2mowerMapProvider'
            }]
    )
 
    return LaunchDescription([
        map_provider,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    )

class TestCases(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_num_of_areas(self):
        self.client_num_areas = self.node.create_client(GetNumOfAreas, 'get_num_of_areas')
        while not self.client_num_areas.wait_for_service(timeout_sec=5.0):
            assert 0 == 1, 'serivce not ready'
        request = GetNumOfAreas.Request()
        self.future = self.client_num_areas.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)

        assert self.future.result().count == 2, 'Num of counts invalid, await 2 got: ' + str(self.future.result().count)
        
        

