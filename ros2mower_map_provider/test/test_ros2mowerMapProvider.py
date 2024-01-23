from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
 
import os
import pytest
import unittest
import rclpy

from ros2mower_msgs.srv import GetNumOfAreas, GetArea, SetArea
from ros2mower_msgs.msg import MapArea
from geometry_msgs.msg import Polygon, Point32

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

        assert self.future.result().count == 2, 'Num of counts invalid, expect 2 got: ' + str(self.future.result().count)

    def test_GetArea_byID(self):
        # register and call service client
        self.client_getArea = self.node.create_client(GetArea, 'get_area')
        while not self.client_getArea.wait_for_service(timeout_sec=5.0):
            assert 0 == 1, 'Service GetArea not ready'
        request = GetArea.Request()
        request.index = 0
        self.future = self.client_getArea.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)

        # check result
        result = self.future.result()
        assert result.id == 0, 'wrong ID, expect 0, got: ' + str(result.id)
        assert result.name == 'Greenhouse', 'wrong area, expect Greenhouse, got: ' + result.name
        assert len(result.outer_polygon) == 5, 'outer polygon count wrong, expect 5 got: ' + str(len(result.outer_polygon))
        assert len(result.keepout_zones) == 0, 'keepout zones count wrong, expect 0, got: ' + str(len(result.keepout_zones))

    def test_GetArea_byName(self):
        # register and call service client
        self.client_getAreaByName = self.node.create_client(GetArea, 'get_area')
        while not self.client_getAreaByName.wait_for_service(timeout_sec=5.0):
            assert 0 == 1, 'Service GetArea not ready'
        request = GetArea.Request()
        request.name = 'Fishpond'
        self.future = self.client_getAreaByName.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)

        # check result
        result = self.future.result()
        assert result.id == 1, 'wrong ID, expect 1, got: ' + str(result.id)
        assert result.name == 'Fishpond', 'wrong area, expect Fishpond, got: ' + result.name
        assert len(result.outer_polygon) == 5, 'outer polygon count wrong, expect 5 got: ' + str(len(result.outer_polygon))
        assert len(result.keepout_zones) == 2, 'keepout zones count wrong, expect 2, got: ' + str(len(result.keepout_zones))
        assert len(result.keepout_zones[0].polygon) == 2, 'keepout zone 0 wrong polygon count, expect 2, got: ' + str(len(result.keepout_zones[0].polygon))
        assert len(result.keepout_zones[1].polygon) == 4, 'keepout zone 0 wrong polygon count, expect 4, got: ' + str(len(result.keepout_zones[1].polygon))

    def test_setArea(self):
        # register client
        self.client_SetArea = self.node.create_client(SetArea, 'set_area')
        while not self.client_SetArea.wait_for_service(timeout=5.0):
            assert 0 == 1, 'Service set_area not ready'
        request = SetArea.Request()
        new_area = MapArea()

        # create new area
        new_area.name = 'Test Area'
        new_area.outer_polygon.points.append(Point32(x=0, y=0)) 
        new_area.outer_polygon.points.append(Point32(x=0, y=2))
        new_area.outer_polygon.points.append(Point32(x=2, y=2))
        new_area.outer_polygon.points.append(Point32(x=2, y=0))
        new_area.outer_polygon.points.append(Point32(x=0, y=0))
        keepout_zones = []
        zone = Polygon()
        zone.points.append(Point32(x=0.5, y=0.5))
        zone.points.append(Point32(x=0.5, y=1.5))
        zone.points.append(Point32(x=1.5, y=1.5))
        zone.points.append(Point32(x=1.5, y=0.5))
        zone.points.append(Point32(x=0.5, y=0.5))
        keepout_zones.append(zone)
        new_area.keepout_zones = keepout_zones

        # call client
        self.future = self.client_SetArea.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)

        # check result
        result = self.future.result()
        assert result.id == 2, 'new area got wrong id, expected 2, got: ' + str(result.id)

        # verify newly created area
        request_new = GetArea.Request()
        request_new.index = result.id
        self.future = self.client_getArea.call_async(request_new)
        rclpy.spin_until_future_complete(self.node, self.future)
        
        response_new = self.future.result()
        assert response_new.name == 'Test Area', 'wrong name for new area, expected Test Area, got: ' + response_new.name
        assert len(new_area.outer_polygon) == len(response_new.outer_polygon), 'new Area outer polygon count mismatch: ' + len(new_area.outer_polygon) + ' vs. ' + len(response_new.outer_polygon)
        index = -1
        for point in new_area.outer_polygon:
            index += 1
            assert point.x == response_new.outer_polygon.points[index].x, 'outer poly wrong'
            assert point.y == response_new.outer_polygon.points[index].y, 'outer poly wrong'
        
        assert len(new_area.keepout_zones) == len(response_new.keepout_zones), 'new Area keepout zones count mismatch: ' + len(new_area.keepout_zones) + ' vs. ' + len(response_new.keepout_zones)
        index = -1
        for point in new_area.keepout_zones[0]:
            index += 1
            assert point.x == response_new.keepout_zones[0].points[index].x, 'keepout zone wrong'
            assert point.y == response_new.keepout_zones[0].points[index].y, 'keepout zone wrong'
    
    def test_getAll_Keepouts(self):
        # register client
        self.client_allKeepouts = self.node.create_client(GetArea, 'get_all_keepout_zones')
        while not self.client_allKeepouts.wait_for_service(timeout=5.0):
            assert 0 == 1, 'Service set_area not ready'
        request = GetArea.Request()
        self.future = self.client_allKeepouts.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)
        response = self.future.result()

        # validate results
        assert len(response.outer_polygon) == 0, 'outer polygon must be empty'
        assert len(response.keepout_zones) == 3, 'expect 3 keepout zones, got: ' + str(len(response.keepout_zones))
        assert len(response.keepout_zones[0].points) == 2, 'first keepout zone point mismatch, expect 2, got: ' + len(response.keepout_zones[0].points)
        assert len(response.keepout_zones[1].points) == 4, 'first keepout zone point mismatch, expect 4, got: ' + len(response.keepout_zones[0].points)
        assert len(response.keepout_zones[2].points) == 5, 'first keepout zone point mismatch, expect 5, got: ' + len(response.keepout_zones[0].points)
        

