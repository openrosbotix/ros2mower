from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
 
import os
import pytest
import unittest
import rclpy

from ros2mower_msgs.srv import GetAreaList, GetArea, SetArea
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

    @classmethod 
    def setUpClass(self):
        rclpy.init()
        self.node = rclpy.create_node('test_node')
        self.client_getAreaByName = self.node.create_client(GetArea, 'map_provider/get_area')
        self.client_area_list = self.node.create_client(GetAreaList, 'map_provider/get_area_list')
        self.client_SetArea = self.node.create_client(SetArea, 'map_provider/set_area')
        self.client_allKeepouts = self.node.create_client(GetArea, 'map_provider/get_all_keepout_zones')

    @classmethod 
    def tearDownClass(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_1_area_list(self):
        while not self.client_area_list.wait_for_service(timeout_sec=5.0):
            assert 0 == 1, 'serivce not ready'
        request = GetAreaList.Request()
        self.future = self.client_area_list.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)

        assert self.future.result().count == 2, 'Num of counts invalid, expect 2 got: ' + str(self.future.result().count)
        assert len(self.future.result().area_names) == 2, 'Array with area names invalid'

    # def test_2_GetArea_byID(self):
    #     # register and call service client
    #     self.client_getArea = self.node.create_client(GetArea, 'get_area')
    #     while not self.client_getArea.wait_for_service(timeout_sec=5.0):
    #         assert 0 == 1, 'Service GetArea not ready'
    #     request = GetArea.Request()
    #     request.index = 0
    #     self.future = self.client_getArea.call_async(request)
    #     rclpy.spin_until_future_complete(self.node, self.future)

    #     # check result
    #     result = self.future.result()
    #     assert result.area.id == 1, 'wrong ID, expect 1, got: ' + str(result.area.id)
    #     assert result.area.name.data == 'Greenhouse', 'wrong area, expect Greenhouse, got: ' + result.area.name.data
    #     assert len(result.area.outer_polygon.points) == 5, 'outer polygon count wrong, expect 5 got: ' + str(len(result.area.outer_polygon.points))
    #     assert len(result.area.keepout_zones) == 0, 'keepout zones count wrong, expect 0, got: ' + str(len(result.area.keepout_zones))

    def test_3_GetArea_byName(self):
        # register and call service client
        while not self.client_getAreaByName.wait_for_service(timeout_sec=5.0):
            assert 0 == 1, 'Service GetArea not ready'
        request = GetArea.Request()
        request.name.data = 'Fishpond'
        self.future = self.client_getAreaByName.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)

        # check result
        result = self.future.result()
        assert result.area.name.data == 'Fishpond', 'wrong area, expect Fishpond, got: ' + result.area.name.data
        assert len(result.area.outer_polygon.points) == 5, 'outer polygon count wrong, expect 5 got: ' + str(len(result.area.outer_polygon.points))
        assert len(result.area.keepout_zones) == 2, 'keepout zones count wrong, expect 2, got: ' + str(len(result.area.keepout_zones))
        assert len(result.area.keepout_zones[0].points) == 2, 'keepout zone 0 wrong polygon count, expect 2, got: ' + str(len(result.area.keepout_zones[0].points))
        assert len(result.area.keepout_zones[1].points) == 4, 'keepout zone 0 wrong polygon count, expect 4, got: ' + str(len(result.area.keepout_zones[1].points))

    def test_4_setArea(self):
        # register client
        while not self.client_SetArea.wait_for_service(timeout_sec=5.0):
            assert 0 == 1, 'Service set_area not ready'
        request = SetArea.Request()
        new_area = MapArea()

        # create new area
        new_area.name.data = 'Test Area'
        new_area.outer_polygon.points.append(Point32(x=0.0, y=0.0)) 
        new_area.outer_polygon.points.append(Point32(x=0.0, y=2.0))
        new_area.outer_polygon.points.append(Point32(x=2.0, y=2.0))
        new_area.outer_polygon.points.append(Point32(x=2.0, y=0.0))
        new_area.outer_polygon.points.append(Point32(x=0.0, y=0.0))
        keepout_zones = []
        zone = Polygon()
        zone.points.append(Point32(x=0.5, y=0.5))
        zone.points.append(Point32(x=0.5, y=1.5))
        zone.points.append(Point32(x=1.5, y=1.5))
        zone.points.append(Point32(x=1.5, y=0.5))
        zone.points.append(Point32(x=0.5, y=0.5))
        keepout_zones.append(zone)
        new_area.keepout_zones = keepout_zones
        request.area = new_area

        # call client
        self.future = self.client_SetArea.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)

        # check result
        result = self.future.result()
        assert result.count == 3, 'new area got wrong total count, expected 3, got: ' + str(result.count)

        # verify newly created area
        while not self.client_getAreaByName.wait_for_service(timeout_sec=5.0):
            assert 0 == 1, 'Service get_area not ready'
        request_new = GetArea.Request()
        request_new.name.data = 'Test Area'
        self.future1 = self.client_getAreaByName.call_async(request_new)
        rclpy.spin_until_future_complete(self.node, self.future1)
        
        response_new = self.future1.result()
        assert response_new.area.name.data == 'Test Area', 'wrong name for new area, expected Test Area, got: ' + response_new.area.name.data
        assert len(new_area.outer_polygon.points) == len(response_new.area.outer_polygon.points), 'new Area outer polygon count mismatch: ' + str(len(new_area.outer_polygon.points)) + ' vs. ' + str(len(response_new.area.outer_polygon.points))
        index = -1
        for point in new_area.outer_polygon.points:
            index += 1
            assert point.x == response_new.area.outer_polygon.points[index].x, 'outer poly wrong'
            assert point.y == response_new.area.outer_polygon.points[index].y, 'outer poly wrong'
        
        assert len(new_area.keepout_zones) == len(response_new.area.keepout_zones), 'new Area keepout zones count mismatch: ' + str(len(new_area.keepout_zones)) + ' vs. ' + str(len(response_new.area.keepout_zones))
        index = -1
        for point in new_area.keepout_zones[0].points:
            index += 1
            assert point.x == response_new.area.keepout_zones[0].points[index].x, 'keepout zone wrong'
            assert point.y == response_new.area.keepout_zones[0].points[index].y, 'keepout zone wrong'

    def test_5_getAll_Keepouts(self):
        # register client
        while not self.client_allKeepouts.wait_for_service(timeout_sec=5.0):
            assert 0 == 1, 'Service set_area not ready'
        request = GetArea.Request()
        self.future = self.client_allKeepouts.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)
        response = self.future.result().area

        # validate results
        assert len(response.outer_polygon.points) == 0, 'outer polygon must be empty'
        assert len(response.keepout_zones) == 3, 'expect 3 keepout zones, got: ' + str(len(response.keepout_zones))
        assert len(response.keepout_zones[0].points) == 2, 'first keepout zone point mismatch, expect 2, got: ' + len(response.keepout_zones[0].points)
        assert len(response.keepout_zones[1].points) == 4, 'first keepout zone point mismatch, expect 4, got: ' + len(response.keepout_zones[0].points)
        assert len(response.keepout_zones[2].points) == 5, 'first keepout zone point mismatch, expect 5, got: ' + len(response.keepout_zones[0].points)
        

