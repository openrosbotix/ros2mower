#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from ros2mower_msgs.srv import GetArea, GetNumOfAreas
from ros2mower_msgs.msg import MapArea
from geometry_msgs.msg import Polygon, Point32
from ros2mower_map_provider.mapPlugins import provider

class MapProviderNode(Node):
    def __init__(self):
        super().__init__('map_provider')
        self.declare_parameter('map_file', 'example/mow_area.yaml' )
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.map_file = "ros2mower/ros2mower_map_provider/example/mow_area.yaml"
        self.map_provider = provider.MapProvider("ros2mowerMapProvider", self.map_file)
        
        self.map_provider.load_map()
        self.srv = self.create_service(GetArea, 'get_area', self.get_area)
        self.srv = self.create_service(GetNumOfAreas, 'get_num_of_areas', self.get_num_of_areas)

    def get_area(self, request, response):

        response.area = self.map_provider.get_area(request.index)
        return response    
    
    def get_num_of_areas(self, request, response):
        response.count = self.map_provider.get_num_of_areas()
        return response

def main(args=None):
    rclpy.init(args=args)

    map_provider = MapProviderNode()

    rclpy.spin(map_provider)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map_provider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    