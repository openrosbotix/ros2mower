#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from ros2mower_msgs.srv import GetArea, GetNumOfAreas, SetArea, SaveMap
from ros2mower_map_provider.mapPlugins import provider
from rcl_interfaces.msg import ParameterDescriptor


class MapProviderNode(Node):
    def __init__(self):
        super().__init__('map_provider')
        map_file_parameter_descriptor = ParameterDescriptor(description='Path and filename of map')
        self.declare_parameter('map_file', 'example/mow_area.yaml', map_file_parameter_descriptor)

        provider_plugin_parameter_descriptor = ParameterDescriptor(
            description='name of plugin which provides info of map')
        self.declare_parameter('provider_plugin', 'ros2mowerMapProvider',
                                provider_plugin_parameter_descriptor)

        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value

        self.map_provider_plugin = self.get_parameter('provider_plugin').get_parameter_value().string_value
        self.map_provider = provider.MapProvider(self.map_provider_plugin, self.map_file)

        self.map_provider.load_map()
        self.srv_get_area = self.create_service(GetArea, 'get_area', self.get_area)
        self.srv_get_keepout = self.create_service(GetArea, 'get_all_keepout_zones',
                                                   self.get_all_keepout_zones)
        self.srv_get_num = self.create_service(GetNumOfAreas, 'get_num_of_areas',
                                               self.get_num_of_areas)
        self.srv_set_area = self.create_service(SetArea, 'set_area', self.set_area)
        self.srv_save_map = self.create_service(SaveMap, 'save_map', self.save_map)

        self.get_logger().info('Mow area map file: ' + self.map_file)

    def get_area(self, request, response):
        if request.name.data == '':
            response.area = self.map_provider.get_area(request.index)
        else:
            response.area = self.map_provider.get_area_by_name(request.name.data)
        return response

    def get_num_of_areas(self, request, response):
        response.count = self.map_provider.get_num_of_areas()
        if response.count == -1:
            self.get_logger().warn('No map data available')
        return response

    def get_all_keepout_zones(self, request, response):
        response.area = self.map_provider.get_all_keepout_zones()
        return response

    def set_area(self, request, response):
        response.id = self.map_provider.set_area(request.area)
        return response

    def save_map(self, request, response):
        self.save_map()
        response.success = True
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
