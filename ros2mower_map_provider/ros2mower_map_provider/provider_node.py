#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from ros2mower_msgs.srv import GetArea, GetAreaList, SetArea, SaveMap
from ros2mower_msgs.msg import MapArea
from ros2mower_map_provider.mapPlugins import provider
from std_msgs.msg import String
from std_msgs.msg import Header
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from PIL import Image, ImageDraw



class MapProviderNode(Node):
    def __init__(self):
        super().__init__('map_provider')
        map_file_parameter_descriptor = ParameterDescriptor(description='Path and filename of map')
        self.declare_parameter('map_file', 'example/mow_area.yaml', map_file_parameter_descriptor)

        provider_plugin_parameter_descriptor = ParameterDescriptor(
            description='name of plugin which provides info of map')
        self.declare_parameter('provider_plugin', 'ros2mowerMapProvider',
                                provider_plugin_parameter_descriptor)

        costmap_keepout_parameter_descriptor = ParameterDescriptor(
            description='publish keepout zones as costmap')
        self.declare_parameter('costmap_keepout', True,
                                costmap_keepout_parameter_descriptor)
        costmap_width_parameter_descriptor = ParameterDescriptor(
            description='costmap width [m]')
        self.declare_parameter('costmap_width', 10,
                                costmap_width_parameter_descriptor)
        costmap_height_parameter_descriptor = ParameterDescriptor(
            description='costmap height [m]')
        self.declare_parameter('costmap_height', 10,
                                costmap_height_parameter_descriptor)
        costmap_resolution_parameter_descriptor = ParameterDescriptor(
            description='resolution of costmap in [m/cell] ')
        self.declare_parameter('costmap_resolution', 0.1,
                                costmap_resolution_parameter_descriptor)
        
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value

        self.map_provider_plugin = self.get_parameter('provider_plugin').get_parameter_value().string_value
        self.map_provider = provider.MapProvider(self.map_provider_plugin, self.map_file)

        self.costmap_keepout = self.get_parameter('costmap_keepout').get_parameter_value().bool_value
        self.costmap_height = self.get_parameter('costmap_height').get_parameter_value().integer_value
        self.costmap_width = self.get_parameter('costmap_width').get_parameter_value().integer_value
        self.costmap_resolution = self.get_parameter('costmap_resolution').get_parameter_value().double_value
                        
        self.map_provider.load_map()
        self.srv_get_area = self.create_service(GetArea, 'map_provider/get_area', self.get_area)
        self.srv_get_keepout = self.create_service(GetArea, 'map_provider/get_all_keepout_zones',
                                                   self.get_all_keepout_zones)
        self.srv_get_area_list = self.create_service(GetAreaList, 'map_provider/get_area_list',
                                               self.get_area_list)
        self.srv_set_area = self.create_service(SetArea, 'map_provider/set_area', self.set_area)
        self.srv_save_map = self.create_service(SaveMap, 'map_provider/save_map', self.save_map)

        self.get_logger().info('Mow area map file: ' + self.map_file)
        
        if self.costmap_keepout == True:
            self.publish_keepout_costmap()

    def get_area(self, request, response):
        response.area = self.map_provider.get_area_by_name(request.name.data)
        return response

    def get_area_list(self, request, response):
        areas = self.map_provider.get_area_list()
        if len(areas) == 0:
            self.get_logger().warn('No map data available')
        response.count = len(areas)
        for area in areas:
            response.area_names.append(String(data = area))
        return response

    def get_all_keepout_zones(self, request, response):
        response.area = self.map_provider.get_all_keepout_zones()
        return response

    def set_area(self, request, response):
        response.count = self.map_provider.set_area(request.area)
        return response

    def save_map(self, request, response):
        self.save_map()
        response.success = True
        return response
    
    def publish_keepout_costmap(self):
        # prepare image
        costmap_image = Image.new(mode = "RGB", size = (int(self.costmap_width / self.costmap_resolution), int(self.costmap_height / self.costmap_resolution)),
                                  color ="white") 
        draw = ImageDraw.Draw(costmap_image)
        
        # get all keepout zones
        keepout_zones = self.map_provider.get_all_keepout_zones()
        
        # loop through all zones. Draw a polygon for each zone
        for zone in keepout_zones.keepout_zones:
            points =[]
            for point in zone.points:
                points.append((int(point.x / self.costmap_resolution), int(point.y / self.costmap_resolution)))     
            draw.polygon(points, fill="black")
        
        # flip image as image origin is different to ROS map origin    
        # for debugging purposes only
        # costmap_image = costmap_image.transpose(Image.FLIP_TOP_BOTTOM)
        # costmap_image.show()
        
        # create publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/ros2mower/costmap_keepout", 3)
        self.costmap_info_pub = self.create_publisher(CostmapFilterInfo, "/ros2mower/costmap_keepout/info", 3)
        
        # create costmap info
        costmap_header = Header()
        costmap_header.frame_id = "map"
        costmap_header.stamp = self.get_clock().now().to_msg()
        costmap_info = CostmapFilterInfo()
        costmap_info.filter_mask_topic = "/ros2mower/costmap_keepout"
        costmap_info.type = 0
        costmap_info.base = 0.0
        costmap_info.multiplier = 1.0
        costmap_info.header = costmap_header

        costmap_origin = Pose()
        costmap_origin.position.x = 0.0
        costmap_origin.position.y = 0.0
        costmap_origin.position.z = 0.0
                
        costmap_meta = MapMetaData()
        costmap_meta.height = costmap_image.height
        costmap_meta.width = costmap_image.width 
        costmap_meta.resolution = self.costmap_resolution
        costmap_meta.origin = costmap_origin
        
        costmap_data = OccupancyGrid()
        costmap_data.header = costmap_header
        costmap_data.info = costmap_meta

        for x in range(0, costmap_image.height):
            for y in range(0, costmap_image.width):
                # check if black
                if (costmap_image.getpixel((y,x)) == (0,0,0) ):
                    costmap_data.data.append(100)
                else:
                    costmap_data.data.append(0)

        self.costmap_pub.publish(costmap_data)
        self.costmap_info_pub.publish(costmap_info)
                
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
