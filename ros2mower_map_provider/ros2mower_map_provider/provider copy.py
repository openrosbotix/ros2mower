import rclpy

from rclpy.node import Node
from ros2mower_msgs.srv import GetArea, GetNumOfAreas
from ros2mower_msgs.msg import MapArea
from geometry_msgs.msg import Polygon, Point32
import yaml

class MapProvider:
    def __init__(self):
        
        self.load_map()

        self.get_area()
        
    
    def load_map(self):
        self.map_file="ros2mower/ros2mower_map_provider/example/mow_area.yaml"
        #open map file and store it to local map object
        with open(self.map_file, 'r') as f:
            self.map = yaml.safe_load(f)

    def save_map(self):
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        with open(map_file, 'w') as file:
            yaml.dump(self.map, file)

    def get_area(self):
        #read requested area definition
        map_area = self.map['mow_areas'][1]
        
        # extract outer polygon
        outer_poly = Polygon()
        for point in map_area['outer_polygon']:
            outer_poly.points.append(Point32(x=point['x'], y=point['y']))            
            
        # extract keepout zones
        keepout_zones = [] 
        for zone in map_area['keepout_zones']:
            keepout_poly = Polygon()
            print(zone)
            for point in zone['polygon']:
                keepout_poly.points.append(Point32(x=point['x'], y=point['y']))
            keepout_zones.append(keepout_poly)  
        
        # build response
        area = MapArea()
        area.outer_polygon = outer_poly
        area.keepout_zones =keepout_zones
        
    
    def get_num_of_areas(self, request, response):
        response.count = 0
        for area in self.map["mow_areas"]:
            response.count += 1

        return response

def main(args=None):
    map_prodiver = MapProvider()

if __name__ == '__main__':
    main()    