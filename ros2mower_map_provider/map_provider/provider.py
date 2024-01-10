import rclpy

from rclpy.node import Node
from ros2mower_msgs.srv import GetArea, GetNumOfAreas, MapArea
from geometry_msgs.msg import Polygon, Point32
import yaml

class MapProvider(Node):
    def __init__(self):
        super().__init__('map_provider')
        self.declare_parameter('map_file', 'my_map.yaml' )
        
        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
        self.srv = self.create_service(GetArea, 'get_area', self.get_area)
        self.srv = self.create_service(GetNumOfAreas, 'get_num_of_areas', self.get_num_of_areas)

    
    def load_map(self):
        #open map file and store it to local map object
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        with open(map_file, 'r') as f:
            self.map = yaml.safe_load(f)

    def save_map(self):
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        with open(map_file, 'w') as file:
            yaml.dump(self.map, file)

    def get_area(self, request, response):
        #read requested area definition
        map_area = self.map['area'][request.index]

        # extract outer polygon
        outer_poly = Polygon()
        for point in map_area['outer_polygon']:
            outer_poly.points.append(Point32(x=point['x'], y=point['y']))            
            
        # extract keepout zones
        keepout_zones = [] 
        for zones in map_area['keepout_zones']:
            for zone in zones['zone']:
                keepout_poly = Polygon()
                for point in zone['polygon']:
                    keepout_poly.points.append(Point32(x=point['x']), y=point['y'])
                keepout_zones.append(keepout_poly)  
        
        # build response
        area = MapArea()
        area.outer_polygon = outer_poly
        area.keepout_zones =keepout_zones
        response.area = area    
        return response    
    
    def get_num_of_areas(self, request, response):
        response.count = sum([len(x["users"]) for x in self.map["area"]])
        return response

def main(args=None):
    rclpy.init(args=args)

    map_prodiver = MapProvider()

    rclpy.spin(map_prodiver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map_prodiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    