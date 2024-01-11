from ros2mower_msgs.msg import MapArea
from geometry_msgs.msg import Polygon, Point32
import yaml
from ros2mower_map_provider.mapPlugins.plugin_base import MapProviderBase

class ros2mower_MapProvider(MapProviderBase):
    def __init__(self, map_file):
        self.map_file=map_file
        self.load_map()
        
    
    def load_map(self):
        #open map file and store it to local map object
        with open(self.map_file, 'r') as f:
            self.map = yaml.safe_load(f)

    def save_map(self): 
        with open(self.map_file, 'w') as file:
            yaml.dump(self.map, file)

    def get_area(self, area_index):
        #read requested area definition
        map_area = self.map['mow_areas'][area_index]
        
        # extract outer polygon
        outer_poly = Polygon()
        for point in map_area['outer_polygon']:
            outer_poly.points.append(Point32(x=point['x'], y=point['y']))            
            
        # extract keepout zones
        keepout_zones = [] 
        for zone in map_area['keepout_zones']:
            keepout_poly = Polygon()
            for point in zone['polygon']:
                keepout_poly.points.append(Point32(x=point['x'], y=point['y']))
            keepout_zones.append(keepout_poly)  
        
        # build response
        area = MapArea()
        area.outer_polygon = outer_poly
        area.keepout_zones =keepout_zones
        return area
        
    
    def get_num_of_areas(self):
        response = 0
        for area in self.map["mow_areas"]:
            response += 1

        return response

def main(args=None):
    map_prodiver = ros2mower_MapProvider("ros2mower/ros2mower_map_provider/example/mow_area.yaml")
    print(map_prodiver.get_num_of_areas())
if __name__ == '__main__':
    main()    