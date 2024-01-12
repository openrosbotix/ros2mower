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
        pass

    def save_map(self): 
        pass

    def get_area(self, area_index):
        #read requested area definition
        # build outer polygon
        outer_poly = Polygon()
        outer_poly.points.append(Point32(x=0.0, y=0.0, z=0.0))
        outer_poly.points.append(Point32(x=0.0, y=1.0, z=0.0))       
        outer_poly.points.append(Point32(x=1.0, y=1.0, z=0.0))            
        outer_poly.points.append(Point32(x=1.0, y=0.0, z=0.0))  
        outer_poly.points.append(Point32(x=0.0, y=0.0, z=0.0))                                   
            
        # build keepout zones
        keepout_zones = [] 
        keepout_poly = Polygon()
        keepout_poly.points.append(Point32(x=0.25, y=0.25, z=0.0))
        keepout_poly.points.append(Point32(x=0.25, y=0.5, z=0.0))
        keepout_poly.points.append(Point32(x=0.5, y=0.5, z=0.0))
        keepout_poly.points.append(Point32(x=0.5, y=0.25, z=0.0))        
        keepout_poly.points.append(Point32(x=0.25, y=0.25, z=0.0))        
        keepout_zones.append(keepout_poly)  
        
        # build response
        area = MapArea()
        area.outer_polygon = outer_poly
        area.keepout_zones =keepout_zones
        area.id = 5
        area.name.data = "Test"
        return area
        
    
    def get_num_of_areas(self):
        return 1
    
    def get_area_by_name(self, name):
        return self.get_area(1)
    
    def get_all_keepout_zones(self):
        return super().get_all_keepout_zones()

def main(args=None):
    map_prodiver = ros2mower_MapProvider("filename.yaml")
    print(map_prodiver.get_num_of_areas())
if __name__ == '__main__':
    main()    