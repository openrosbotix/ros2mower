from ros2mower_msgs.msg import MapArea
from geometry_msgs.msg import Polygon, Point32
import yaml
from ros2mower_map_provider.mapPlugins.plugin_base import MapProviderBase

class ros2mower_MapProvider(MapProviderBase):
    def __init__(self, map_file):
        self.map_file=map_file
        self.map = None

        self.load_map()
        
    
    def load_map(self):
        #open map file and store it to local map object
        try:
            f = open(self.map_file, 'r')
            try:
                self.map = yaml.safe_load(f)
            finally:    
                f.close()
        except (IOError, OSError) as e:
            print(e)

    def save_map(self): 
        with open(self.map_file, 'w') as file:
            yaml.dump(self.map, file)

    def get_area(self, area_index):
        area = MapArea()

        if self.map != None and len(self.map['mow_areas']) > area_index:
        #read requested area definition
            map_area = self.map['mow_areas'][area_index]
            area.id = map_area['id']
            area.name.data = map_area['name']  
            # extract outer polygon
            outer_poly = Polygon()
            for point in map_area['outer_polygon']:
                outer_poly.points.append(Point32(x=point['x'], y=point['y']))            
            
            # extract keepout zones
            keepout_zones = []
            try: 
                for zone in map_area['keepout_zones']:
                    keepout_poly = Polygon()
                    for point in zone['polygon']:
                        keepout_poly.points.append(Point32(x=point['x'], y=point['y']))
                    keepout_zones.append(keepout_poly)  
            except KeyError:
                pass # no keepout aread defined, it's ok
        
            # build response
            area.outer_polygon = outer_poly
            area.keepout_zones =keepout_zones
        return area

    def get_area_by_name(self, name):
        # determine index of the given name
        if self.map != None:
            index = -1

            for area in self.map['mow_areas']:
                index += 1
                if area['name'] == name:
                    # name found
                    return self.get_area(index)


        # if we reach this point, either map is empty or name not found
        area = MapArea()
        return area
    
    def get_num_of_areas(self):
        if self.map == None:
            return -1
        response = 0
        for area in self.map["mow_areas"]:
            response += 1

        return response
    
    def get_all_keepout_zones(self):
        keepout_zones = [] 

        if self.map != None:
            index = -1
            for areas in self.map['mow_areas']:
                index += 1
                area = self.get_area(index)

                for zone in area.keepout_zones:
                    keepout_zones.append(zone)        
        area = MapArea()
        area.keepout_zones = keepout_zones

        return area

def main(args=None):
    map_prodiver = ros2mower_MapProvider("ros2mower/ros2mower_map_provider/example/mow_area.yaml")
    #print(map_prodiver.get_num_of_areas())
    #print(map_prodiver.get_area_by_name('Greenhouse'))
    print(map_prodiver.get_all_keepout_zones())
if __name__ == '__main__':
    main()    