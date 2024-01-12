uint32 id                           # id of area. This is not the index in map array This is the unique key of the area
std_msgs/String name                # name of the area
geometry_msgs/Polygon outer_polygon
geometry_msgs/Polygon[] keepout_zones