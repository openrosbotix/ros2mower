# ros2mower_map_provider
This package provides services to load, access and store mow area definitions.
Each mow area consists out of an outher polygon (X and Y points only). Also it may contains keepout zones (no go areas). These polygons defines inner areas, which the mower should not access. 
Like your fish pond or the roses of your wife.

# Plugins
This package uses plugins whereas each plugin represents an implementation of map provider. This allows us to use different data sources of mow areas, like flat files, databases, web services or whatever you implement. To build your own plugin, create a class inheriting from plugin_base.

## ros2mowerMapProvider
This is a working plugin of map provider. The mow area definition is stored as YAML file

### map file
An example of mow area YAML file can be found under /example/mow_area.yaml
Each area is defined by a name, an outer polygon and an array of keepout zones represented by polygons.

## template
This plugin can be used as a draft for your own plugin.

# Usage

## Launch
run map_provider.launch.py file to start map provider node by running this command
```
ros2 launch ros2mower_map_provider map_provider.launch.py
```
## Services
These services are available
### Get area
Get data of a single mow area by it's name
```
/map_provider/get_area
```
### Get area list
Get total number of mow areas as well as their names
```
/map_provider/get_area_list
```

### Get all keepout zones
collect all keppout zones of all mow areas in a single request. This can be used to build a costmap for nav2 containing all keepout zones
```
/map_provider/get_all_keepout_zones
```
### Set area
add or override a mow area. Name of mow area is used as key. If a mow area with the requested name already exists, it gets overridden. Otherwise it gers added as new area. This service returns the total number of mow areas after update.
Please note that changes are not persist yet. So your YAML file (or any other source) will not be changed
```
/map_provider/set_area
```

### Save map
this service is used to persist any changes. So to save your changes made by set area service, call save area service afterwards.
```
/map_provider/save_map
```

# PyTest
This package includes pytest class. To run tests, use this command
```
colcon test --event-handlers console_cohesion+ --packages-select ros2mower_map_provider
```
