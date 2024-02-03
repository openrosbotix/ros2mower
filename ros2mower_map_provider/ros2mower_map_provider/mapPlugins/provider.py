import importlib.util
import sys
import string
import secrets
from ament_index_python.packages import get_package_share_directory


class MapProvider:
    def __init__(self, plugin, map_file):
        pkg_share = get_package_share_directory('ros2mower_map_provider')
        map_provider_path = pkg_share + "/plugins/" + plugin + ".py"
        map_provider_module = self.load_module(map_provider_path, "map_provider")
        self.map_plugin = map_provider_module.ros2mower_MapProvider(map_file)
        self.load_map()

    def load_module(self, source, module_name=None):
        """
        reads file source and loads it as a module

        :param source: file to load
        :param module_name: name of module to register in sys.modules
        :return: loaded module
        """

        if module_name is None:
            module_name = self.gensym()

        spec = importlib.util.spec_from_file_location(module_name, source)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)

        return module

    def gensym(length=32, prefix="gensym_"):
        """
        generates a fairly unique symbol, used to make a module name,
        used as a helper function for load_module

        :return: generated symbol
        """
        alphabet = string.ascii_uppercase + string.ascii_lowercase + string.digits
        symbol = "".join([secrets.choice(alphabet) for i in range(length)])

        return prefix + symbol

    def load_map(self):
        self.map = self.map_plugin.load_map()

    def save_map(self):
        self.map_plugin.save_map()

    def get_area_by_name(self, name):
        return self.map_plugin.get_area_by_name(name)

    def get_area_list(self):
        areas = self.map_plugin.get_area_list()
        return areas

    def get_all_keepout_zones(self):
        return self.map_plugin.get_all_keepout_zones()

    def set_area(self, area):
        return self.map_plugin.set_area(area)


def main(args=None):
    map_prodiver = MapProvider("ros2mowerMapProvider", 
                               "ros2mower/ros2mower_map_provider/example/mow_area.yaml")
    print(map_prodiver.get_area_by_name('Greenhouse'))
    print(map_prodiver.get_area_list())
    # print(map_prodiver.get_all_keepout_zones())


if __name__ == '__main__':
    main()
