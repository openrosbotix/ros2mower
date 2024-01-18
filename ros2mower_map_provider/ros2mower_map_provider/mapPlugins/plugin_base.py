# Python program showing
# abstract base class work
from abc import ABC, abstractmethod


class MapProviderBase(ABC):

    @abstractmethod
    def __init__(self, map_file):
        pass

    @abstractmethod
    def load_map(self):
        pass

    @abstractmethod
    def save_map(self):
        pass

    @abstractmethod
    def get_area(self, area_index):
        pass

    @abstractmethod
    def get_num_of_areas(self):
        pass

    @abstractmethod
    def get_area_by_name(self, name):
        pass

    @abstractmethod
    def get_all_keepout_zones(self):
        pass

    @abstractmethod
    def set_area(self, area):
        pass
