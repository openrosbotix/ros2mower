import yaml

class test_yaml():
    def __init__(self):
        
        self.map_file = 'mow_area.yaml'
        self.map_result = 'mow_area_result.yaml'
        self.map = None

        self.load_map(self.map_file)
        self.save_map(self.map_result)
        
        self.load_map(self.map_result)
        self.save_map('mow_area_result1.yaml')
                    
    def load_map(self, path):
    # open map file and store it to local map object
        try:
            f = open(path, 'r')
            try:
                self.map = yaml.safe_load(f)
            finally:
                f.close()
        except (IOError, OSError) as e:
            print(e)
            
    def save_map(self, result):
        with open(result, 'w') as file:
            yaml.safe_dump(self.map, file, sort_keys=False)
            
if __name__ == '__main__':
    test_yaml()
            