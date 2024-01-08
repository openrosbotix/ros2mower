import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts

class MapProvider(Node):
    def __init__(self):
        super().__init__('map_provider')
        self.declare_parameter('map_file_name', 'my_map.yaml' )
        
        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    
    def load_map(self):
        a = 1
        #load map from file
        #store map in local object

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response
    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MapProvider()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    