import rclpy
from rclpy.node import Node
from my_ros_python_pkg.my_python_library.sample_class import SampleClass, add_two_things

class MyPythonNode(Node):
    def __init__(self):
        super().__init__('my_python_node')
        self.get_logger().info('My Python Node has been started.')
        
        # Declare parameters with defaults
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('greeting', 'hello')

        rate = float(self.get_parameter('rate_hz').value)
        self.greeting = str(self.get_parameter('greeting').value)
        self.get_logger().info(f'Parameters - rate_hz: {rate}, greeting: {self.greeting}')

        # Using SampleClass
        sample = SampleClass("User")
        sample.welcome_print()

        # Initial values
        self.a = 5
        self.b = 10
        self.result = add_two_things(self.a, self.b)
        self.get_logger().info(f'Initial result: {self.result}')

        # Create a timer to call self.update_result once per second
        period = 1.0 / rate if rate > 0 else 1.0
        self.timer = self.create_timer(period, self.update_result)

    def update_result(self):
        # Increment one of the operands (say b)
        self.b += 1
        self.result = add_two_things(self.a, self.b)
        self.get_logger().info(f'{self.greeting}: {self.result}')

def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
