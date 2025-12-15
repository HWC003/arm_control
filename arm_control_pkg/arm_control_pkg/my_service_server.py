import rclpy
from rclpy.node import Node
from my_ros_interfaces.srv import MyCustomService
from geometry_msgs.msg import Pose
from rclpy.task import Future

class MyCustomServiceServer(Node):
    def __init__(self):
        super().__init__('my_custom_service_server')
        self.srv = self.create_service(MyCustomService, 'my_custom_service', 
                                       self.handle_service)
        self.get_logger().info('My Custom Service Server is ready.')

    def handle_service(self, request: MyCustomService.Request, 
                       response: MyCustomService.Response) -> MyCustomService.Response:
        # Log the received pose
        self.get_logger().info(f'Received pose: position=({request.pose.position.x}, '
                               f'{request.pose.position.y}, {request.pose.position.z}), '
                               f'orientation=({request.pose.orientation.x}, {request.pose.orientation.y}, '
                               f'{request.pose.orientation.z}, {request.pose.orientation.w})')
        
        response.success = True  # Indicate that the service was handled successfully
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = MyCustomServiceServer()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()