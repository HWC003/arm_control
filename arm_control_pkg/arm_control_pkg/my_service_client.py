import sys
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Pose
from my_ros_interfaces.srv import MyCustomService

class MyCustomServiceClient(Node):
    def __init__(self):
        super().__init__('my_custom_service_client')
        self.cli = self.create_client(MyCustomService, 'my_custom_service')
        self.get_logger().info('Waiting for /my_custom_service...')
        self.cli.wait_for_service()

    def send_request(self, pose: Pose) -> Future:
        req = MyCustomService.Request()
        req.pose = pose
        return self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomServiceClient()

    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0


    future = node.send_request(pose)
    node.get_logger().info('Request sent, waiting for response...')

    rclpy.spin_until_future_complete(node, future)
    if future.result().success:
        node.get_logger().info(f'Service returned successfully')
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
