import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'state_topic',
            self.listener_callback,
            10)
        # self.subscription = self.create_subscription(
        #     String,
        #     'wheel_distance',
        #     self.listener_callback,
        #     10)
        # self.subscription = self.create_subscription(
        #     String,
        #     '/mod_lidar',
        #     self.listener_callback,
        #     10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

