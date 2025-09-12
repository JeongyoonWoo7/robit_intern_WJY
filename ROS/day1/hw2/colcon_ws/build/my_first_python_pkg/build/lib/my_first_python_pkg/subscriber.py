import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

     
        self.subscription_str = self.create_subscription(
            String,
            'topic_str',
            self.listener_callback_str,
            10)

        self.subscription_int = self.create_subscription(
            Int32,
            'topic_int',
            self.listener_callback_int,
            10)

        self.subscription_float = self.create_subscription(
            Float32,
            'topic_float',
            self.listener_callback_float,
            10)

    def listener_callback_str(self, msg: String):
        self.get_logger().info(f'I heard string: "{msg.data}"')

    def listener_callback_int(self, msg: Int32):
        self.get_logger().info(f'I heard int: {msg.data}')

    def listener_callback_float(self, msg: Float32):
        self.get_logger().info(f'I heard float: {msg.data:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.get_logger().info('Keyboard Interrupt (SIGINT)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

