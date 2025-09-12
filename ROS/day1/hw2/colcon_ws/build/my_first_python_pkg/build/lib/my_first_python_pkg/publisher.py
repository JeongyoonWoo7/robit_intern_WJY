import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32


class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        
        self.publisher_str = self.create_publisher(String, 'topic_str', 10)
        self.publisher_int = self.create_publisher(Int32, 'topic_int', 10)
        self.publisher_float = self.create_publisher(Float32, 'topic_float', 10)
        
        timer_period = 1.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg_str = String()
        msg_str.data = 'Hello World from PY: {0}'.format(self.i)
        
        msg_int = Int32()
        msg_int.data = self.i
        
        msg_float = Float32()
        msg_float.data = float(self.i) * 0.1
        
        self.publisher_str.publish(msg_str)
        self.publisher_int.publish(msg_int)
        self.publisher_float.publish(msg_float)
        
        
        self.get_logger().info(
            'Published -> str: "{0}", int: {1}, float: {2:.2f}'
            .format(msg_str.data, msg_int.data, msg_float.data)
        )
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
