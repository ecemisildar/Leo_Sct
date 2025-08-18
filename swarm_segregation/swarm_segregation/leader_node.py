import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')

        self.declare_parameter('color', 'red')
        self.color = self.get_parameter('color').get_parameter_value().string_value

        self.topic_name = f'/leader_broadcast'
        self.publisher = self.create_publisher(String, self.topic_name, 10)

        self.timer = self.create_timer(0.2, self.broadcast_color)
        self.get_logger().info(f"[{self.get_namespace()}] Publishing as leader of color: {self.color}")

    def broadcast_color(self):
        msg = String()
        msg.data = self.color
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
