import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml
from swarm_basics.sct import SCT

class RobotMotionController(Node):
    def __init__(self):
        super().__init__('robot_motion_controller')
        self.get_logger().info('Robot Motion Controller node started.')

        # --- Load YAML configuration ---
        config_path = os.path.join(
            get_package_share_directory('swarm_basics'),
            'config',
            'supervisor.yaml'
        )
        try:
            with open(config_path, 'r') as stream:
                robot_config = yaml.safe_load(stream)
            self.sct = SCT(config_path)
            self.action_map = robot_config.get('action_map', {})
            self.sensor_map = robot_config.get('sensor_map', {})
        except FileNotFoundError:
            self.get_logger().error(f'YAML file not found at: {config_path}. Exiting.')
            rclpy.shutdown()
            return

        # --- Robot color ---
        self.declare_parameter('color', 'red')
        raw_color = self.get_parameter('color').get_parameter_value().string_value
        code_to_color = {
            "1 0 0 1": "red",
            "0 1 0 1": "green",
            "0 0 1 1": "blue",
            "1 1 0 1": "yellow",
        }
        self.robot_color = code_to_color.get(raw_color, raw_color)
        self.get_logger().info(f"Robot Color: {self.robot_color}")

        # --- Publishers / Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher = None
        self.color_subscribers = {}
        self.leader_signals = {}

        leader_colors = ['red', 'green', 'blue']
        if self.robot_color in leader_colors:
            # Leaders publish their own color
            self.publisher = self.create_publisher(
                String,
                f"/leader_broadcast/{self.robot_color}",
                10
            )
            self.get_logger().info(f"{self.robot_color} leader will publish signals.")
        else:
            # Followers subscribe to all leader colors
            for c in leader_colors:
                topic_name = f"/leader_broadcast/{c}"
                self.color_subscribers[c] = self.create_subscription(
                    String,
                    topic_name,
                    lambda msg, c=c: self.color_callback(msg, c),
                    10
                )
            self.get_logger().info(f"{self.robot_color} follower will subscribe to leader signals.")

        # --- Setup SCT callbacks ---
        self.setup_callbacks()

        # --- Timer to run SCT ---
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f"Registered callbacks: {list(self.sct.callback.keys())}")

    # ------------------------
    # Motion callbacks
    # ------------------------
    def move_forward(self, sup_data):
        twist = Twist()
        twist.linear.x = 0.2
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"{self.robot_color} Executing: Move Forward")

    def stop_movement(self, sup_data):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"{self.robot_color} Executing: Stop Movement")

    def turn_clockwise(self, sup_data):
        twist = Twist()
        twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"{self.robot_color} Executing: Turn Clockwise")

    def turn_counter_clockwise(self, sup_data):
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"{self.robot_color} Executing: Turn Counter-Clockwise")

    # ------------------------
    # Send signal
    # ------------------------
    def send_signal(self, sup_data):
        if not self.publisher:
            return
        msg = String()
        msg.data = self.robot_color
        self.publisher.publish(msg)
        # self.get_logger().info(f"{self.robot_color} sending {self.robot_color.upper()} signal")

    # ------------------------
    # SCT callback setup
    # ------------------------
    def setup_callbacks(self):
        action_functions = {
            'move_forward': self.move_forward,
            'turn_clockwise': self.turn_clockwise,
            'turn_counter_clockwise': self.turn_counter_clockwise,
            'stop': self.stop_movement,
            # 'send_red': self.send_signal,
            # 'send_green': self.send_signal,
            # 'send_blue': self.send_signal,
            # 'send_nothing': self.send_signal,
        }

        for action_type, event_name in self.action_map.items():
            func = action_functions.get(action_type)
            event_id = self.sct.EV.get(event_name)
            if func and event_id is not None:
                self.sct.add_callback(event_id, func, lambda sup_data: True, None)

        # Register sensor events
        for sensor_type, event_name in self.sensor_map.items():
            event_id = self.sct.EV.get(event_name)
            if event_id is not None:
                self.sct.add_callback(
                    event_id,
                    lambda sup_data: None,
                    self.check_sensor_input,
                    {'sensor_type': sensor_type}
                )

    # ------------------------
    # Color callback for followers
    # ------------------------
    def color_callback(self, msg, color):
        self.leader_signals[color] = msg.data
        active_colors = {c for c, v in self.leader_signals.items() if v != "none"}
        # self.get_logger().info(f"{self.robot_color} received {len(active_colors)} colors: {active_colors}")

    # ------------------------
    # Sensor input check
    # ------------------------
    def check_sensor_input(self, sup_data):
        sensor_type = sup_data['sensor_type']
        if not hasattr(self, "leader_signals"):
            return False

        active_colors = {c for c, v in self.leader_signals.items() if v != "none"}

        # self.get_logger().info(
        #     f"[{self.robot_color}] check_sensor_input called for '{sensor_type}' "
        #     f"with active_colors: {active_colors}"
        # )

        if sensor_type.startswith("get_signal_"):
            target = sensor_type[len("get_signal_"):]  # e.g., "R", "not_R"
            if target.startswith("not_"):
                excluded_color = self.code_to_color_name(target[len("not_"):])
                result = any(color != excluded_color for color in active_colors)
                # self.get_logger().info(f"Sensor: {sensor_type}, excluded: {excluded_color}, result: {result}")
            else:
                target_color = self.code_to_color_name(target)
                result = target_color in active_colors
                # self.get_logger().info(f"Sensor: {sensor_type}, target: {target_color}, result: {result}")

            return result
        return False

    def code_to_color_name(self, code):
        mapping = {'r': 'red', 'g': 'green', 'b': 'blue'}
        return mapping.get(code.lower(), code.lower())

    # ------------------------
    # SCT main loop
    # ------------------------
    def timer_callback(self):
        
        self.sct.run_step()

def main(args=None):
    rclpy.init(args=args)
    node = RobotMotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
