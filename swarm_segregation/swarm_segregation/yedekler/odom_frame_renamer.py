#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pathlib import PurePosixPath

class OdomFrameRenamer(Node):
    """
    Auto-discovers all Odometry topics that end with '/odom',
    and republishes them to '<topic><suffix>' with header.frame_id set to '<ns>/odom'.
    Example:
      In:  /robot1/odom (frame_id 'odom')
      Out: /robot1/odom_reframed (frame_id 'robot1/odom')
    """

    def __init__(self):
        super().__init__('odom_frame_renamer')

        # Parameter: suffix for output topics
        self.declare_parameter('output_suffix', '_reframed')
        self.output_suffix = self.get_parameter('output_suffix').get_parameter_value().string_value or '_reframed'

        # (Optional) also rewrite child_frame_id to '<ns>/base_link'
        self.declare_parameter('rewrite_child_frame_id', False)
        self.rewrite_child = self.get_parameter('rewrite_child_frame_id').get_parameter_value().bool_value

        # Track created subs/pubs so we don't duplicate
        self._subs = {}
        self._pubs = {}

        # Periodically scan the graph for new odom topics
        self._scan_timer = self.create_timer(2.0, self._scan_topics)
        self.get_logger().info(f"Running. Output suffix: '{self.output_suffix}', rewrite_child_frame_id={self.rewrite_child}")

    def _scan_topics(self):
        topics = self.get_topic_names_and_types()
        for topic_name, types in topics:
            # Only handle Odometry topics whose name ends with '/odom'
            if 'nav_msgs/msg/Odometry' not in types:
                continue
            if not topic_name.endswith('/odom'):
                continue
            if topic_name in self._subs:
                continue  # already wired up

            out_topic = topic_name + self.output_suffix

            # Create publisher for the out topic
            pub = self.create_publisher(Odometry, out_topic, 10)
            self._pubs[topic_name] = pub

            # Create a subscription with a closure capturing topic_name
            sub = self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, src=topic_name: self._odom_cb(msg, src),
                10
            )
            self._subs[topic_name] = sub

            self.get_logger().info(f"Reframing '{topic_name}' -> '{out_topic}'")

    def _odom_cb(self, msg: Odometry, src_topic: str):
        # Derive namespace from topic path (everything before the final 'odom')
        # '/robot1/odom' -> 'robot1'; '/robots/r2/odom' -> 'robots/r2'; '/odom' -> ''
        p = PurePosixPath(src_topic)
        parts = list(p.parts)  # e.g. ['/', 'robot1', 'odom']
        ns = '/'.join(parts[1:-1]) if len(parts) > 2 else ''  # skip leading '/'

        # Build new frame_id
        new_frame = f"{ns}/odom" if ns else "odom"

        # Mutate a copy-ish (Odometry is mutable; we just reassign the header field)
        msg.header.frame_id = new_frame

        # Optionally rewrite child_frame_id to '<ns>/base_link' (common convention)
        if self.rewrite_child:
            child = f"{ns}/base_link" if ns else "base_link"
            msg.child_frame_id = child

        # Publish on the paired out-topic
        pub = self._pubs.get(src_topic)
        if pub is not None:
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFrameRenamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
