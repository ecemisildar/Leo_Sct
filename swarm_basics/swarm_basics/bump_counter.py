#!/usr/bin/env python3
import os, csv, math, time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import UInt32, String
from ros_gz_interfaces.msg import Contacts

# --- tune here if you like ---
COOLDOWN_SEC = 0.5          # must be separated this long to count a new bump
PRUNE_HZ = 10.0             # prune inactive contacts at this rate
CONTACT_TOPIC = 'contact'   # will be remapped in launch per robot
LOG_DIR = Path.home() / 'ros_bump_logs'  # per-robot CSVs land here
# --------------------------------

def entity_name(ent) -> str:
    # Contacts.collision{1,2} are Entity objects with .name
    try:
        return ent.name
    except AttributeError:
        return str(ent)

def mean_xyz(pts):
    if not pts:
        return (math.nan, math.nan, math.nan)
    sx = sum(getattr(p, 'x', 0.0) for p in pts)
    sy = sum(getattr(p, 'y', 0.0) for p in pts)
    sz = sum(getattr(p, 'z', 0.0) for p in pts)
    n = float(len(pts))
    return (sx/n, sy/n, sz/n)

class BumpLogger(Node):
    def __init__(self):
        super().__init__('bump_counter')
        # identity
        self.ns = self.get_namespace().strip('/') or 'root'
        self.cooldown = Duration(seconds=COOLDOWN_SEC)

        # state
        self.bump_count = 0
        self.active = {}   # other_collision_name -> last_seen (rclpy.Time)

        # pubs/subs
        self.sub = self.create_subscription(Contacts, CONTACT_TOPIC, self.on_contacts, 10)
        self.pub_count = self.create_publisher(UInt32, 'bump_count', 10)
        self.pub_last  = self.create_publisher(String, 'last_bump_with', 10)
        self.create_timer(1.0/PRUNE_HZ, self.prune)

        # logging
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.csv_path = self._csv_path_for_today()
        self._ensure_csv_header()

        self.get_logger().info(
            f"[{self.ns}] listening on '{CONTACT_TOPIC}' (remap this). "
            f"Logging to {self.csv_path}"
        )

    # ---------- CSV helpers ----------
    def _csv_path_for_today(self) -> Path:
        datestr = time.strftime('%Y-%m-%d')
        fname = f"bumps_{self.ns}_{datestr}.csv"
        return LOG_DIR / fname

    def _ensure_csv_header(self):
        if not self.csv_path.exists():
            with self.csv_path.open('w', newline='') as f:
                w = csv.writer(f)
                w.writerow([
                    'stamp_sec', 'stamp_nsec',
                    'robot_ns', 'bump_index',
                    'self_collision', 'other_collision',
                    'avg_contact_x', 'avg_contact_y', 'avg_contact_z'
                ])

    def _append_csv(self, stamp_sec, stamp_nsec, self_col, other_col, avg_xyz):
        with self.csv_path.open('a', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                int(stamp_sec), int(stamp_nsec),
                self.ns, self.bump_count,
                self_col, other_col,
                *[f"{v:.6f}" if isinstance(v, float) else "" for v in avg_xyz]
            ])
    # ----------------------------------

    def on_contacts(self, msg: Contacts):
        now = self.get_clock().now()
        for c in msg.contacts:
            col1 = entity_name(c.collision1)
            col2 = entity_name(c.collision2)

            # Heuristic: the side that contains this robot's namespace is "me"
            is1_me = self.ns in col1
            is2_me = self.ns in col2
            if is1_me == is2_me:
                # ambiguous or unrelated to me -> skip
                continue

            self_col = col1 if is1_me else col2
            other_col = col2 if is1_me else col1

            # New bump with this "other"?
            if other_col not in self.active:
                self.bump_count += 1
                self.active[other_col] = now
                self.pub_count.publish(UInt32(data=self.bump_count))
                self.pub_last.publish(String(data=other_col))

                avg_xyz = mean_xyz(c.positions)
                stamp = msg.header.stamp
                self._append_csv(stamp.sec, stamp.nanosec, self_col, other_col, avg_xyz)

                self.get_logger().info(
                    f"[{self.ns}] bump #{self.bump_count} with: {other_col}"
                )
            else:
                # refresh last seen
                self.active[other_col] = now

    def prune(self):
        now = self.get_clock().now()
        to_drop = [k for k, t in self.active.items() if (now - t) > self.cooldown]
        for k in to_drop:
            del self.active[k]

def main():
    rclpy.init()
    node = BumpLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
