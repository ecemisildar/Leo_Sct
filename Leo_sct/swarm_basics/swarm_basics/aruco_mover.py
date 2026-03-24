import math
import shutil
import subprocess
import time

import rclpy
from rclpy.node import Node


class ArucoMover(Node):
    def __init__(self):
        super().__init__("aruco_mover")

        self.world_name = str(self.declare_parameter("world_name", "random_world").value)
        self.entity_name = str(self.declare_parameter("entity_name", "moving_aruco_box").value)
        self.center_x = float(self.declare_parameter("center_x", 2.0).value)
        self.center_y = float(self.declare_parameter("center_y", 0.0).value)
        self.z = float(self.declare_parameter("z", 0.375).value)
        self.radius = float(self.declare_parameter("radius", 4.0).value)
        self.angular_speed = float(self.declare_parameter("angular_speed", 0.35).value)
        self.update_rate_hz = float(self.declare_parameter("update_rate_hz", 2.0).value)
        self.face_motion = bool(self.declare_parameter("face_motion", True).value)

        self.service_name = f"/world/{self.world_name}/set_pose"
        self.started_at = time.time()
        self._last_failure_log = 0.0
        self._selected_backend = None
        self._pending_process = None

        self.backends = []
        gz_bin = shutil.which("gz")
        ign_bin = shutil.which("ign")
        if gz_bin:
            self.backends.append((gz_bin, "gz.msgs.Pose", "gz.msgs.Boolean"))
            self.backends.append((gz_bin, "ignition.msgs.Pose", "ignition.msgs.Boolean"))
        if ign_bin:
            self.backends.append((ign_bin, "ignition.msgs.Pose", "ignition.msgs.Boolean"))
            self.backends.append((ign_bin, "gz.msgs.Pose", "gz.msgs.Boolean"))

        period = 1.0 / max(0.1, self.update_rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

        if not self.backends:
            self.get_logger().warn("No Gazebo CLI found (`gz` or `ign`). ArUco mover is disabled.")
        else:
            self.get_logger().info(
                f"Moving ArUco target '{self.entity_name}' in world '{self.world_name}' via Gazebo service."
            )

    def _build_pose_request(self, x: float, y: float, yaw: float) -> str:
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        # Protobuf text format for ignition.msgs.Pose. Keep it comma-free.
        return (
            f'name: "{self.entity_name}" '
            f'position {{ x: {x:.6f} y: {y:.6f} z: {self.z:.6f} }} '
            f'orientation {{ x: 0.0 y: 0.0 z: {qz:.6f} w: {qw:.6f} }}'
        )

    def _invoke_backend(self, backend, req_text: str):
        binary, req_type, rep_type = backend
        cmd = [
            binary,
            "service",
            "-s",
            self.service_name,
            "--reqtype",
            req_type,
            "--reptype",
            rep_type,
            "--timeout",
            "1000",
            "--req",
            req_text,
        ]
        return subprocess.run(cmd, capture_output=True, text=True)

    def _square_pose(self, t: float):
        side_length = max(0.1, self.radius)
        half_side = 0.5 * side_length
        perimeter = 4.0 * side_length
        distance = (self.angular_speed * t) % perimeter

        if distance < side_length:
            x = self.center_x - half_side + distance
            y = self.center_y - half_side
        elif distance < 2.0 * side_length:
            x = self.center_x + half_side
            y = self.center_y - half_side + (distance - side_length)
        elif distance < 3.0 * side_length:
            x = self.center_x + half_side - (distance - 2.0 * side_length)
            y = self.center_y + half_side
        else:
            x = self.center_x - half_side
            y = self.center_y + half_side - (distance - 3.0 * side_length)

        return x, y

    def _on_timer(self):
        if not self.backends:
            return

        t = time.time() - self.started_at
        x, y = self._square_pose(t)
        yaw = 0.0
        req_text = self._build_pose_request(x, y, yaw)

        candidates = (
            [self._selected_backend]
            if self._selected_backend is not None
            else self.backends
        )

        last_error = ""
        for backend in candidates:
            result = self._invoke_backend(backend, req_text)
            if result.returncode == 0:
                self._selected_backend = backend
                return
            last_error = (result.stderr or result.stdout).strip()

        now = time.time()
        if now - self._last_failure_log > 2.0:
            self.get_logger().warn(
                f"Failed to move '{self.entity_name}' via {self.service_name}: {last_error}"
            )
            self._last_failure_log = now


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
