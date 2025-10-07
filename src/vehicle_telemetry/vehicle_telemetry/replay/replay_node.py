from __future__ import annotations

import signal
import subprocess
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class TelemetryReplayNode(Node):
    """Node that wraps ros2 bag play for deterministic telemetry replay."""

    def __init__(self) -> None:
        super().__init__("telemetry_replay")
        self.declare_parameter("bag_path", "")
        self.declare_parameter("loop", True)
        self.declare_parameter("rate", 1.0)
        self.declare_parameter("autoplay", False)
        self.declare_parameter("wait_for_topics", False)

        self._process: Optional[subprocess.Popen] = None
        self._bag_path = self._resolve_bag_path(self.get_parameter("bag_path").value)

        self._play_srv = self.create_service(Trigger, "play", self._handle_play_service)
        self._stop_srv = self.create_service(Trigger, "stop", self._handle_stop_service)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        if self.get_parameter("autoplay").value and self._bag_path:
            self.start_replay()

    def _resolve_bag_path(self, path: str) -> Optional[Path]:
        if not path:
            return None
        candidate = Path(path)
        if candidate.is_dir():
            return candidate
        if candidate.is_file():
            return candidate
        self.get_logger().warning(f"Provided bag path '{path}' does not exist.")
        return None

    def _on_set_parameters(self, params):
        from rclpy.parameter import SetParametersResult

        for param in params:
            if param.name == "bag_path":
                path = self._resolve_bag_path(param.value)
                if path:
                    self._bag_path = path
                else:
                    return SetParametersResult(successful=False, reason="Invalid bag path")
        return SetParametersResult(successful=True)

    def _handle_play_service(self, _request, response):
        if self._process and self._process.poll() is None:
            response.success = False
            response.message = "Replay already running."
            return response

        if not self._bag_path:
            response.success = False
            response.message = "No rosbag path configured."
            return response

        if self.start_replay():
            response.success = True
            response.message = "Replay started."
        else:
            response.success = False
            response.message = "Failed to start replay."
        return response

    def _handle_stop_service(self, _request, response):
        if not self._process or self._process.poll() is not None:
            response.success = False
            response.message = "Replay not running."
            return response
        self.stop_replay()
        response.success = True
        response.message = "Replay stopped."
        return response

    def start_replay(self) -> bool:
        if not self._bag_path:
            self.get_logger().error("Cannot start replay; no bag path configured.")
            return False
        command = [
            "ros2",
            "bag",
            "play",
            str(self._bag_path),
            "--rate",
            str(self.get_parameter("rate").value),
        ]
        if self.get_parameter("loop").value:
            command.append("--loop")
        if self.get_parameter("wait_for_topics").value:
            command.append("--wait-for-all")

        self.get_logger().info(f"Starting rosbag replay: {' '.join(command)}")
        try:
            self._process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
            return True
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Failed to start replay: {exc}")
            self._process = None
            return False

    def stop_replay(self) -> None:
        if not self._process or self._process.poll() is not None:
            self._process = None
            return

        self.get_logger().info("Stopping rosbag replay…")
        try:
            self._process.send_signal(signal.SIGINT)
            self._process.wait(timeout=5)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f"Error stopping replay gracefully ({exc}); killing process.")
            self._process.kill()
        finally:
            self._process = None

    def destroy_node(self) -> bool:
        self.stop_replay()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Telemetry replay interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
