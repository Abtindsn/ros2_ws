from __future__ import annotations

import atexit
import os
import signal
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, TextIO
from types import FrameType

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from vehicle_telemetry.utils.config_loader import RecorderConfig, load_recorder_config, load_topics_config


class TelemetryRecorderNode(Node):
    """Node that manages rosbag2 recordings for telemetry topics."""

    def __init__(self) -> None:
        super().__init__("telemetry_recorder")
        self.declare_parameter("recorder_config_path", "")
        self.declare_parameter("topics_config_path", "")
        self.declare_parameter("autostart", True)
        self._process: Optional[subprocess.Popen] = None
        self._active_bag_path: Optional[Path] = None
        self._log_file: Optional[TextIO] = None
        self._recorder_config = self._load_config()
        self._topics = self._load_topics()
        self._previous_signal_handlers: Dict[int, object] = {}
        self._signal_handlers_installed = False

        self._start_srv = self.create_service(Trigger, "start", self._handle_start_service)
        self._stop_srv = self.create_service(Trigger, "stop", self._handle_stop_service)

        self._install_signal_handlers()

        if self.get_parameter("autostart").value:
            self.start_recording()

    def _load_config(self) -> RecorderConfig:
        path = self.get_parameter("recorder_config_path").value
        if path:
            return load_recorder_config(path)
        return load_recorder_config(str(self.package_share_path() / "config" / "recorder.yaml"))

    def _load_topics(self):
        path = self.get_parameter("topics_config_path").value
        if not path:
            return {}
        return load_topics_config(path)

    @staticmethod
    def package_share_path() -> Path:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("vehicle_telemetry"))

    def _handle_start_service(self, _request, response):
        if self._process and self._process.poll() is None:
            response.success = False
            response.message = "Recorder already running."
            return response
        if self.start_recording():
            response.success = True
            response.message = f"Recorder started at {self._active_bag_path}"
        else:
            response.success = False
            response.message = "Failed to start recorder."
        return response

    def _handle_stop_service(self, _request, response):
        if not self._process or self._process.poll() is not None:
            response.success = False
            response.message = "Recorder is not running."
            return response
        self.stop_recording()
        response.success = True
        response.message = "Recorder stopped."
        return response

    def start_recording(self) -> bool:
        if self._process and self._process.poll() is None:
            self.get_logger().info("Recorder already running.")
            return True

        timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        output_dir = self._recorder_config.output_directory / timestamp
        output_dir.mkdir(parents=True, exist_ok=True)
        bag_prefix = output_dir / "telemetry"

        command = [
            "ros2",
            "bag",
            "record",
            "-o",
            str(bag_prefix),
            "--storage",
            self._recorder_config.storage_id,
        ]

        if self._recorder_config.compression_enabled and self._recorder_config.compression_format:
            command.extend(
                [
                    "--compression-mode",
                    "file",
                    "--compression-format",
                    self._recorder_config.compression_format,
                ]
            )

        if self._recorder_config.max_bag_size_mb:
            size_bytes = int(self._recorder_config.max_bag_size_mb * 1024 * 1024)
            command.extend(["--max-bag-size", str(size_bytes)])
        if self._recorder_config.max_bag_duration_s:
            command.extend(["--max-bag-duration", str(self._recorder_config.max_bag_duration_s)])

        # Ensure parent directory exists before launching recorder; rosbag2 creates files using the prefix
        try:
            Path(bag_prefix).parent.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            self.get_logger().error(f"Failed to prepare bag parent for {bag_prefix}: {exc}")
            return False

        qos_overrides = self.package_share_path() / "config" / "rosbag_qos.yaml"
        if qos_overrides.is_file():
            command.extend(["--qos-profile-overrides-path", str(qos_overrides)])
        else:
            self.get_logger().warning(f"QoS overrides file missing at {qos_overrides}; recorder will use defaults.")

        topics = self._determine_topics()
        if not topics:
            command.append("-a")
        else:
            command.extend(topics)

        self.get_logger().info(f"Recording to directory: {bag_prefix}")
        self.get_logger().info(f"Starting rosbag2 recorder: {' '.join(command)}")
        try:
            log_path = output_dir / "recorder.log"
            self._log_file = open(log_path, "a", buffering=1)  # line-buffered text log
            self._process = subprocess.Popen(
                command,
                preexec_fn=os.setsid,
                stdout=self._log_file,
                stderr=subprocess.STDOUT,
            )
            self._active_bag_path = output_dir
            return True
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Failed to start recorder: {exc}")
            if self._log_file:
                try:
                    self._log_file.close()
                except Exception:  # pragma: no cover - best effort cleanup
                    pass
                finally:
                    self._log_file = None
            return False

    def _determine_topics(self):
        if self._recorder_config.all_topics:
            return []
        if self._recorder_config.include_topics:
            return self._recorder_config.include_topics
        return [topic.name for topic in self._topics.values()]

    def stop_recording(self) -> None:
        self._graceful_stop()

    def _install_signal_handlers(self) -> None:
        if self._signal_handlers_installed:
            return
        atexit.register(self._graceful_stop)
        for sig in (signal.SIGINT, signal.SIGTERM):
            self._previous_signal_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, self._on_signal)
        self._signal_handlers_installed = True

    def _close_log_file(self) -> None:
        if not self._log_file:
            return
        try:
            if not self._log_file.closed:
                self._log_file.flush()
            self._log_file.close()
        except Exception:  # pragma: no cover - logging close best effort
            pass
        finally:
            self._log_file = None

    def _on_signal(self, signum: int, frame: Optional[FrameType]) -> None:
        self.get_logger().info(f"Telemetry recorder received signal {signum}; shutting down rosbag2.")
        self._graceful_stop()
        if rclpy.ok():
            rclpy.shutdown()
        previous = self._previous_signal_handlers.get(signum)
        if callable(previous):
            previous(signum, frame)
        elif previous == signal.SIG_DFL:
            if signum == signal.SIGINT:
                signal.default_int_handler(signum, frame)
            else:
                raise SystemExit(0)
        elif previous == signal.SIG_IGN:
            return

    def _graceful_stop(self) -> None:
        if not self._process:
            return
        if self._process.poll() is not None:
            self._process = None
            self._close_log_file()
            return

        self.get_logger().info("Stopping rosbag2 recorder…")
        try:
            os.killpg(self._process.pid, signal.SIGINT)
            self._process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.get_logger().warning("Recorder did not stop after SIGINT; sending SIGKILL.")
            os.killpg(self._process.pid, signal.SIGKILL)
            self._process.wait(timeout=5)
        except ProcessLookupError:
            self.get_logger().debug("Recorder process group already gone.")
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Error stopping recorder: {exc}")
        finally:
            self._process = None
            self._active_bag_path = None
            self._close_log_file()

    def destroy_node(self) -> bool:
        self.stop_recording()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Telemetry recorder interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
