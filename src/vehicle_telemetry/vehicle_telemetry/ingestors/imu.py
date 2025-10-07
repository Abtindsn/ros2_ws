from __future__ import annotations

import math
import threading
from typing import Optional

from rclpy.node import Node
from sensor_msgs.msg import Imu

from vehicle_telemetry.ingestors.base import TelemetryIngestor

try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    serial = None


class IMUIngestor(TelemetryIngestor):
    """IMU ingestor reading from a serial connection."""

    def __init__(self, node: Node, topic, use_simulated: bool) -> None:
        super().__init__(node, topic, use_simulated)
        self._port = node.declare_parameter("imu.port", "/dev/ttyUSB0").value
        self._baudrate = node.declare_parameter("imu.baudrate", 115200).value
        self._frame_id = node.declare_parameter("imu.frame_id", "imu_link").value
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._serial: Optional["serial.Serial"] = None
        self._time = 0.0

        if not self._use_simulated and serial:
            try:
                self._serial = serial.Serial(self._port, self._baudrate, timeout=0.1)
                self._node.get_logger().info(f"IMU serial connected on {self._port}")
            except Exception as exc:  # pylint: disable=broad-except
                self._node.get_logger().warning(
                    f"Failed to open IMU serial port ({exc}); enabling simulated IMU data."
                )
                self._use_simulated = True
        elif not serial:
            self._node.get_logger().warning("pyserial not installed; IMU ingestor running in simulation mode.")
            self._use_simulated = True

    def start(self) -> None:
        if self._use_simulated or not self._serial:
            self._node.create_timer(0.01, self._publish_simulated)
            return

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self._node.get_logger().info("Started IMU serial reader thread.")

    def _read_loop(self) -> None:
        assert self._serial is not None
        while not self._stop_event.is_set():
            try:
                raw = self._serial.readline().decode("utf-8").strip()
                if not raw:
                    continue
                parts = [float(x) for x in raw.split(",")]
                if len(parts) < 10:
                    continue
                msg = Imu()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.header.frame_id = self._frame_id
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = parts[0:3]
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = parts[3:6]
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = parts[6:10]
                self._publisher.publish(msg)
            except Exception as exc:  # pylint: disable=broad-except
                self._node.get_logger().error(f"IMU read error: {exc}", throttle_duration_sec=5.0)

    def _publish_simulated(self) -> None:
        self._time += 0.01
        msg = Imu()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.linear_acceleration.x = math.sin(self._time) * 0.2
        msg.linear_acceleration.y = math.cos(self._time) * 0.2
        msg.linear_acceleration.z = 9.81
        msg.angular_velocity.x = math.sin(self._time) * 0.1
        msg.angular_velocity.y = math.cos(self._time) * 0.1
        msg.angular_velocity.z = math.sin(self._time * 0.5) * 0.2
        msg.orientation.w = 1.0
        self._publisher.publish(msg)

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().shutdown()
