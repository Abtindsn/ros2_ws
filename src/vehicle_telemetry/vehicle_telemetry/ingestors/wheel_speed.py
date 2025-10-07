from __future__ import annotations

import math
import threading
from typing import Optional

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from vehicle_telemetry.ingestors.base import TelemetryIngestor

try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    serial = None


class WheelSpeedIngestor(TelemetryIngestor):
    """Wheel speed ingestor reading from serial or UDP."""

    def __init__(self, node: Node, topic, use_simulated: bool) -> None:
        super().__init__(node, topic, use_simulated)
        self._port = node.declare_parameter("wheel_speed.port", "/dev/ttyUSB2").value
        self._baudrate = node.declare_parameter("wheel_speed.baudrate", 115200).value
        self._serial: Optional["serial.Serial"] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._time = 0.0

        if not self._use_simulated and serial:
            try:
                self._serial = serial.Serial(self._port, self._baudrate, timeout=0.1)
                self._node.get_logger().info(f"Wheel speed serial connected on {self._port}")
            except Exception as exc:  # pylint: disable=broad-except
                self._node.get_logger().warning(
                    f"Failed to open wheel speed serial port ({exc}); enabling simulated wheel speeds."
                )
                self._use_simulated = True
        elif not serial:
            self._node.get_logger().warning("pyserial not installed; wheel speed ingestor running in simulation mode.")
            self._use_simulated = True

    def start(self) -> None:
        if self._use_simulated or not self._serial:
            self._node.create_timer(0.05, self._publish_simulated)
            return
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self._node.get_logger().info("Started wheel speed reader thread.")

    def _read_loop(self) -> None:
        assert self._serial is not None
        while not self._stop_event.is_set():
            try:
                raw = self._serial.readline().decode("utf-8").strip()
                if not raw:
                    continue
                parts = [float(x) for x in raw.split(",")]
                if len(parts) != 4:
                    continue
                msg = Float32MultiArray()
                msg.data = parts
                self._publisher.publish(msg)
            except Exception as exc:  # pylint: disable=broad-except
                self._node.get_logger().error(f"Wheel speed read error: {exc}", throttle_duration_sec=5.0)

    def _publish_simulated(self) -> None:
        self._time += 0.05
        base_speed = 20.0  # m/s
        msg = Float32MultiArray()
        msg.data = [
            base_speed + math.sin(self._time) * 0.5,
            base_speed + math.sin(self._time + 0.2) * 0.5,
            base_speed + math.sin(self._time + 0.4) * 0.5,
            base_speed + math.sin(self._time + 0.6) * 0.5,
        ]
        self._publisher.publish(msg)

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().shutdown()
