from __future__ import annotations

import random
import threading
from typing import Optional

from rclpy.node import Node

from can_msgs.msg import Frame  # type: ignore

from vehicle_telemetry.ingestors.base import TelemetryIngestor

try:
    import can  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    can = None


class CANIngestor(TelemetryIngestor):
    """CAN bus ingestor that reads frames and republishes as ROS2 messages."""

    def __init__(self, node: Node, topic, use_simulated: bool) -> None:
        super().__init__(node, topic, use_simulated)
        self._channel = node.declare_parameter("can.channel", "can0").value
        self._bitrate = node.declare_parameter("can.bitrate", 500000).value
        self._poll_interval = node.declare_parameter("can.poll_interval", 0.005).value
        self._bus: Optional["can.Bus"] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        if not self._use_simulated and can:
            try:
                self._bus = can.Bus(channel=self._channel, bitrate=self._bitrate, bustype="socketcan")
                self._node.get_logger().info(f"CAN bus initialized on channel {self._channel}")
            except Exception as exc:  # pylint: disable=broad-except
                self._node.get_logger().warning(
                    f"Failed to initialize CAN interface ({exc}); falling back to simulated CAN frames."
                )
                self._use_simulated = True
        elif not can:
            self._node.get_logger().warning("python-can not available; CAN ingestor running in simulation mode.")
            self._use_simulated = True

    def start(self) -> None:
        if self._use_simulated or not self._bus:
            self._node.create_timer(0.02, self._publish_simulated_frame)
            return

        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        self._node.get_logger().info("Started CAN polling thread.")

    def _poll_loop(self) -> None:
        assert self._bus is not None
        while not self._stop_event.is_set():
            try:
                frame = self._bus.recv(timeout=self._poll_interval)
                if frame:
                    msg = Frame()
                    msg.id = frame.arbitration_id
                    msg.dlc = frame.dlc
                    msg.is_extended = frame.is_extended_id
                    msg.is_rtr = frame.is_remote_frame
                    msg.is_error = frame.is_error_frame
                    msg.data = list(frame.data)
                    msg.header.stamp = self._node.get_clock().now().to_msg()
                    self._publisher.publish(msg)
            except Exception as exc:  # pylint: disable=broad-except
                self._node.get_logger().error(f"Error reading CAN frame: {exc}", throttle_duration_sec=5.0)

    def _publish_simulated_frame(self) -> None:
        msg = Frame()
        msg.id = random.randint(0, 2047)
        msg.dlc = 8
        msg.data = [random.getrandbits(8) for _ in range(8)]
        msg.is_extended = False
        msg.is_rtr = False
        msg.is_error = False
        msg.header.stamp = self._node.get_clock().now().to_msg()
        self._publisher.publish(msg)

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._bus:
            self._bus.shutdown()
        super().shutdown()
