from __future__ import annotations

import math
import threading
from typing import Optional

from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from vehicle_telemetry.ingestors.base import TelemetryIngestor

try:
    import pynmea2  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    pynmea2 = None

try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    serial = None


class GPSIngestor(TelemetryIngestor):
    """GPS ingestor that parses NMEA sentences from serial."""

    def __init__(self, node: Node, topic, use_simulated: bool) -> None:
        super().__init__(node, topic, use_simulated)
        self._port = node.declare_parameter("gps.port", "/dev/ttyUSB1").value
        self._baudrate = node.declare_parameter("gps.baudrate", 9600).value
        self._frame_id = node.declare_parameter("gps.frame_id", "gps_link").value
        self._publish_velocity = node.declare_parameter("gps.publish_velocity", True).value
        self._velocity_pub = None
        if self._publish_velocity:
            self._velocity_pub = self._node.create_publisher(
                TwistStamped,
                "/vehicle/gps/velocity",
                10,
            )
        self._serial: Optional["serial.Serial"] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._time = 0.0

        if not self._use_simulated and serial and pynmea2:
            try:
                self._serial = serial.Serial(self._port, self._baudrate, timeout=0.2)
                self._node.get_logger().info(f"GPS serial connected on {self._port}")
            except Exception as exc:  # pylint: disable=broad-except
                self._node.get_logger().warning(
                    f"Failed to open GPS serial port ({exc}); enabling simulated GPS data."
                )
                self._use_simulated = True
        else:
            if not serial:
                self._node.get_logger().warning("pyserial not installed; GPS ingestor running in simulation mode.")
            if not pynmea2:
                self._node.get_logger().warning("pynmea2 not installed; GPS ingestor running in simulation mode.")
            self._use_simulated = True

    def start(self) -> None:
        if self._use_simulated or not self._serial:
            self._node.create_timer(0.1, self._publish_simulated)
            return
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self._node.get_logger().info("Started GPS reader thread.")

    def _read_loop(self) -> None:
        assert self._serial is not None
        while not self._stop_event.is_set():
            try:
                raw = self._serial.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue
                msg = pynmea2.parse(raw)
                if isinstance(msg, pynmea2.types.talker.GGA):
                    nav = NavSatFix()
                    nav.header.stamp = self._node.get_clock().now().to_msg()
                    nav.header.frame_id = self._frame_id
                    nav.latitude = msg.latitude
                    nav.longitude = msg.longitude
                    nav.altitude = msg.altitude
                    nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    self._publisher.publish(nav)
                elif isinstance(msg, pynmea2.types.talker.RMC) and self._velocity_pub:
                    twist = TwistStamped()
                    twist.header.stamp = self._node.get_clock().now().to_msg()
                    twist.header.frame_id = self._frame_id
                    # Convert knots to m/s
                    speed_ms = float(msg.spd_over_grnd or 0.0) * 0.514444
                    course_rad = math.radians(float(msg.true_course or 0.0))
                    twist.twist.linear.x = speed_ms * math.cos(course_rad)
                    twist.twist.linear.y = speed_ms * math.sin(course_rad)
                    self._velocity_pub.publish(twist)
            except Exception as exc:  # pylint: disable=broad-except
                self._node.get_logger().error(f"GPS read error: {exc}", throttle_duration_sec=5.0)

    def _publish_simulated(self) -> None:
        self._time += 0.1
        nav = NavSatFix()
        nav.header.stamp = self._node.get_clock().now().to_msg()
        nav.header.frame_id = self._frame_id
        # Simulate circular path around a reference point
        base_lat = 48.2620
        base_lon = 11.6670
        radius_deg = 0.0005
        nav.latitude = base_lat + radius_deg * math.cos(self._time / 10.0)
        nav.longitude = base_lon + radius_deg * math.sin(self._time / 10.0)
        nav.altitude = 500.0 + 2.0 * math.sin(self._time / 30.0)
        nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self._publisher.publish(nav)

        if self._velocity_pub:
            twist = TwistStamped()
            twist.header.stamp = nav.header.stamp
            twist.header.frame_id = self._frame_id
            linear_speed = 15.0
            twist.twist.linear.x = linear_speed * math.cos(self._time / 10.0)
            twist.twist.linear.y = linear_speed * math.sin(self._time / 10.0)
            self._velocity_pub.publish(twist)

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().shutdown()
