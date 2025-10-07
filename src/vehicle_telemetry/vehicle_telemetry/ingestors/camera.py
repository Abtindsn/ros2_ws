from __future__ import annotations

import threading
from typing import Optional

from rclpy.node import Node
from sensor_msgs.msg import Image

from vehicle_telemetry.ingestors.base import TelemetryIngestor

try:
    import cv2  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    cv2 = None

try:
    from cv_bridge import CvBridge  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    CvBridge = None


class CameraIngestor(TelemetryIngestor):
    """Camera ingestor that streams frames into ROS2."""

    def __init__(self, node: Node, topic, use_simulated: bool) -> None:
        super().__init__(node, topic, use_simulated)
        self._device_index = node.declare_parameter("camera.device_index", 0).value
        self._frame_id = node.declare_parameter("camera.frame_id", "camera_front").value
        self._frame_rate = node.declare_parameter("camera.frame_rate", 15.0).value
        self._bridge = CvBridge() if CvBridge else None
        self._capture: Optional["cv2.VideoCapture"] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        if not self._use_simulated and cv2 and self._bridge:
            self._capture = cv2.VideoCapture(self._device_index)
            if not self._capture.isOpened():
                self._node.get_logger().warning(
                    f"Unable to open camera device {self._device_index}; enabling simulated camera frames."
                )
                self._use_simulated = True
        else:
            if not cv2:
                self._node.get_logger().warning("OpenCV not installed; camera ingestor running in simulation mode.")
            if not self._bridge:
                self._node.get_logger().warning("cv_bridge not installed; camera ingestor running in simulation mode.")
            self._use_simulated = True

    def start(self) -> None:
        if self._use_simulated or not self._capture or not self._bridge:
            period = 1.0 / max(self._frame_rate, 1.0)
            self._node.create_timer(period, self._publish_simulated_frame)
            return
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self._node.get_logger().info("Camera capture thread started.")

    def _capture_loop(self) -> None:
        assert self._capture is not None and self._bridge is not None
        period = 1.0 / max(self._frame_rate, 1.0)
        while not self._stop_event.is_set():
            ret, frame = self._capture.read()
            if not ret:
                self._node.get_logger().warning("Failed to read camera frame", throttle_duration_sec=5.0)
                continue
            msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            self._publisher.publish(msg)
            self._stop_event.wait(period)

    def _publish_simulated_frame(self) -> None:
        height, width = 480, 640
        try:
            import numpy as np  # type: ignore
        except ModuleNotFoundError:
            self._node.get_logger().warning("numpy missing; skipping simulated camera frame.")
            return

        gradient = np.linspace(0, 255, width, dtype=np.uint8)
        frame = np.tile(gradient, (height, 1))
        frame = np.stack([frame, np.flipud(frame), frame], axis=2)
        if self._bridge:
            msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        else:
            msg = Image()
            msg.height = height
            msg.width = width
            msg.encoding = "rgb8"
            msg.step = width * 3
            msg.data = frame.tobytes()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._capture:
            self._capture.release()
        super().shutdown()
