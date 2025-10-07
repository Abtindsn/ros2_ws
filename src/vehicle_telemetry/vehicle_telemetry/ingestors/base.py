from __future__ import annotations

import abc
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from vehicle_telemetry.utils.config_loader import QoSConfig, TopicConfig


def _build_qos_profile(config: QoSConfig) -> QoSProfile:
    reliability = ReliabilityPolicy.RELIABLE if config.reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT
    durability = DurabilityPolicy.TRANSIENT_LOCAL if config.durability == "transient_local" else DurabilityPolicy.VOLATILE
    history = HistoryPolicy.KEEP_ALL if config.history == "keep_all" else HistoryPolicy.KEEP_LAST
    depth = max(1, int(config.depth)) if history == HistoryPolicy.KEEP_LAST else 10
    return QoSProfile(
        depth=depth,
        reliability=reliability,
        durability=durability,
        history=history,
    )


class TelemetryIngestor(abc.ABC):
    """Base class for telemetry ingestors that publish to ROS2 topics."""

    def __init__(self, node: Node, topic: TopicConfig, use_simulated: bool) -> None:
        self._node = node
        self._topic = topic
        self._use_simulated = use_simulated
        try:
            msg_type = get_message(topic.type)
        except (ModuleNotFoundError, AttributeError) as exc:
            raise RuntimeError(
                f"Unable to import message type '{topic.type}'. Ensure required ROS2 interfaces are built."
            ) from exc
        self._publisher = self._node.create_publisher(
            msg_type,
            topic.name,
            _build_qos_profile(topic.qos),
        )

    @abc.abstractmethod
    def start(self) -> None:
        """Start data ingestion."""

    def shutdown(self) -> None:
        """Shutdown ingest task."""
        self._node.get_logger().info(f"{self.__class__.__name__} shutdown.")


def create_ingestor(node: Node, key: str, topic: TopicConfig, use_sim: bool):
    """Factory to instantiate ingestors based on topic key."""
    ingestor_map = {
        "can": "vehicle_telemetry.ingestors.can:CANIngestor",
        "imu": "vehicle_telemetry.ingestors.imu:IMUIngestor",
        "gps": "vehicle_telemetry.ingestors.gps:GPSIngestor",
        "wheel_speed": "vehicle_telemetry.ingestors.wheel_speed:WheelSpeedIngestor",
        "camera_front": "vehicle_telemetry.ingestors.camera:CameraIngestor",
    }
    dotted_path = ingestor_map.get(key)
    if not dotted_path:
        raise ValueError(f"No ingestor registered for '{key}'")

    module_name, _, class_name = dotted_path.rpartition(":")
    module = __import__(module_name, fromlist=[class_name])
    cls = getattr(module, class_name)
    return cls(node=node, topic=topic, use_simulated=use_sim)
