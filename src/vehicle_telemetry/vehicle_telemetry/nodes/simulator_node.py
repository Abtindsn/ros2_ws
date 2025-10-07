from __future__ import annotations

import rclpy
from rclpy.node import Node

from vehicle_telemetry.ingestors.base import create_ingestor
from vehicle_telemetry.utils.config_loader import QoSConfig, TopicConfig, load_topics_config


class TelemetrySimulatorNode(Node):
    """Node that reuses ingestor simulation paths to publish synthetic telemetry data."""

    def __init__(self) -> None:
        super().__init__("telemetry_simulator")
        self.declare_parameter("topics_config_path", "")
        self._ingestors = []
        topics = self._load_topics()
        for key, topic in topics.items():
            ingestor = create_ingestor(self, key, topic, use_sim=True)
            ingestor.start()
            self._ingestors.append(ingestor)
            self.get_logger().info(f"Simulator publishing synthetic data for '{key}' on {topic.name}")

    def _load_topics(self):
        path = self.get_parameter("topics_config_path").value
        if path:
            return load_topics_config(path)
        topics_param = self.get_parameter("topics").value
        if isinstance(topics_param, dict):
            topics = {}
            for key, entry in topics_param.items():
                qos = entry.get("qos", {})
                topics[key] = TopicConfig(
                    name=entry.get("topic"),
                    type=entry.get("type"),
                    qos=QoSConfig(
                        reliability=qos.get("reliability", "reliable"),
                        durability=qos.get("durability", "volatile"),
                    ),
                )
            return topics
        raise RuntimeError("Simulator requires telemetry topics configuration.")

    def destroy_node(self) -> bool:
        for ingestor in self._ingestors:
            ingestor.shutdown()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetrySimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Telemetry simulator interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
