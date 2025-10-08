# Copyright (c) 2025 Abtin Doostan
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from __future__ import annotations

from typing import Dict

import rclpy
from rclpy.node import Node

from vehicle_telemetry.ingestors.base import create_ingestor
from vehicle_telemetry.utils.config_loader import QoSConfig, TopicConfig, load_topics_config


class TelemetryIngestorNode(Node):
    """Node that spins up telemetry ingestors based on configuration."""

    def __init__(self) -> None:
        super().__init__("telemetry_ingestor")
        self.declare_parameter("topics_config_path", "")
        self.declare_parameter("use_simulated_sources", True)
        self._ingestors = []

        use_simulated = self.get_parameter("use_simulated_sources").value
        topics = self._load_topics()

        for key, topic in topics.items():
            try:
                ingestor = create_ingestor(self, key, topic, use_simulated)
                ingestor.start()
                self._ingestors.append(ingestor)
                self.get_logger().info(f"Started ingestor '{key}' publishing to {topic.name}")
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(f"Failed to start ingestor '{key}': {exc}")

        self.get_logger().info(f"Telemetry ingestor active with {len(self._ingestors)} sources.")

    def _load_topics(self) -> Dict[str, TopicConfig]:
        path = self.get_parameter("topics_config_path").value
        if path:
            return load_topics_config(path)

        topics_param = self.get_parameter("topics").value
        if isinstance(topics_param, dict):
            topics: Dict[str, TopicConfig] = {}
            for key, entry in topics_param.items():
                qos_data = entry.get("qos", {})
                topics[key] = TopicConfig(
                    name=entry.get("topic"),
                    type=entry.get("type"),
                    qos=QoSConfig(
                        reliability=qos_data.get("reliability", "reliable"),
                        durability=qos_data.get("durability", "volatile"),
                    ),
                )
            return topics
        raise RuntimeError("No telemetry topics provided. Set 'topics_config_path' or pass a YAML with 'topics'.")

    def destroy_node(self) -> bool:
        for ingestor in self._ingestors:
            try:
                ingestor.shutdown()
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(f"Error shutting down ingestor: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryIngestorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Telemetry ingestor interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
