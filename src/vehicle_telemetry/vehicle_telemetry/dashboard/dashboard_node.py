from __future__ import annotations

import threading
from typing import Dict, Optional

import rclpy
from fastapi import FastAPI
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict
from uvicorn import Config, Server

from vehicle_telemetry.utils.config_loader import QoSConfig, TopicConfig, load_topics_config


class TelemetryDashboardNode(Node):
    """Node that exposes a FastAPI dashboard with the latest telemetry samples."""

    def __init__(self) -> None:
        super().__init__("telemetry_dashboard")
        self.declare_parameter("topics_config_path", "")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)

        self._topics = self._load_topics()
        self._latest_messages: Dict[str, Dict] = {}
        self._lock = threading.Lock()
        self._server: Optional[Server] = None
        self._server_thread: Optional[threading.Thread] = None

        for key, topic in self._topics.items():
            msg_type = self._import_msg_type(topic.type)
            self.create_subscription(
                msg_type,
                topic.name,
                lambda msg, key=key: self._on_message(key, msg),
                10,
            )
            self.get_logger().info(f"Dashboard listening to {topic.name} ({topic.type}) as '{key}'")

        self._app = self._create_app()
        self._start_server()

    def _import_msg_type(self, type_name: str):
        from rosidl_runtime_py.utilities import get_message

        return get_message(type_name)

    def _load_topics(self):
        path = self.get_parameter("topics_config_path").value
        if path:
            return load_topics_config(path)
        topics_param = self.get_parameter("topics").value
        if isinstance(topics_param, dict):
            topics: Dict[str, TopicConfig] = {}
            for key, value in topics_param.items():
                qos = value.get("qos", {})
                topics[key] = TopicConfig(
                    name=value.get("topic"),
                    type=value.get("type"),
                    qos=QoSConfig(
                        reliability=qos.get("reliability", "reliable"),
                        durability=qos.get("durability", "volatile"),
                    ),
                )
            return topics
        raise RuntimeError("Dashboard requires telemetry topics configuration.")

    def _on_message(self, key: str, msg) -> None:
        with self._lock:
            self._latest_messages[key] = {
                "stamp": self.get_clock().now().nanoseconds,
                "data": message_to_ordereddict(msg),
            }

    def _create_app(self) -> FastAPI:
        app = FastAPI(title="Telemetry Dashboard", version="0.1.0")

        @app.get("/")
        def index():
            return {
                "status": "ok",
                "endpoints": ["/", "/health", "/healthz", "/latest", "/latest/{key}"],
            }

        @app.get("/health")
        def health():
            return {"status": "ok", "sources": list(self._topics.keys())}

        @app.get("/healthz")
        def healthz():
            return {"ok": True}

        @app.get("/latest")
        def latest():
            with self._lock:
                return self._latest_messages

        @app.get("/latest/{key}")
        def latest_key(key: str):
            with self._lock:
                data = self._latest_messages.get(key)
            if data is None:
                return {"error": f"No data for key '{key}'"}
            return data

        return app

    def _start_server(self) -> None:
        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        config = Config(app=self._app, host=host, port=port, log_level="info")
        self._server = Server(config)
        self._server_thread = threading.Thread(target=self._server.run, daemon=True)
        self._server_thread.start()
        self.get_logger().info(f"Telemetry dashboard serving at http://{host}:{port}")

    def destroy_node(self) -> bool:
        if self._server:
            self._server.should_exit = True
        if self._server_thread:
            self._server_thread.join(timeout=2)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryDashboardNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Telemetry dashboard interrupted.")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
