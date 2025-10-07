from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import yaml


@dataclass
class QoSConfig:
    reliability: str = "reliable"
    durability: str = "volatile"


@dataclass
class TopicConfig:
    name: str
    type: str
    qos: QoSConfig


def _as_qos(data: Dict) -> QoSConfig:
    qos_data = data or {}
    return QoSConfig(
        reliability=qos_data.get("reliability", "reliable"),
        durability=qos_data.get("durability", "volatile"),
    )


def load_topics_config(path: str | Path) -> Dict[str, TopicConfig]:
    """Load telemetry topic configuration from YAML file."""
    with open(path, "r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}

    topic_entries = raw.get("topics", {})
    topics: Dict[str, TopicConfig] = {}
    for key, entry in topic_entries.items():
        topic_name = entry.get("topic")
        topic_type = entry.get("type")
        if not topic_name or not topic_type:
            raise ValueError(f"Topic '{key}' is missing required fields 'topic' or 'type'")
        topics[key] = TopicConfig(
            name=topic_name,
            type=topic_type,
            qos=_as_qos(entry.get("qos", {})),
        )
    return topics


@dataclass
class RecorderConfig:
    output_directory: Path
    storage_id: str
    serialization_format: str
    compression_enabled: bool
    compression_format: Optional[str]
    compression_queue_size: int
    max_bag_size_mb: Optional[int]
    max_bag_duration_s: Optional[int]
    all_topics: bool
    include_topics: List[str]


def load_recorder_config(path: str | Path) -> RecorderConfig:
    """Load recorder configuration from YAML file."""
    with open(path, "r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}

    data = raw.get("recording", {})
    compression = data.get("compression", {})
    return RecorderConfig(
        output_directory=Path(data.get("output_directory", "/tmp/rosbags")),
        storage_id=data.get("storage_id", "sqlite3"),
        serialization_format=data.get("serialization_format", "cdr"),
        compression_enabled=bool(compression.get("enabled", False)),
        compression_format=compression.get("format"),
        compression_queue_size=int(compression.get("queue_size", 1)),
        max_bag_size_mb=data.get("max_bag_size_mb"),
        max_bag_duration_s=data.get("max_bag_duration_s"),
        all_topics=bool(data.get("all_topics", False)),
        include_topics=list(data.get("include_topics", [])),
    )
