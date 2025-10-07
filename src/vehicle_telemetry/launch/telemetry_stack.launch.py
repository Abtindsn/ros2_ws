import logging
import os
from pathlib import Path
from typing import Any, Optional

try:
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.logging import get_logger
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue

    _LAUNCH_IMPORT_ERROR: Optional[ModuleNotFoundError] = None
except ModuleNotFoundError as exc:  # pragma: no cover - handled fallback path
    LaunchDescription = Any  # type: ignore
    DeclareLaunchArgument = Any  # type: ignore
    LaunchConfiguration = Any  # type: ignore
    Node = Any  # type: ignore
    ParameterValue = Any  # type: ignore
    get_logger = None  # type: ignore
    _LAUNCH_IMPORT_ERROR = exc

LOGGER = get_logger(__name__) if get_logger else logging.getLogger(__name__)


def _get_package_share_directory() -> Path:
    share_override = os.environ.get("VEHICLE_TELEMETRY_SHARE")
    if share_override:
        return Path(share_override)

    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore

        return Path(get_package_share_directory("vehicle_telemetry"))
    except ModuleNotFoundError:
        fallback = Path(__file__).resolve().parents[1]
        LOGGER.warning(
            "ament_index_python not available; falling back to package directory at %s",
            fallback,
        )
        return fallback


def generate_launch_description() -> LaunchDescription:
    if _LAUNCH_IMPORT_ERROR is not None:
        raise RuntimeError(
            "ROS 2 launch infrastructure is unavailable. "
            "Ensure the ROS environment is sourced (e.g., 'source /opt/ros/<distro>/setup.bash') "
            "before running this launch file."
        ) from _LAUNCH_IMPORT_ERROR

    package_share = _get_package_share_directory()
    topics_yaml = str(package_share / "config" / "telemetry_topics.yaml")
    recorder_yaml = str(package_share / "config" / "recorder.yaml")
    if not Path(topics_yaml).exists():
        LOGGER.warning("Telemetry topics config not found at %s", topics_yaml)
    if not Path(recorder_yaml).exists():
        LOGGER.warning("Recorder config not found at %s", recorder_yaml)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_simulated_sources",
                default_value="true",
                choices=["true", "false"],
                description="Enable simulated telemetry producers when hardware is unavailable.",
            ),
            Node(
                package="vehicle_telemetry",
                executable="telemetry_ingestor",
                name="telemetry_ingestor",
                parameters=[
                    {
                        "topics_config_path": topics_yaml,
                        "use_simulated_sources": ParameterValue(
                            LaunchConfiguration("use_simulated_sources"), value_type=bool
                        ),
                    },
                ],
                output="screen",
            ),
            Node(
                package="vehicle_telemetry",
                executable="telemetry_recorder",
                name="telemetry_recorder",
                parameters=[
                    {
                        "recorder_config_path": recorder_yaml,
                        "topics_config_path": topics_yaml,
                    }
                ],
                output="screen",
            ),
            Node(
                package="vehicle_telemetry",
                executable="telemetry_dashboard",
                name="telemetry_dashboard",
                parameters=[{"topics_config_path": topics_yaml}],
                output="screen",
            ),
        ]
    )


if __name__ == "__main__":  # pragma: no cover - convenience for direct execution
    if _LAUNCH_IMPORT_ERROR is not None:
        LOGGER.error(
            "Cannot construct launch description because ROS 2 launch packages are missing. "
            "Source your ROS environment (e.g., 'source /opt/ros/<distro>/setup.bash') "
            "before running this file."
        )
    else:
        generate_launch_description()
        LOGGER.info(
            "Launch description created. Run with 'ros2 launch vehicle_telemetry telemetry_stack.launch.py'."
        )
