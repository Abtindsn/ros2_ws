from setuptools import find_packages, setup

package_name = "vehicle_telemetry"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/telemetry_stack.launch.py"]),
        (
            "share/" + package_name + "/config",
            ["config/telemetry_topics.yaml", "config/recorder.yaml", "config/rosbag_qos.yaml"],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Abtin Doostan",
    maintainer_email="abtindsn@users.noreply.github.com",
    description="ROS2 stack for logging and replaying Formula Student telemetry data.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "telemetry_ingestor = vehicle_telemetry.nodes.ingestor_node:main",
            "telemetry_recorder = vehicle_telemetry.recording.recorder_node:main",
            "telemetry_replay = vehicle_telemetry.replay.replay_node:main",
            "telemetry_dashboard = vehicle_telemetry.dashboard.dashboard_node:main",
            "telemetry_simulator = vehicle_telemetry.nodes.simulator_node:main",
        ],
    },
)
