# ROS2 Vehicle Telemetry Stack

Plug-and-play ROS2 stack that records and replays vehicle telemetry (CAN, IMU, GPS, wheel speed, cameras) with a real-time debugging dashboard for driverless EV teams.

## Workspace Layout

```
ros2_ws/
  ├── src/vehicle_telemetry
  │   ├── config/             # Topic + recorder configuration
  │   ├── launch/             # Launch description for the stack
  │   └── vehicle_telemetry/  # Python package with nodes and helpers
  └── README.md
```

## Quick Start

```bash
cd ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build
source install/setup.bash
ros2 launch vehicle_telemetry telemetry_stack.launch.py
```

### Docker (recommended for first run)

```bash
cd ros2_ws
./scripts/run_stack_docker.sh
```

The helper script builds the container, installs dependencies, compiles the workspace, and launches the stack.

Hardware passthrough & simulation control:

- Export comma-separated device lists (e.g. `CAN_DEVICES=/dev/can0,/dev/can1`, `CAMERA_DEVICES=/dev/video0`) to expose hardware into the container.
- Set `USE_SIMULATION=false` to disable synthetic publishers and rely on real sensors (`use_simulated_sources:=false` is forwarded to the ingestor).

The launch file starts:

- `telemetry_ingestor`: spins up CAN, IMU, GPS, wheel speed, and camera producers (real or simulated).
- `telemetry_recorder`: manages `ros2 bag record` sessions with compression and splitting.
- `telemetry_dashboard`: serves a FastAPI dashboard exposing most recent samples.

Add `telemetry_simulator:=true` to run the simulator node instead of hardware drivers:

```bash
ros2 run vehicle_telemetry telemetry_simulator --ros-args -p topics_config_path:=<path-to-telemetry_topics.yaml>
```

Launch without Docker and disable simulation via:

```bash
ros2 launch vehicle_telemetry telemetry_stack.launch.py use_simulated_sources:=false
```

## Configuration

- `config/telemetry_topics.yaml`: declares the topics, message types, and QoS settings.
- `config/recorder.yaml`: controls rosbag storage, compression, and topic selection.

Override parameters by passing custom YAML files or `--ros-args -p <param>:=<value>` to the nodes.

## Services & API

- `telemetry_recorder/start` & `telemetry_recorder/stop` (`std_srvs/Trigger`): control bag recording.
- `telemetry_replay/play` & `telemetry_replay/stop` (`std_srvs/Trigger`): control bag playback.
- Dashboard REST endpoints:
  - `GET /health`
  - `GET /latest`
  - `GET /latest/{key}`

## Extending the Stack

- Add hardware-specific ingest logic by extending classes in `vehicle_telemetry.ingestors`.
- Update `telemetry_topics.yaml` to publish additional signals; the recorder and dashboard pick them up automatically.
- Adjust recorder defaults (e.g., output path, compression) in `config/recorder.yaml`.

## Development Notes

- Requires ROS2 (Foxy or later), `python-can`, `pyserial`, `pynmea2`, OpenCV (`opencv-python-headless` recommended), `cv_bridge`, `fastapi`, `uvicorn`, and `numpy`.
- Launching on bare metal CAN or serial hardware may need additional OS configuration (SocketCAN, udev rules, etc.).
