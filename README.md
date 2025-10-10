# ROS 2 Vehicle Telemetry Stack

A turnkey ROS 2 workspace for collecting, visualising, and recording vehicle telemetry.  
It launches a complete pipeline:

- **Telemetry ingestors** publish CAN, IMU, GPS, wheel speed, and camera topics (real hardware or simulation).
- **Recorder** wraps `ros2 bag record`, manages rolling splits, and flushes cleanly on shutdown.
- **Dashboard** (FastAPI + Uvicorn) serves live diagnostics, including FPS badges and a camera preview.
- **Replay utilities** play recorded bag files back into the ROS graph.

Everything runs either natively or inside the provided container image.

---

## Repository Layout

```
ros2_ws/
  ├── Dockerfile                 # Humble base image + pinned deps (numpy 1.24.4, OpenCV, cv_bridge)
  ├── scripts/
  │   └── run_stack_docker.sh    # Convenience wrapper around podman build/run
  └── src/vehicle_telemetry/
      ├── config/                # telemetry_topics.yaml, recorder.yaml, rosbag QoS overrides
      ├── launch/                # telemetry_stack.launch.py (ingestor + recorder + dashboard)
      └── vehicle_telemetry/
          ├── ingestors/         # Sensor publishers (camera, CAN, IMU, GPS, wheel speed)
          ├── recording/         # Recorder node that spawns rosbag2 with graceful shutdown
          ├── dashboard/         # FastAPI dashboard with live cards + /camera.jpg endpoint
          ├── nodes/             # Entry points (ingestor, simulator, replay)
          └── utils/             # YAML config loaders, QoS helpers
```

---

## Architecture at a Glance

| Component          | Path                                           | Notes                                                                                     |
|-------------------|------------------------------------------------|-------------------------------------------------------------------------------------------|
| Launch file        | `launch/telemetry_stack.launch.py`             | Starts ingestor, recorder, dashboard; forwards `use_simulated_sources`.                   |
| Ingestors          | `ingestors/*.py`                               | Extend `TelemetryIngestor`; publish with per-topic QoS from YAML.                         |
| Camera ingestor    | `ingestors/camera.py`                          | Uses OpenCV/cv_bridge when available, otherwise synthesises frames (no NumPy subscripts). |
| Recorder           | `recording/recorder_node.py`                   | Spawns `ros2 bag record`, writes `recorder.log`, handles SIGINT/SIGKILL on stop.          |
| Dashboard          | `dashboard/dashboard_node.py`                  | Caches latest messages, serves HTML UI, `/latest`, `/camera.jpg`, `/healthz`.             |
| Config loader      | `utils/config_loader.py`                       | Parses QoS (reliability, durability, history, depth) and recorder settings.               |

Dashboard, recorder, and ingestors all honour the same QoS declaration, so best-effort camera topics Just Work™.

---

## Running the Stack

### Container Workflow (recommended)

```bash
cd ~/ros2_ws
sudo podman build --network=host -t vehicle-telemetry:latest .

sudo podman rm -f vt 2>/dev/null || true
mkdir -p ~/rosbags

sudo podman run -d --name vt --network=host \
  -e ROS_DOMAIN_ID=42 \
  -e USE_SIMULATED_SOURCES=1 \
  -v ~/rosbags:/tmp/rosbags \
  vehicle-telemetry:latest \
  bash -lc '. install/setup.bash && ros2 launch vehicle_telemetry telemetry_stack.launch.py use_simulated_sources:=true'
```

Optional environment variables before launching:

- `CAN_DEVICES=/dev/can0,/dev/can1` and `CAMERA_DEVICES=/dev/video0` to map hardware into the container.
- `USE_SIMULATED_SOURCES=0` (or pass `use_simulated_sources:=false`) to disable synthetic publishers.

Handy commands once the container is running:

```bash
sudo podman logs -f vt | grep "camera frame"     # confirm camera frames arrive

# flush recorder (SIGINT) and wait until it exits
sudo podman exec vt bash -lc 'pkill -2 -f "ros2 bag record" || true; \
  for i in {1..30}; do pgrep -fa "ros2 bag record" >/dev/null || break; sleep 0.5; done'

# stop the container
sudo podman stop --time 60 vt
# (optional) remove it
sudo podman rm -f vt
```

The helper script `scripts/run_stack_docker.sh` wraps build and launch if you prefer a single command.

### Native ROS 2 Install

```bash
sudo apt update && sudo apt install python3-colcon-common-extensions python3-rosdep
cd ~/ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch vehicle_telemetry telemetry_stack.launch.py use_simulated_sources:=true
```

Flip to real sensors by launching with `use_simulated_sources:=false`.

---

## Dashboard & APIs

- Web UI: `http://localhost:8080/`
  - Live cards for IMU, GPS, wheel speed, CAN, camera preview
  - Connection status indicator + per-topic FPS badges
  - Pause/resume polling button and raw JSON viewer
- REST endpoints:
  - `GET /health` – topic list + status
  - `GET /healthz` – simple OK probe
  - `GET /latest` / `GET /latest/{key}` – latest samples
- `GET /camera.jpg` – most recent camera frame (204 until a frame arrives)
  - `HEAD /camera.jpg` – 200 if a frame is cached, else 204

---

## Recording & Bag Management

- `config/recorder.yaml` controls:
  - Output root (`/tmp/rosbags`)
  - Rolling splits (10 s duration, 1 GB size in bytes)
  - Compression toggle (compression disabled by default)
  - Topic allowlist
- Recorder writes one directory per session:

```
/tmp/rosbags/<timestamp>/
  metadata.yaml
  telemetry_0.db3
  recorder.log    # rosbag stdout/stderr for debugging
```

- `config/rosbag_qos.yaml` keeps rosbag subscriptions best-effort to match the ingestors.
- Exposes `telemetry_recorder/start` and `telemetry_recorder/stop` (`std_srvs/Trigger`) services.

---

## Modifying or Extending the Stack

- **Add sensors** – create a class in `vehicle_telemetry/ingestors/`, register it in `ingestors/base.py`, and update `telemetry_topics.yaml`.
- **Adjust QoS** – tweak reliability/durability/history/depth in `telemetry_topics.yaml`; ingestors, recorder, and dashboard adapt automatically.
- **Tune recorder** – change split sizes/duration, enable ZSTD compression, or adjust topic allowlist in `config/recorder.yaml`.
- **Custom UI** – edit `dashboard/dashboard_node.py` (plain HTML/CSS/JS) to add charts, overlays, etc.

---

## Development Notes

- Designed around ROS 2 Humble; the Dockerfile captures all required dependencies (notably `numpy==1.24.4` to satisfy both OpenCV typing and `cv_bridge`).
- Python dependencies include `python-can`, `pyserial`, `pynmea2`, `opencv-python-headless==4.8.1.78`, `cv_bridge`, `fastapi`, `uvicorn[standard]`, and matching ROS message packages.
- Camera preview works with BGR/RGB 8-bit frames. Without `cv2`/`cv_bridge`, the ingestor falls back to simulated gradients so the dashboard still lights up.
- Recorder logs to each session directory (`recorder.log`)—read this first if bag files do not appear.
- Always export `ROS_DOMAIN_ID=42` (or match the container’s ID) when running `ros2` CLI commands on the host to see the same ROS graph.

---

Happy hacking—flip between simulation and hardware, record everything, and watch the live telemetry in your browser!

---

## Author

- Abtin Doostan – Initial work & maintenance – [@abtindsn](https://github.com/abtindsn)
