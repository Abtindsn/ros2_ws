#!/usr/bin/env bash

set -euo pipefail

IMAGE_NAME=${IMAGE_NAME:-vehicle-telemetry:latest}
DOCKERFILE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

echo "Building Docker image '${IMAGE_NAME}'..."
sudo podman build  -t "${IMAGE_NAME}" "${DOCKERFILE_DIR}"

map_devices() {
    local value=$1
    local -a result=()
    if [[ -n "${value}" ]]; then
        # Split on comma or whitespace
        IFS=', ' read -r -a devices <<< "${value}"
        for dev in "${devices[@]}"; do
            [[ -z "${dev}" ]] && continue
            if [[ ! -e "${dev}" ]]; then
                echo "Warning: device path '${dev}' does not exist; skipping." >&2
                continue
            fi
            result+=("--device=${dev}")
        done
    fi
    echo "${result[@]}"
}

declare -a DOCKER_RUN_ARGS
DOCKER_RUN_ARGS=(
    --rm
    -it
    --net=host
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
)

if [[ -n "${CAN_DEVICES:-}" ]]; then
    DOCKER_RUN_ARGS+=($(map_devices "${CAN_DEVICES}"))
fi

if [[ -n "${CAMERA_DEVICES:-}" ]]; then
    DOCKER_RUN_ARGS+=($(map_devices "${CAMERA_DEVICES}"))
fi

SIMULATION_DEFAULT="true"
if [[ -n "${USE_SIMULATION:-}" ]]; then
    case "${USE_SIMULATION}" in
        0|false|False|FALSE) SIMULATION_DEFAULT="false" ;;
        1|true|True|TRUE) SIMULATION_DEFAULT="true" ;;
        *)
            echo "Unknown USE_SIMULATION value '${USE_SIMULATION}', defaulting to 'true'." >&2
            ;;
    esac
fi

ROS_LAUNCH_CMD=". install/setup.bash && ros2 launch vehicle_telemetry telemetry_stack.launch.py use_simulated_sources:=${SIMULATION_DEFAULT}"

echo "Launching telemetry stack from image '${IMAGE_NAME}' (use_simulated_sources=${SIMULATION_DEFAULT})..."
sudo podman run  "${DOCKER_RUN_ARGS[@]}" \
    "${IMAGE_NAME}" \
    bash -lc "${ROS_LAUNCH_CMD}"
