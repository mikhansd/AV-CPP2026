#!/usr/bin/env bash
set -Eeuo pipefail

SLAM_PARAMS="${1:-$HOME/AV-CPP2026/maps/slam_toolbox.yaml}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"

BASE_FRAME="${BASE_FRAME:-base_link}"
LASER_FRAME="${LASER_FRAME:-lidar_link}"

# Replace these with the measured LiDAR mounting pose.
LASER_X="${LASER_X:-0.0}"
LASER_Y="${LASER_Y:-0.0}"
LASER_Z="${LASER_Z:-0.0}"
LASER_YAW="${LASER_YAW:-0.0}"
LASER_PITCH="${LASER_PITCH:-0.0}"
LASER_ROLL="${LASER_ROLL:-0.0}"

EKF_NODE="/ekf_localization_node"

wait_for_node() {
    local node="$1"
    local timeout="$2"
    local elapsed=0

    echo "Waiting for $node..."

    until ros2 node list 2>/dev/null | grep -Fxq "$node"; do
        sleep 1
        elapsed=$((elapsed + 1))

        if (( elapsed >= timeout )); then
            echo "ERROR: $node was not found."
            echo "Start the vehicle bringup first."
            exit 1
        fi
    done
}

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ROS 2 is not sourced."
    exit 1
fi

if [[ ! -r "$SLAM_PARAMS" ]]; then
    echo "ERROR: SLAM parameter file not found:"
    echo "  $SLAM_PARAMS"
    exit 1
fi

if ros2 node list 2>/dev/null | grep -Fxq "/map_server"; then
    echo "ERROR: /map_server is running."
    echo "Stop the static map server before starting SLAM mapping."
    exit 1
fi

wait_for_node "$EKF_NODE" 30

echo "EKF is running; leaving TF publication enabled."

# echo "Publishing $BASE_FRAME -> $LASER_FRAME..."
# ros2 run tf2_ros static_transform_publisher \
#     "$LASER_X" "$LASER_Y" "$LASER_Z" \
#     "$LASER_YAW" "$LASER_PITCH" "$LASER_ROLL" \
#     "$BASE_FRAME" "$LASER_FRAME" &
# TF_PID=$!

cleanup() {
    echo
    echo "Stopping static LiDAR transform..."
    kill "$TF_PID" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

echo "Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$SLAM_PARAMS" \
    use_sim_time:="$USE_SIM_TIME"
