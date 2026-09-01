#!/usr/bin/env bash
# Jetson 开机自启：OBC（LoRa joy→twist）+ controller + msgs_adapter。
# 依赖已有 comm_bringup（MAVLink）与 rov_bringup（LoRa RX /joy）。
set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/sealien/sealien_auv_ws/install/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ERROR: 找不到 ROS: ${ROS_SETUP}" >&2
    exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
    echo "ERROR: 找不到工作区: ${WS_SETUP}" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WS_SETUP}"
set -u

CHILD_PIDS=()

cleanup()
{
    local pid
    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -INT "${pid}" 2>/dev/null || true
        fi
    done
    sleep 1
    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -TERM "${pid}" 2>/dev/null || true
        fi
    done
    sleep 0.5
    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -KILL "${pid}" 2>/dev/null || true
        fi
    done
}

trap cleanup EXIT INT TERM

echo "starting OBC (lora zorro), ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
ros2 launch sealien_ctrlpilot_onboardcontrol onboardcontrol_lora_zorro.launch.py &
CHILD_PIDS+=("$!")

echo "starting msgs_adapter"
ros2 launch sealien_ctrlpilot_controller msgs_adapter.launch.py &
CHILD_PIDS+=("$!")

echo "starting controller"
ros2 launch sealien_ctrlpilot_controller sealien_ctrlpilot_controller.launch.py &
CHILD_PIDS+=("$!")

# 任一子进程退出则结束，由 systemd Restart=on-failure 拉起。
wait -n
exit 1
