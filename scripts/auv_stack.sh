#!/usr/bin/env bash
# Jetson AUV 栈一键启停（systemd：comm / imu_lora / ctrl / planner / tracking）。
# 推荐放在工作区根目录：~/sealien_auv_ws/auv_stack.sh
# 用法：
#   ./auv_stack.sh start|stop|restart|status
#   ./auv_stack.sh start|stop|restart <名>   # 单服务
# 名可用: comm imu_lora ctrl planner tracking
set -eo pipefail

SERVICES_START=(comm_bringup imu_lora_bringup ctrl_bringup planner_bringup tracking_bringup)
SERVICES_STOP=(tracking_bringup planner_bringup ctrl_bringup imu_lora_bringup comm_bringup)

usage()
{
    echo "用法:" >&2
    echo "  $(basename "$0") {start|stop|restart|status}" >&2
    echo "  $(basename "$0") {start|stop|restart} <名>" >&2
    echo "名可用: comm imu_lora ctrl planner tracking" >&2
    exit 1
}

need_sudo()
{
    if [[ "${EUID}" -eq 0 ]]; then
        return 0
    fi
    if sudo -n true 2>/dev/null; then
        return 0
    fi
    echo "需要 sudo 权限操作 systemd（请输入密码）。" >&2
}

run_systemctl()
{
    if [[ "${EUID}" -eq 0 ]]; then
        systemctl "$@"
    else
        sudo systemctl "$@"
    fi
}

resolve_service()
{
    local raw="${1:-}"
    local name="${raw%.service}"
    case "${name}" in
        comm|comm_bringup)
            echo "comm_bringup"
            ;;
        imu_lora|imu_lora_bringup)
            echo "imu_lora_bringup"
            ;;
        ctrl|ctrl_bringup)
            echo "ctrl_bringup"
            ;;
        planner|planner_bringup)
            echo "planner_bringup"
            ;;
        tracking|track|tracking_bringup)
            echo "tracking_bringup"
            ;;
        *)
            echo "未知服务名: ${raw}" >&2
            echo "可用: comm imu_lora ctrl planner tracking" >&2
            return 1
            ;;
    esac
}

cmd_status()
{
    local svc
    echo "=== AUV stack status ==="
    for svc in "${SERVICES_START[@]}"; do
        printf "%-18s enabled=%-8s active=%s\n" \
            "${svc}" \
            "$(systemctl is-enabled "${svc}.service" 2>/dev/null || echo unknown)" \
            "$(systemctl is-active "${svc}.service" 2>/dev/null || echo unknown)"
    done
}

cmd_stop_all()
{
    local svc
    need_sudo
    echo "stopping AUV stack (tracking → planner → ctrl → imu_lora → comm) ..."
    for svc in "${SERVICES_STOP[@]}"; do
        echo "  stop ${svc}"
        run_systemctl stop "${svc}.service" || true
    done
    cmd_status
}

cmd_start_all()
{
    local svc
    need_sudo
    echo "starting AUV stack (comm → imu_lora → ctrl → planner → tracking) ..."
    for svc in "${SERVICES_START[@]}"; do
        echo "  start ${svc}"
        run_systemctl start "${svc}.service"
    done
    sleep 3
    cmd_status
}

cmd_stop_one()
{
    local svc
    svc="$(resolve_service "$1")" || exit 1
    need_sudo
    echo "stop ${svc}"
    run_systemctl stop "${svc}.service" || true
    cmd_status
}

cmd_start_one()
{
    local svc
    svc="$(resolve_service "$1")" || exit 1
    need_sudo
    echo "start ${svc}"
    run_systemctl start "${svc}.service"
    sleep 2
    cmd_status
}

cmd_restart_one()
{
    local svc
    svc="$(resolve_service "$1")" || exit 1
    need_sudo
    echo "restart ${svc}"
    run_systemctl restart "${svc}.service"
    sleep 2
    cmd_status
}

cmd_restart_all()
{
    cmd_stop_all
    echo
    cmd_start_all
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
    usage
fi

case "$1" in
    start)
        if [[ $# -eq 1 ]]; then
            cmd_start_all
        else
            cmd_start_one "$2"
        fi
        ;;
    stop)
        if [[ $# -eq 1 ]]; then
            cmd_stop_all
        else
            cmd_stop_one "$2"
        fi
        ;;
    restart)
        if [[ $# -eq 1 ]]; then
            cmd_restart_all
        else
            cmd_restart_one "$2"
        fi
        ;;
    status)
        if [[ $# -ne 1 ]]; then
            usage
        fi
        cmd_status
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage
        ;;
esac
