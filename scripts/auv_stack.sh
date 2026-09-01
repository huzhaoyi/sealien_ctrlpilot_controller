#!/usr/bin/env bash
# Jetson AUV 栈一键启停（systemd：comm / rov / ctrl）。
# 推荐放在工作区根目录：~/sealien_auv_ws/auv_stack.sh
# 用法：
#   ./auv_stack.sh start
#   ./auv_stack.sh stop
#   ./auv_stack.sh restart
#   ./auv_stack.sh status
set -eo pipefail

SERVICES_START=(comm_bringup rov_bringup ctrl_bringup)
SERVICES_STOP=(ctrl_bringup rov_bringup comm_bringup)

usage()
{
    echo "用法: $(basename "$0") {start|stop|restart|status}" >&2
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

cmd_status()
{
    local svc
    echo "=== AUV stack status ==="
    for svc in "${SERVICES_START[@]}"; do
        printf "%-16s enabled=%-8s active=%s\n" \
            "${svc}" \
            "$(systemctl is-enabled "${svc}.service" 2>/dev/null || echo unknown)" \
            "$(systemctl is-active "${svc}.service" 2>/dev/null || echo unknown)"
    done
}

cmd_stop()
{
    local svc
    need_sudo
    echo "stopping AUV stack (ctrl → rov → comm) ..."
    for svc in "${SERVICES_STOP[@]}"; do
        echo "  stop ${svc}"
        run_systemctl stop "${svc}.service" || true
    done
    cmd_status
}

cmd_start()
{
    local svc
    need_sudo
    echo "starting AUV stack (comm → rov → ctrl) ..."
    for svc in "${SERVICES_START[@]}"; do
        echo "  start ${svc}"
        run_systemctl start "${svc}.service"
    done
    # ctrl_bringup 有 ExecStartPre sleep，稍等再查状态
    sleep 3
    cmd_status
}

cmd_restart()
{
    cmd_stop
    echo
    cmd_start
}

if [[ $# -ne 1 ]]; then
    usage
fi

case "$1" in
    start)
        cmd_start
        ;;
    stop)
        cmd_stop
        ;;
    restart)
        cmd_restart
        ;;
    status)
        cmd_status
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage
        ;;
esac
