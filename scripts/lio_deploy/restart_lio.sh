#!/bin/bash
# 重启里程计 lightning-lio（systemd 用户服务），放在 ~/lightning_lio/ 下，和 still_check.py 同目录
#
# 默认等机器人停稳再重启：重启会让里程计原点重置；启动时用最初 0.1 秒的加计估计重力方向，
# 在动的时候启动会让坐标系倾斜几度。"停稳"指 3 秒内每个 1 秒窗口的加计模长波动都小于 10 mg
# （吊轨机器人停下后会晃，长时间停放才降到 1–2 mg，10 mg 时倾角误差不到 1°）。
#
# 用法：
#   restart_lio.sh              等停稳再重启，最多等 30 分钟，没等到就放弃（不重启）
#   restart_lio.sh --wait 60    最多等 60 分钟
#   restart_lio.sh --now        不等，立即重启
#   restart_lio.sh --bg         放到后台等（断开 SSH 也会继续），过程写到 ~/lightning_lio/restart.log
#   restart_lio.sh --dry-run    只等停稳并报告，不真的重启（测试用）
# 远程：
#   ssh -p <端口> <用户>@<板子> lightning_lio/restart_lio.sh --bg

D=$(cd "$(dirname "$0")" && pwd)
WAIT_MIN=30
NOW=0
BG=0
DRY=0
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --now) NOW=1; ARGS+=("$1") ;;
        --wait) WAIT_MIN="$2"; ARGS+=("$1" "$2"); shift ;;
        --bg) BG=1 ;;
        --dry-run) DRY=1; ARGS+=("$1") ;;
        -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "未知参数: $1（见 --help）"; exit 2 ;;
    esac
    shift
done

log() { echo "$(date '+%F %T') $*"; }

if [ $BG -eq 1 ]; then
    UNIT=lightning-lio-restart-$(date +%s)
    systemd-run --user -q --unit="$UNIT" --collect -p StandardOutput=append:"$D/restart.log" \
        -p StandardError=append:"$D/restart.log" "$D/restart_lio.sh" "${ARGS[@]}" > /dev/null || exit 1
    log "已在后台运行（$UNIT），过程见 $D/restart.log"
    exit 0
fi

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

if [ $NOW -eq 0 ]; then
    log "等机器人停稳（3 秒内加计波动 < 10 mg），最多等 $WAIT_MIN 分钟"
    deadline=$(( $(date +%s) + WAIT_MIN * 60 ))
    n=0
    until out=$(python3 "$D/still_check.py" 3 0.010 2>&1); do
        n=$((n + 1))
        [ $((n % 20)) -eq 1 ] && log "还在动（$out）"
        if [ "$(date +%s)" -gt $deadline ]; then
            log "$WAIT_MIN 分钟内没等到停稳，放弃，没有重启。要立即重启用 --now"
            exit 1
        fi
        sleep 3
    done
    log "已停稳（$out）"
fi

if [ $DRY -eq 1 ]; then
    log "--dry-run：到这里会重启 lightning-lio，实际没有重启"
    exit 0
fi

t0=$(date '+%F %T')
old_pid=$(systemctl --user show lightning-lio -p MainPID --value)
systemctl --user restart lightning-lio || { log "重启失败"; exit 1; }
log "已重启 lightning-lio（原 PID $old_pid），等它收到数据"

# run.sh 先确认 IMU 有数据才启动程序，程序每分钟写一行状态；最多等 3 分钟看到处理了帧
for i in $(seq 1 36); do
    sleep 5
    row=$(awk -v t="$t0" '/^# start/{ok = (substr($0, 9, 19) >= t); next} ok && $0 !~ /^time/' "$D/records/status.csv" | tail -1)
    frames=$(echo "$row" | cut -d, -f2)
    if [ -n "$frames" ] && [ "$frames" -gt 0 ] 2>/dev/null; then
        log "正常：新进程 PID $(systemctl --user show lightning-lio -p MainPID --value)，已处理 $frames 帧（$(echo "$row" | cut -d, -f1)）"
        exit 0
    fi
done

log "3 分钟内还没看到处理帧，最近的日志："
journalctl --user-unit=lightning-lio --since "$t0" --no-pager -n 10 | cut -c1-200
exit 1
