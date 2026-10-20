#!/bin/bash
# 等机器人停稳后，把 ~/lightning_lio 换成 /tmp/lio_deploy 里的新版本并重启服务（日志写到 ~/lightning_lio_backup/deploy.log）
# 重启会让里程计原点重置；LIO 启动时用最初 0.1 秒的加计估计重力方向，机器人在动时启动会带来几度的倾角误差，所以等它停稳。
# 现场是吊轨机器人，停下后会晃，1 秒内加计波动常在 4–10 mg，长时间才降到 1–2 mg；这里取 10 mg、3 秒，倾角误差不到 1°。
# 用法（在目标板子上）：
#   mkdir -p /tmp/lio_deploy && 拷入 lightning_lio_pkg.tgz、lio_field.yaml、still_check.py、本脚本 && tar xzf lightning_lio_pkg.tgz
#   systemd-run --user --unit=lightning-lio-deploy /tmp/lio_deploy/deploy_when_still.sh   # 断开 SSH 也会继续等
S=/tmp/lio_deploy; L=~/lightning_lio; LOG=~/lightning_lio_backup/deploy.log
log() { echo "$(date '+%F %T') $*" >> $LOG; }
source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0
log "waiting for the robot to be calm: < 10 mg for 3 s (new $(cat $S/lightning_lio/VERSION))"
deadline=$(( $(date +%s) + 7200 ))
until python3 $S/still_check.py 3 0.010 >> $LOG 2>&1; do
    [ $(date +%s) -gt $deadline ] && { log "gave up: robot did not stay still within 2 h"; exit 1; }
    sleep 5
done
log "still, deploying"
OLD=$(cut -d' ' -f1 $L/VERSION); B=~/lightning_lio_backup/$OLD; mkdir -p $B
cp -a $L/bin $L/lib $L/VERSION $L/config $B/
systemctl --user stop lightning-lio
rm -rf $L/bin $L/lib && mv $S/lightning_lio/bin $S/lightning_lio/lib $L/ && cp $S/lightning_lio/VERSION $L/VERSION
cp $S/lio_field.yaml $L/config/lio_field.yaml
systemctl --user start lightning-lio
log "started $(cat $L/VERSION), old version backed up in $B"
