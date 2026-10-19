#!/bin/bash
# 启动持续运行的里程计 run_lio_online，由 systemd 用户服务 lightning-lio 调用
# 目录结构见 doc/lio_online.md：bin/ lib/ config/lio_field.yaml records/ ros_log/
D=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOG_DIR=$D/ros_log   # ros 日志（每分钟一行状态）放在这里，不进 ~/.ros
IMU_TOPIC=${LIO_IMU_TOPIC:-/livox/imu}

# 开机时用户服务可能比网络先起来。DDS 参与者如果在网卡配好地址之前创建，之后就一直发现不了别的节点、
# 收不到任何数据（2026-10-17 现场板子重启后出现过，空跑了两天多）。所以先确认真的能收到 IMU 再启动。
# 每次检查都用新的参与者（--no-daemon），网络起来后就能通过
n=0
until timeout 10 ros2 topic echo --once --no-daemon "$IMU_TOPIC" sensor_msgs/msg/Imu > /dev/null 2>&1; do
    n=$((n + 1))
    if [ $((n % 12)) -eq 1 ]; then
        echo "waiting for data on $IMU_TOPIC (attempt $n)"
    fi
    sleep 5
done
echo "$IMU_TOPIC is publishing, starting run_lio_online"

export LD_LIBRARY_PATH=$D/lib:$LD_LIBRARY_PATH
# glog 只写 stderr（进 journald，自动轮转），不在 /tmp 下堆日志文件
exec $D/bin/run_lio_online --config $D/config/lio_field.yaml --logtostderr=1
