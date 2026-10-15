#!/usr/bin/env python3
"""
按顺序循环播放若干个 ros2 数据包（例如一来一回的两个包），用来在没有传感器的设备上长时间测试在线程序。

和 ros2 bag play --loop 的区别：
    - 多个包首尾相接地播，播完最后一个再从第一个开始
    - 改写消息的 header.stamp（livox CustomMsg 的 timebase 一起改），让时间一直往前走、不回跳：
      第一段从当前时间开始，后一段紧接前一段（间隔 --gap 秒），跳过包之间原有的空档
    - 直接改 CDR 字节，不反序列化，播 20 万点/秒的点云也不怎么占 CPU

只支持以 std_msgs/Header 开头、小端 CDR 编码的消息（sensor_msgs/Imu、PointCloud2、livox CustomMsg 等）。

用法：
    source /opt/ros/humble/setup.bash
    source <livox_ros_driver2 的 install>/setup.bash   # 播 CustomMsg 需要它的消息定义
    python3 scripts/bag_loop_player.py <包1> [<包2> ...] [--loops 0] [--rate 1.0] [--topics /livox/lidar /livox/imu]
"""

import argparse
import struct
import sys
import time

import rclpy
from rclpy.qos import QoSProfile
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message

CUSTOM_MSG_TYPE = 'livox_ros_driver2/msg/CustomMsg'
NS = 1000000000


def open_reader(bag, topics):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=''), ConverterOptions('', ''))
    if topics:
        reader.set_filter(StorageFilter(topics=topics))
    return reader


def read_stamp(data):
    """header.stamp（ns），位于 4 字节的 CDR 封装头之后"""
    if data[1] != 1:
        raise RuntimeError('只支持小端 CDR 编码的消息')
    sec, nanosec = struct.unpack_from('<iI', data, 4)
    return sec * NS + nanosec


def restamp(data, offset, msg_type):
    """把 header.stamp 加上 offset（ns）；CustomMsg 的 timebase 同样平移"""
    buf = bytearray(data)
    t = read_stamp(buf) + offset
    struct.pack_into('<iI', buf, 4, t // NS, t % NS)

    if msg_type == CUSTOM_MSG_TYPE:
        # header 之后是 frame_id 字符串，再按 8 字节对齐（相对封装头之后的起点）是 uint64 timebase
        (str_len,) = struct.unpack_from('<I', buf, 12)
        pos = 16 + str_len
        pos += (-(pos - 4)) % 8
        (timebase,) = struct.unpack_from('<Q', buf, pos)
        struct.pack_into('<Q', buf, pos, timebase + offset)

    return bytes(buf), t


def earliest_stamp(bag, topics, n=500):
    """包开头一段消息里最早的 header 时间。点云的时间是帧起点，比它的到达时间早，所以要往后多看一些"""
    reader = open_reader(bag, topics)
    first = None
    for _ in range(n):
        if not reader.has_next():
            break
        _, data, _ = reader.read_next()
        t = read_stamp(data)
        first = t if first is None else min(first, t)
    return first


def main():
    parser = argparse.ArgumentParser(description='按顺序循环播放 ros2 数据包，改写时间戳使其连续')
    parser.add_argument('bags', nargs='+', help='数据包目录，按给定顺序播放')
    parser.add_argument('--loops', type=int, default=0, help='播几轮，0 表示一直播')
    parser.add_argument('--rate', type=float, default=1.0, help='播放倍速')
    parser.add_argument('--gap', type=float, default=0.2, help='相邻两段之间的时间间隔（秒）')
    parser.add_argument('--topics', nargs='*', help='只播这些话题，默认全部')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('bag_loop_player')

    # 每个包里各话题的类型，以及开头的最早时间
    types, first_stamp = {}, {}
    for bag in args.bags:
        reader = open_reader(bag, None)
        for t in reader.get_all_topics_and_types():
            if args.topics and t.name not in args.topics:
                continue
            if types.setdefault(t.name, t.type) != t.type:
                print(f'话题 {t.name} 在不同包里类型不同: {types[t.name]} / {t.type}')
                sys.exit(1)
        first_stamp[bag] = earliest_stamp(bag, args.topics)

    if not types:
        print('没有要播的话题')
        sys.exit(1)

    pubs = {name: node.create_publisher(get_message(t), name, QoSProfile(depth=100)) for name, t in types.items()}
    print('播放话题: ' + ', '.join(f'{n} [{t}]' for n, t in types.items()), flush=True)
    time.sleep(1.0)  # 等订阅方连上

    next_start = time.time_ns()  # 第一段的 header 时间从当前时间开始
    loop = 0
    try:
        while args.loops == 0 or loop < args.loops:
            for bag in args.bags:
                offset = next_start - first_stamp[bag]
                reader = open_reader(bag, args.topics)
                n, last, max_late = 0, next_start, 0.0
                wall0 = t0 = None

                while reader.has_next():
                    topic, data, t_recv = reader.read_next()
                    if wall0 is None:
                        wall0, t0 = time.monotonic(), t_recv

                    # 按包里的到达时间控制节奏
                    delay = wall0 + (t_recv - t0) / NS / args.rate - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    else:
                        max_late = max(max_late, -delay)

                    out, t = restamp(data, offset, types[topic])
                    pubs[topic].publish(out)
                    last = max(last, t)
                    n += 1

                print(f'loop {loop} {bag}: {n} msgs, stamp {next_start / NS:.3f} -> {last / NS:.3f}, '
                      f'wall {time.monotonic() - wall0:.1f} s, max late {max_late:.3f} s', flush=True)

                next_start = last + int(args.gap * NS)
                time.sleep(args.gap / args.rate)
            loop += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
