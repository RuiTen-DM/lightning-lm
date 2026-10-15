#!/usr/bin/env python3
"""
将 ros2 数据包中 livox_ros_driver2 输出的 PointCloud2 点云（xfer_format=0）
转换为 livox_ros_driver2/msg/CustomMsg（xfer_format=1），其余话题原样拷贝。

PointCloud2 中每个点的 timestamp 为绝对时间（ns），转换后：
    timebase    = header 时间（ns）
    offset_time = timestamp - timebase
    reflectivity = intensity

用法：
    source /opt/ros/humble/setup.bash
    python3 scripts/livox_pc2_to_custom.py <输入包目录> [-o 输出包目录] [--topic /livox/lidar]
"""

import argparse
import os
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, SequentialWriter, StorageOptions, TopicMetadata
from sensor_msgs.msg import PointCloud2

CUSTOM_MSG_TYPE = 'livox_ros_driver2/msg/CustomMsg'

# sensor_msgs/PointField 的 datatype 到 numpy 类型（小端）
POINT_FIELD_DTYPES = {1: '<i1', 2: '<u1', 3: '<i2', 4: '<u2', 5: '<i4', 6: '<u4', 7: '<f4', 8: '<f8'}

# CustomPoint 的 CDR 布局：uint32 offset_time, float32 x/y/z, uint8 reflectivity/tag/line，
# 下一个点的 offset_time 需 4 字节对齐，因此每个点占 20 字节（末尾 1 字节填充）
CUSTOM_POINT_DTYPE = np.dtype({
    'names': ['offset_time', 'x', 'y', 'z', 'reflectivity', 'tag', 'line'],
    'formats': ['<u4', '<f4', '<f4', '<f4', 'u1', 'u1', 'u1'],
    'offsets': [0, 4, 8, 12, 16, 17, 18],
    'itemsize': 20,
})

UINT32_MAX = np.iinfo(np.uint32).max


def align(buf, n):
    """CDR 对齐，偏移量相对于封装头之后的数据起点"""
    pad = (-(len(buf) - 4)) % n
    buf.extend(b'\x00' * pad)


def serialize_custom_msg(sec, nanosec, frame_id, timebase, lidar_id, points):
    """按 CDR（小端）手动序列化 CustomMsg，比逐点构造 python 对象快得多"""
    buf = bytearray(b'\x00\x01\x00\x00')  # CDR_LE 封装头

    # std_msgs/Header
    buf += np.array([sec], '<i4').tobytes() + np.array([nanosec], '<u4').tobytes()
    frame = frame_id.encode() + b'\x00'
    buf += np.array([len(frame)], '<u4').tobytes() + frame

    align(buf, 8)
    buf += np.array([timebase], '<u8').tobytes()
    align(buf, 4)
    buf += np.array([len(points)], '<u4').tobytes()  # point_num
    buf += bytes([lidar_id, 0, 0, 0])  # lidar_id + rsvd[3]

    align(buf, 4)
    buf += np.array([len(points)], '<u4').tobytes()  # points 序列长度
    if len(points) > 0:
        buf += points.tobytes()[:-1]  # 去掉最后一个点的填充字节
    return bytes(buf)


class Converter:
    def __init__(self, lidar_id):
        self.lidar_id = lidar_id
        self.time_scale = None  # timestamp 字段换算到 ns 的倍数，由第一帧确定
        self.n_clamped = 0

    def field_view(self, raw, msg, name, n):
        """按字段的偏移和 point_step 取出一列，兼容 livox 驱动的非对齐打包布局"""
        field = next((f for f in msg.fields if f.name == name), None)
        if field is None:
            return None
        dtype = np.dtype(POINT_FIELD_DTYPES[field.datatype])
        return np.ndarray(shape=(n,), dtype=dtype, buffer=raw, offset=field.offset, strides=(msg.point_step,))

    def convert(self, data):
        msg = deserialize_message(data, PointCloud2)
        if msg.is_bigendian:
            raise RuntimeError('不支持大端序的 PointCloud2')

        n = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.height > 1 and msg.row_step != msg.width * msg.point_step:
            # 有行填充时去掉每行末尾的填充字节
            raw = raw.reshape(msg.height, msg.row_step)[:, :msg.width * msg.point_step].copy().reshape(-1)

        timestamp = self.field_view(raw, msg, 'timestamp', n)
        if timestamp is None:
            raise RuntimeError('PointCloud2 中没有 timestamp 字段，无法得到每个点的时间')

        head_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        timestamp = timestamp.astype(np.float64)

        if self.time_scale is None and n > 0:
            t_max = float(timestamp.max())
            if t_max > 1e17:
                self.time_scale = 1.0  # 绝对时间，ns（livox_ros_driver2 默认）
            elif t_max > 1e8:
                self.time_scale = 1e9  # 绝对时间，s
            else:
                raise RuntimeError(f'无法识别 timestamp 的单位（最大值 {t_max}），只支持绝对时间的 ns 或 s')

        points = np.zeros(n, dtype=CUSTOM_POINT_DTYPE)
        if n > 0:
            # 在 float64 下作差：ns 绝对时间的精度约 256ns，对点云去畸变足够
            offset = np.rint(timestamp * self.time_scale - float(head_ns))
            clamped = (offset < 0) | (offset > UINT32_MAX)
            self.n_clamped += int(clamped.sum())
            points['offset_time'] = np.clip(offset, 0, UINT32_MAX).astype(np.uint32)

            for name in ('x', 'y', 'z'):
                points[name] = self.field_view(raw, msg, name, n)

            intensity = self.field_view(raw, msg, 'intensity', n)
            if intensity is not None:
                points['reflectivity'] = np.clip(np.rint(intensity.astype(np.float64)), 0, 255).astype(np.uint8)

            for name in ('tag', 'line'):
                col = self.field_view(raw, msg, name, n)
                if col is not None:
                    points[name] = col

        return serialize_custom_msg(msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id, head_ns,
                                    self.lidar_id, points)


def main():
    parser = argparse.ArgumentParser(description='livox PointCloud2 转 CustomMsg')
    parser.add_argument('input_bag', help='输入的 ros2 数据包目录')
    parser.add_argument('-o', '--output', help='输出数据包目录，默认为 <输入>_custom')
    parser.add_argument('--topic', default='/livox/lidar', help='要转换的 PointCloud2 话题')
    parser.add_argument('--out-topic', help='转换后的话题名，默认与 --topic 相同')
    parser.add_argument('--lidar-id', type=int, default=0, help='CustomMsg 中的 lidar_id')
    args = parser.parse_args()

    input_bag = os.path.normpath(args.input_bag)
    output_bag = args.output or input_bag + '_custom'
    out_topic = args.out_topic or args.topic

    if os.path.exists(output_bag):
        print(f'输出目录已存在: {output_bag}，请删除或用 -o 指定其他目录')
        sys.exit(1)

    reader = SequentialReader()
    reader.open(StorageOptions(uri=input_bag, storage_id=''), ConverterOptions('', ''))
    topics = {t.name: t for t in reader.get_all_topics_and_types()}

    if args.topic not in topics:
        print(f'包中没有话题 {args.topic}，现有话题:')
        for t in topics.values():
            print(f'  {t.name}  [{t.type}]')
        sys.exit(1)

    src_type = topics[args.topic].type
    if src_type != 'sensor_msgs/msg/PointCloud2':
        print(f'{args.topic} 的类型是 {src_type}，不是 PointCloud2，无需转换')
        sys.exit(1)

    if out_topic != args.topic and out_topic in topics:
        print(f'输出话题 {out_topic} 在包中已存在，请换一个 --out-topic')
        sys.exit(1)

    # lightning 的离线程序按 sqlite3 读取数据包
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=output_bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    for t in topics.values():
        if t.name == args.topic:
            writer.create_topic(TopicMetadata(name=out_topic, type=CUSTOM_MSG_TYPE, serialization_format='cdr',
                                              offered_qos_profiles=t.offered_qos_profiles))
        else:
            writer.create_topic(t)

    converter = Converter(args.lidar_id)
    n_frames = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == args.topic:
            writer.write(out_topic, converter.convert(data), t)
            n_frames += 1
            if n_frames % 100 == 0:
                print(f'已转换 {n_frames} 帧')
        else:
            writer.write(topic, data, t)

    del writer  # 关闭并写出 metadata.yaml

    print(f'转换完成: {n_frames} 帧 {args.topic} -> {out_topic} [{CUSTOM_MSG_TYPE}]')
    if converter.n_clamped > 0:
        print(f'警告: {converter.n_clamped} 个点的时间早于 header 或超出 uint32 范围，已截断')
    print(f'输出: {output_bag}')


if __name__ == '__main__':
    main()
