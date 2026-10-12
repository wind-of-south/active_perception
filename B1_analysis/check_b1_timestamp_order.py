#!/usr/bin/env python3
"""比较 B1 原始传感器与 FAST-LIO 输出 rosbag 的 header 时间顺序。"""

import argparse
import struct
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_topics(bag_path: Path, topics: dict[str, str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    message_classes = {
        topic: get_message(message_type) for topic, message_type in topics.items()
    }
    header_stamps = {topic: [] for topic in topics}
    recorded_stamps = {topic: [] for topic in topics}
    verified_topics = set()
    while reader.has_next():
        topic, serialized, recorded_ns = reader.read_next()
        if topic not in topics:
            continue
        sec, nanosec = struct.unpack_from("<iI", serialized, 4)
        if topic not in verified_topics:
            message = deserialize_message(serialized, message_classes[topic])
            stamp = message.header.stamp
            if (sec, nanosec) != (stamp.sec, stamp.nanosec):
                raise RuntimeError(f"CDR header parse mismatch for {topic}")
            verified_topics.add(topic)
        header_stamps[topic].append(sec + nanosec * 1e-9)
        recorded_stamps[topic].append(recorded_ns * 1e-9)
    results = {}
    for topic in topics:
        if not header_stamps[topic]:
            raise RuntimeError(f"No messages for {topic} in {bag_path}")
        results[topic] = (
            np.asarray(header_stamps[topic]),
            np.asarray(recorded_stamps[topic]),
        )
    return results


def format_topic(label: str, topic: str, headers: np.ndarray, recorded: np.ndarray):
    header_intervals = np.diff(headers)
    recorded_intervals = np.diff(recorded)
    return (
        f"{label} {topic} ({len(headers)} 条):\n"
        f"  header 时间非递增数：{int(np.sum(header_intervals <= 0))} "
        f"(倒退 {int(np.sum(header_intervals < 0))}，重复 "
        f"{int(np.sum(header_intervals == 0))})\n"
        f"  header 时间间隔最小/最大：{header_intervals.min():.9f} / "
        f"{header_intervals.max():.9f} 秒\n"
        f"  rosbag 记录时间非递增数："
        f"{int(np.sum(recorded_intervals <= 0))}"
    )


def main():
    parser = argparse.ArgumentParser()
    default_dir = Path(__file__).resolve().parent
    parser.add_argument(
        "--source-bag",
        type=Path,
        default=default_dir.parent
        / "交付"
        / "B1_static_10min_2026-09-23",
    )
    parser.add_argument(
        "--output-bag",
        type=Path,
        default=default_dir / "B1_fastlio_odometry_slow",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_dir / "B1_timestamp_order.txt",
    )
    args = parser.parse_args()

    source_topics = read_topics(
        args.source_bag,
        {
            "/livox/lidar": "livox_ros_driver2/msg/CustomMsg",
            "/livox/imu": "sensor_msgs/msg/Imu",
        },
    )
    output_topics = read_topics(
        args.output_bag,
        {"/Odometry": "nav_msgs/msg/Odometry"},
    )
    results = [
        "B1 输入与 FAST-LIO 输出时间戳顺序检查",
        "====================================",
        "用途：对比原始 B1 雷达/IMU 与 FAST-LIO /Odometry 的 header 时间顺序，",
        "用于定位时间戳异常出现在哪个记录阶段；此检查不单独诊断异常根因。",
        "",
        format_topic(
            "原始 B1",
            "/livox/lidar",
            *source_topics["/livox/lidar"],
        ),
        format_topic(
            "原始 B1",
            "/livox/imu",
            *source_topics["/livox/imu"],
        ),
        format_topic(
            "FAST-LIO 输出",
            "/Odometry",
            *output_topics["/Odometry"],
        ),
        "",
        "判读：原始 B1 雷达与 IMU 的 header 时间严格递增；FAST-LIO",
        "输出 /Odometry 的 header 时间存在倒退。该检查能确认异常在",
        "输出记录中出现，但不能单独确定是算法调度、仿真时钟处理还是",
        "下游发布/记录造成；不要据此宣称已找到根因。",
    ]
    args.output.write_text("\n".join(results) + "\n", encoding="utf-8")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
