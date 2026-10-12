#!/usr/bin/env python3
"""分析 B1 FAST-LIO 输出，生成离线静止位姿波动报告、分段 CSV 和曲线图。"""

import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation


def read_odometry(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    odometry_type = get_message("nav_msgs/msg/Odometry")
    timestamps = []
    recording_timestamps = []
    positions = []
    quaternions = []
    frames = set()
    child_frames = set()
    while reader.has_next():
        topic, serialized, recorded_ns = reader.read_next()
        if topic != "/Odometry":
            continue
        message = deserialize_message(serialized, odometry_type)
        timestamps.append(
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        )
        recording_timestamps.append(recorded_ns * 1e-9)
        positions.append(
            (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            )
        )
        q = message.pose.pose.orientation
        quaternions.append((q.x, q.y, q.z, q.w))
        frames.add(message.header.frame_id)
        child_frames.add(message.child_frame_id)
    return (
        np.asarray(timestamps),
        np.asarray(recording_timestamps),
        np.asarray(positions),
        np.asarray(quaternions),
        frames,
        child_frames,
    )


def orientation_summary(quaternions):
    rotations = Rotation.from_quat(quaternions)
    mean = rotations.mean()
    residuals = (mean.inv() * rotations).as_rotvec()
    residual_degrees = np.rad2deg(residuals)
    return {
        "mean_quaternion_xyzw": mean.as_quat(),
        "std_deg": residual_degrees.std(axis=0, ddof=1),
        "range_deg": np.rad2deg(
            np.max(np.linalg.norm(residuals, axis=1))
            - np.min(np.linalg.norm(residuals, axis=1))
        ),
        "max_abs_deg": np.max(np.abs(residual_degrees), axis=0),
    }


def vec(values):
    return "[" + ", ".join(f"{value:.8g}" for value in values) + "]"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "bag",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().parent / "B1_fastlio_odometry",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent,
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    stamps, recorded_stamps, positions, quaternions, frames, child_frames = (
        read_odometry(args.bag)
    )
    if len(stamps) < 2:
        raise RuntimeError(f"Expected at least two /Odometry messages in {args.bag}")
    raw_stamps = stamps.copy()
    raw_recorded_stamps = recorded_stamps.copy()
    chronological_order = np.argsort(stamps, kind="stable")
    stamps = stamps[chronological_order]
    recorded_stamps = recorded_stamps[chronological_order]
    positions = positions[chronological_order]
    quaternions = quaternions[chronological_order]
    relative_time = stamps - stamps[0]
    sample_dt = np.diff(stamps)
    raw_header_dt = np.diff(raw_stamps)
    header_nonincreasing = int(np.sum(raw_header_dt <= 0))
    sorted_duplicate_stamps = int(np.sum(sample_dt <= 0))
    header_long_gaps = int(np.sum(sample_dt > 0.2))
    recording_duration = (
        raw_recorded_stamps.max() - raw_recorded_stamps.min()
    )
    measurement_status = (
        "原始输出时间戳有异常；这里只作离线静态分布统计，不验收实时性"
        if header_nonincreasing or header_long_gaps
        else "header 时间戳顺序正常"
    )
    overall_orientation = orientation_summary(quaternions)
    position_mean = positions.mean(axis=0)
    position_std = positions.std(axis=0, ddof=1)
    position_centered = positions - position_mean
    radial = np.linalg.norm(position_centered, axis=1)
    radial_rms = np.sqrt(np.mean(radial**2))
    net_displacement = np.linalg.norm(positions[-1] - positions[0])
    imu_to_base = np.array((-0.2285, -0.02329, -0.18188))
    base_positions = positions + Rotation.from_quat(quaternions).apply(imu_to_base)
    base_mean = base_positions.mean(axis=0)
    base_std = base_positions.std(axis=0, ddof=1)
    base_radial = np.linalg.norm(base_positions - base_mean, axis=1)
    base_net_displacement = np.linalg.norm(base_positions[-1] - base_positions[0])
    settled = relative_time >= 60
    settled_base_positions = base_positions[settled]
    settled_quaternions = quaternions[settled]
    settled_base_std = settled_base_positions.std(axis=0, ddof=1)
    settled_orientation = orientation_summary(settled_quaternions)
    settled_base_drift = np.linalg.norm(
        settled_base_positions[-1] - settled_base_positions[0]
    )

    csv_path = args.output_dir / "B1_fastlio_1min.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "分段序号",
                "起始相对秒",
                "结束相对秒",
                "样本数",
                "位置均值_x_m",
                "位置均值_y_m",
                "位置均值_z_m",
                "位置标准差_x_m",
                "位置标准差_y_m",
                "位置标准差_z_m",
                "姿态标准差_roll_deg",
                "姿态标准差_pitch_deg",
                "姿态标准差_yaw_deg",
                "相对均值姿态最大绝对偏差_deg",
            ]
        )
        for start in np.arange(0, relative_time[-1], 60):
            stop = min(start + 60, relative_time[-1])
            mask = (relative_time >= start) & (relative_time < stop)
            if mask.sum() < 2:
                continue
            orientation = orientation_summary(quaternions[mask])
            writer.writerow(
                [
                    int(start // 60) + 1,
                    f"{start:.3f}",
                    f"{stop:.3f}",
                    int(mask.sum()),
                    *[f"{value:.9g}" for value in base_positions[mask].mean(axis=0)],
                    *[
                        f"{value:.9g}"
                        for value in base_positions[mask].std(axis=0, ddof=1)
                    ],
                    *[f"{value:.9g}" for value in orientation["std_deg"]],
                    f"{np.max(orientation['max_abs_deg']):.9g}",
                ]
            )

    pose_plot_path = args.output_dir / "B1_fastlio_pose.png"
    figure, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for label, values in zip(("X", "Y", "Z"), base_positions.T):
        axes[0].plot(
            relative_time, (values - np.median(values)) * 1000, label=label
        )
    axes[0].set_ylabel("Base position from median (mm)")
    axes[0].legend(ncol=3)
    axes[0].grid(True, linestyle=":")
    rotations = Rotation.from_quat(quaternions)
    relative_rotations = (rotations.mean().inv() * rotations).as_rotvec()
    for label, values in zip(("Roll", "Pitch", "Yaw"), relative_rotations.T):
        axes[1].plot(relative_time, np.rad2deg(values), label=label)
    axes[1].set_xlabel("B1 elapsed time from first odometry stamp (s)")
    axes[1].set_ylabel("Rotation vector (deg)")
    axes[1].legend(ncol=3)
    axes[1].grid(True, linestyle=":")
    figure.suptitle(
        "B1 FAST-LIO Static Pose Variation (Nominal Extrinsic, Header-Time Sorted)"
    )
    figure.tight_layout()
    figure.savefig(pose_plot_path, dpi=160)
    plt.close(figure)

    report_path = args.output_dir / "B1_fastlio_static_jitter.txt"
    with report_path.open("w", encoding="utf-8") as report:
        report.write(
            f"""B1 静止场景 FAST-LIO 位姿波动测量
=================================
用途：统计 B1 半速 FAST-LIO 输出的离线静止位姿分布和逐分钟波动。
这不是独立真值误差；输出时间戳有倒退，因此本报告不用于实时性验收。

数据顺序检查：{measurement_status}

轨迹数据：{args.bag}
输入数据：B1 静止包；回放时使用 /clock，FAST-LIO use_sim_time=true
里程计话题：/Odometry
frame_id：{", ".join(sorted(frames))}
child_frame_id：{", ".join(sorted(child_frames))}
里程计样本数：{len(stamps)}
header 时间戳覆盖：{relative_time[-1]:.6f} 秒（{relative_time[-1] / 60:.3f} 分钟）
rosbag 实际记录到达覆盖：{recording_duration:.6f} 秒（{recording_duration / 60:.3f} 分钟）
采样间隔均值/中位数/最大值：{sample_dt.mean():.6f} / {np.median(sample_dt):.6f} / {sample_dt.max():.6f} 秒
原始消息顺序中的 header 非递增数：{header_nonincreasing}
按时间排序后的重复 header 时间戳数：{sorted_duplicate_stamps}
header 相邻时间戳间隔超过 0.2 秒数：{header_long_gaps}

FAST-LIO 原始 body（雷达 IMU）位置分布（单位 m）：
位置均值：{vec(position_mean)}
位置标准差（X、Y、Z）：{vec(position_std)}
三维位置相对均值的 RMS：{radial_rms:.8g}
三维位置相对均值的最大距离：{radial.max():.8g}

按名义 TF 外参换算至 base_link 的位置分布（单位 m）：
使用的 IMU -> base_link 平移（m）：{vec(imu_to_base)}
位置均值：{vec(base_mean)}
位置标准差（X、Y、Z）：{vec(base_std)}
三维位置相对均值的最大距离：{base_radial.max():.8g}
起点至终点的位置变化量：{vec(base_positions[-1] - base_positions[0])}
起点至终点的位移模长：{base_net_displacement:.8g}
参考目标：各轴位置标准差 ≤ 0.002 m；下列姿态标准差 ≤ 0.05 度。
当前 base_link 位置标准差目标检查：{"通过" if np.max(base_std) <= 0.002 else "未通过"}
剔除启动首分钟后的 base_link 位置标准差：{vec(settled_base_std)} m
剔除启动首分钟后的 base_link 位移模长：{settled_base_drift:.8g} m
剔除启动首分钟后的姿态标准差：{vec(settled_orientation['std_deg'])} 度
剔除启动首分钟后的位置目标检查：{"通过" if np.max(settled_base_std) <= 0.002 else "未通过"}
剔除启动首分钟后的姿态目标检查：{"通过" if np.max(settled_orientation['std_deg']) <= 0.05 else "未通过"}

相对平均姿态的旋转向量统计（近似 roll、pitch、yaw 分量，单位 deg）：
标准差：{vec(overall_orientation['std_deg'])}
各分量最大绝对偏差：{vec(overall_orientation['max_abs_deg'])}
单位四元数均值 (x,y,z,w)：{vec(overall_orientation['mean_quaternion_xyzw'])}
姿态标准差目标检查：{"通过" if np.max(overall_orientation['std_deg']) <= 0.05 else "未通过"}

分段统计见 B1_fastlio_1min.csv。
时序变化图见 B1_fastlio_pose.png。

判读限制：
1. 这是所记录 FAST-LIO /Odometry 的波动，不是独立真值误差；FAST-LIO
   输出 frame 为 camera_init -> body，body 并非 base_link。
2. 包录制说明称车辆静止，但场景中可能有人走动；测量包含雷达/IMU
   噪声、地面振动、动态物体、算法初始化和地图效应。
3. 逐分钟统计按 header 时间戳排序分段；全程位置标准差会混入慢漂移，
   应同时查看分段数据和起终点变化。
4. base_link 结果使用 README/TF 中尚待标定的名义安装外参，仅为参考；
   外参标定后需重新计算。
5. 若原始 header 时间戳倒退或出现大间隔，静态分布仍可作为离线诊断，
   但这次回放不能证明实时连续性，不能当成最终在线验收。
"""
        )
    print(f"Wrote {report_path}")
    print(f"Wrote {csv_path}")


if __name__ == "__main__":
    main()
