#!/usr/bin/env python3
"""Analyze B2 FAST-LIO output, export trajectories, and plot the trajectory/map."""

import argparse
import sqlite3
import struct
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation


IMU_POSITION_IN_BASE = np.array((0.2285, 0.02329, 0.18188))
IMU_TO_BASE_IN_IMU = -IMU_POSITION_IN_BASE
START_WINDOW = (10.0, 20.0)
END_WINDOW = (160.0, 170.0)
B2_LATERAL_GROUND_TRUTH_M = 0.007


def read_topic_stamps(bag_path: Path, topic: str) -> np.ndarray:
    database_files = sorted(bag_path.glob("*.db3"))
    if len(database_files) != 1:
        raise RuntimeError(
            f"Expected one SQLite bag file in {bag_path}, found {len(database_files)}"
        )
    connection = sqlite3.connect(f"file:{database_files[0]}?mode=ro", uri=True)
    try:
        row = connection.execute(
            "SELECT id FROM topics WHERE name = ?", (topic,)
        ).fetchone()
        if row is None:
            raise RuntimeError(f"Topic {topic} is missing from {bag_path}")
        stamps = []
        for (serialized,) in connection.execute(
            "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (row[0],),
        ):
            sec, nanosec = struct.unpack_from("<iI", serialized, 4)
            stamps.append(sec + nanosec * 1e-9)
        if not stamps:
            raise RuntimeError(f"Topic {topic} has no messages in {bag_path}")
        return np.asarray(stamps)
    finally:
        connection.close()


def read_odometry(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    message_type = get_message("nav_msgs/msg/Odometry")
    stamps, recording_stamps, positions, quaternions = [], [], [], []
    frames, child_frames = set(), set()
    while reader.has_next():
        topic, serialized, recorded_ns = reader.read_next()
        if topic != "/Odometry":
            continue
        message = deserialize_message(serialized, message_type)
        stamp = message.header.stamp
        stamps.append(stamp.sec + stamp.nanosec * 1e-9)
        recording_stamps.append(recorded_ns * 1e-9)
        position = message.pose.pose.position
        positions.append((position.x, position.y, position.z))
        orientation = message.pose.pose.orientation
        quaternions.append(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        frames.add(message.header.frame_id)
        child_frames.add(message.child_frame_id)
    if len(stamps) < 2:
        raise RuntimeError(f"Expected at least two /Odometry messages in {bag_path}")
    return (
        np.asarray(stamps),
        np.asarray(recording_stamps),
        np.asarray(positions),
        np.asarray(quaternions),
        frames,
        child_frames,
    )


def read_pcd_xyz(path: Path):
    with path.open("rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError(f"PCD header is incomplete: {path}")
            decoded = line.decode("ascii").strip()
            if not decoded or decoded.startswith("#"):
                continue
            key, _, value = decoded.partition(" ")
            header[key] = value
            if key == "DATA":
                if value != "binary":
                    raise RuntimeError(f"Expected binary PCD, got DATA {value}")
                data_offset = stream.tell()
                break

    fields = header["FIELDS"].split()
    sizes = [int(value) for value in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = [int(value) for value in header["COUNT"].split()]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise RuntimeError(f"Malformed PCD field metadata: {path}")

    scalar_types = {
        ("F", 4): "<f4",
        ("F", 8): "<f8",
        ("I", 1): "i1",
        ("I", 2): "<i2",
        ("I", 4): "<i4",
        ("I", 8): "<i8",
        ("U", 1): "u1",
        ("U", 2): "<u2",
        ("U", 4): "<u4",
        ("U", 8): "<u8",
    }
    dtype_fields = []
    for name, size, kind, count in zip(fields, sizes, types, counts):
        try:
            scalar_type = scalar_types[(kind, size)]
        except KeyError as error:
            raise RuntimeError(f"Unsupported PCD field type {kind}{size}") from error
        dtype_fields.append(
            (name, scalar_type) if count == 1 else (name, scalar_type, (count,))
        )
    cloud = np.fromfile(
        path,
        dtype=np.dtype(dtype_fields),
        count=int(header["POINTS"]),
        offset=data_offset,
    )
    if len(cloud) != int(header["POINTS"]):
        raise RuntimeError(f"PCD point data is truncated: {path}")
    return np.column_stack((cloud["x"], cloud["y"], cloud["z"])), len(cloud)


def write_tum(path: Path, stamps: np.ndarray, positions: np.ndarray, quaternions):
    with path.open("w", encoding="ascii") as stream:
        for stamp, position, quaternion in zip(stamps, positions, quaternions):
            values = np.concatenate(([stamp], position, quaternion))
            stream.write(" ".join(f"{value:.9f}" for value in values) + "\n")


def vector(values, digits=6):
    return "[" + ", ".join(f"{value:.{digits}f}" for value in values) + "]"


def main():
    default_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-bag",
        type=Path,
        default=default_dir.parent
        / "交付"
        / "B2_teleop_loop_tilt0_2026-09-23",
    )
    parser.add_argument(
        "--odometry-bag", type=Path, default=default_dir / "B2_fastlio_odom_saved"
    )
    parser.add_argument(
        "--map", type=Path, default=default_dir / "B2_fastlio_map.pcd"
    )
    parser.add_argument("--output-dir", type=Path, default=default_dir)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    source_lidar_stamps = read_topic_stamps(
        args.source_bag, "/livox/lidar"
    )
    source_imu_stamps = read_topic_stamps(
        args.source_bag, "/livox/imu"
    )
    source_start = source_lidar_stamps[0]
    stamps, recording_stamps, positions, quaternions, frames, child_frames = (
        read_odometry(args.odometry_bag)
    )
    received_dt = np.diff(stamps)
    backwards = int(np.sum(received_dt < 0))
    repeated = int(np.sum(received_dt == 0))
    nonincreasing = backwards + repeated
    record_nonincreasing = int(np.sum(np.diff(recording_stamps) <= 0))
    chronological = np.argsort(stamps, kind="stable")
    stamps = stamps[chronological]
    positions = positions[chronological]
    quaternions = quaternions[chronological]
    elapsed = stamps - source_start
    sample_dt = np.diff(stamps)
    rotations = Rotation.from_quat(quaternions)
    base_positions = positions + rotations.apply(IMU_TO_BASE_IN_IMU)

    def window_stats(window):
        mask = (elapsed >= window[0]) & (elapsed < window[1])
        if mask.sum() < 2:
            raise RuntimeError(
                f"Not enough odometry samples in B2 interval {window[0]}-{window[1]}s"
            )
        window_rotations = rotations[mask]
        mean_rotation = window_rotations.mean()
        orientation_residuals = (
            mean_rotation.inv() * window_rotations
        ).as_rotvec()
        return {
            "count": int(mask.sum()),
            "mean_position": base_positions[mask].mean(axis=0),
            "position_std": base_positions[mask].std(axis=0, ddof=1),
            "mean_rotation": mean_rotation,
            "orientation_std_deg": np.rad2deg(orientation_residuals).std(
                axis=0, ddof=1
            ),
        }

    start = window_stats(START_WINDOW)
    end = window_stats(END_WINDOW)
    loop_delta = start["mean_rotation"].inv().apply(
        end["mean_position"] - start["mean_position"]
    )
    loop_euler = (start["mean_rotation"].inv() * end["mean_rotation"]).as_euler(
        "xyz", degrees=True
    )
    lateral_error = loop_delta[1] - B2_LATERAL_GROUND_TRUTH_M
    endpoint_xy_distance = float(np.linalg.norm(loop_delta[:2]))

    body_tum = args.output_dir / "B2_fastlio_trajectory_body.tum"
    base_tum = args.output_dir / "B2_fastlio_trajectory_base_nominal.tum"
    write_tum(body_tum, stamps, positions, quaternions)
    write_tum(base_tum, stamps, base_positions, quaternions)

    trajectory_plot = args.output_dir / "B2_fastlio_trajectory.png"
    figure, axes = plt.subplots(1, 2, figsize=(13, 6))
    scatter = axes[0].scatter(
        base_positions[:, 0] - base_positions[0, 0],
        base_positions[:, 1] - base_positions[0, 1],
        c=elapsed,
        s=5,
        cmap="viridis",
        rasterized=True,
    )
    axes[0].scatter(0, 0, marker="o", color="green", label="Start")
    axes[0].scatter(
        base_positions[-1, 0] - base_positions[0, 0],
        base_positions[-1, 1] - base_positions[0, 1],
        marker="x",
        color="red",
        label="End",
    )
    axes[0].set_xlabel("Forward from first pose (m)")
    axes[0].set_ylabel("Left from first pose (m)")
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].grid(True, linestyle=":")
    axes[0].legend()
    figure.colorbar(scatter, ax=axes[0], label="B2 elapsed time (s)")
    axes[1].plot(elapsed, (base_positions - base_positions[0])[:, 2], color="purple")
    axes[1].set_xlabel("B2 elapsed time (s)")
    axes[1].set_ylabel("Nominal base_link height change (m)")
    axes[1].grid(True, linestyle=":")
    figure.suptitle("B2 FAST-LIO Trajectory (Nominal IMU-to-base_link Extrinsic)")
    figure.tight_layout()
    figure.savefig(trajectory_plot, dpi=160)
    plt.close(figure)

    map_points, map_count = read_pcd_xyz(args.map)
    finite = np.isfinite(map_points).all(axis=1)
    map_points = map_points[finite]
    map_plot = args.output_dir / "B2_fastlio_map.png"
    figure, axis = plt.subplots(figsize=(9, 8))
    point_plot = axis.scatter(
        map_points[:, 0],
        map_points[:, 1],
        c=map_points[:, 2],
        s=0.5,
        cmap="terrain",
        rasterized=True,
    )
    axis.set_xlabel("camera_init X (m)")
    axis.set_ylabel("camera_init Y (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, linestyle=":")
    figure.colorbar(point_plot, ax=axis, label="camera_init Z (m)")
    figure.suptitle(f"B2 FAST-LIO Accumulated Map ({len(map_points):,} finite points)")
    figure.tight_layout()
    figure.savefig(map_plot, dpi=160)
    plt.close(figure)

    report_path = args.output_dir / "B2_fastlio_report.txt"
    report_path.write_text(
        f"""B2 FAST-LIO 单圈回环初步测量
============================
用途：记录 B2 单圈 FAST-LIO 离线运行的时间戳质量、轨迹、地图和
起终点静止段之间的位移估计。FAST-LIO 没有回环优化；此处不等同
于最终闭环验收，也不是独立真值评测。

输入包：{args.source_bag}
FAST-LIO 输出包：{args.odometry_bag}
FAST-LIO 配置：b2_mid360.yaml；/clock 回放，use_sim_time=true；
固定名义 IMU-LiDAR 外参，关闭在线外参估计；PCD 通过 /map_save 写出。

Odometry 样本数：{len(stamps)}
输出坐标系：{", ".join(sorted(frames))} -> {", ".join(sorted(child_frames))}
header 时间覆盖：{elapsed[0]:.6f} 至 {elapsed[-1]:.6f} 秒（相对 B2 首个 LiDAR header）
输出消息原始顺序中的 header 非递增数：{nonincreasing}
原始消息顺序 header 间隔最小/最大：{received_dt.min():.6f} / {received_dt.max():.6f} 秒
按 header 时间排序后，采样间隔均值/中位数/最小/最大：
  {sample_dt.mean():.6f} / {np.median(sample_dt):.6f} /
  {sample_dt.min():.6f} / {sample_dt.max():.6f} 秒
输入 LiDAR header 倒退数：{int(np.sum(np.diff(source_lidar_stamps) < 0))}；
重复数：{int(np.sum(np.diff(source_lidar_stamps) == 0))}
输入 IMU header 倒退数：{int(np.sum(np.diff(source_imu_stamps) < 0))}；
重复数：{int(np.sum(np.diff(source_imu_stamps) == 0))}
输出 header 倒退/重复数：{backwards} / {repeated}
rosbag 接收时间非递增数：{record_nonincreasing}
与输入 LiDAR 帧数对照：输出 {len(stamps)} 条 /Odometry；输入 {len(source_lidar_stamps)} 帧。
输出轨迹按 header 时间排序后导出；由于原始发布顺序有倒退，
该次结果只用于离线轨迹诊断，不能证明实时连续性。

起点静止窗口：B2 相对时间 {START_WINDOW[0]:.0f}-{START_WINDOW[1]:.0f} 秒，
{start['count']} 个样本；终点静止窗口：{END_WINDOW[0]:.0f}-{END_WINDOW[1]:.0f} 秒，
{end['count']} 个样本。窗口依据 IMU 低运动区间选择，不是独立定位真值。
起点窗口位置标准差 X/Y/Z：{vector(start['position_std'] * 1000)} mm
终点窗口位置标准差 X/Y/Z：{vector(end['position_std'] * 1000)} mm
起点窗口姿态标准差：{vector(start['orientation_std_deg'])} 度
终点窗口姿态标准差：{vector(end['orientation_std_deg'])} 度

起终点窗口平均位姿差（转换到起点 IMU/body 轴向，base_link 名义外参）：
前/左/上位移：{vector(loop_delta * 1000)} mm
水平位移模长：{endpoint_xy_distance * 1000:.2f} mm
roll/pitch/yaw 差：{vector(loop_euler)} 度

B2 卷尺真值仅测得“向左 7 mm”，未测前后位移和航向角。
以 +Y 为左，FAST-LIO 估计的左向分量为 {loop_delta[1] * 1000:.2f} mm；
相对已测左向真值的差为 {lateral_error * 1000:.2f} mm。
这只是单一分量对照，不足以判定完整 3 cm 位置 / 0.5 度航向目标通过与否；
名义外参未标定、起终点窗口选择及环境动态也会影响结果。

PCD 地图：{args.map}
地图点数：{map_count}（有限 XYZ 点 {len(map_points)}）
地图坐标系：camera_init；地图通过 FAST-LIO /map_save 服务生成。
俯视图可见主密集区域外有稀疏点簇；用于正式建图前建议在 CloudCompare/RViz 中复核。

轨迹导出：
  {body_tum.name}：camera_init -> body（body 为雷达 IMU 坐标系）
  {base_tum.name}：名义外参换算后的 base_link 轨迹，仅用于参考
图表：
  {trajectory_plot.name}：XY 轨迹和高度变化
  {map_plot.name}：点云地图俯视图，颜色表示高度

判读限制：
1. FAST-LIO2 不含回环优化，闭环残差会随累计漂移保留。
2. `base_link` 通过尚未标定的名义安装外参从 IMU 位姿换算。
3. B2 仅有向左 7 mm 的卷尺测量；前后位移和航向真值缺失。
4. 这是离线单次包回放结果，不代表实时负载或五次实车闭环验收。
""",
        encoding="utf-8",
    )
    print(f"Wrote {report_path}")
    print(f"Wrote {body_tum}")
    print(f"Wrote {base_tum}")
    print(f"Wrote {trajectory_plot}")
    print(f"Wrote {map_plot}")


if __name__ == "__main__":
    main()
