#!/usr/bin/env python3
"""Analyze B3 FAST-LIO output and summarize drift at the five loop stops."""

import argparse
import csv
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
LOOP_WINDOWS = (
    (20.0, 30.0),
    (118.0, 128.0),
    (221.0, 231.0),
    (307.0, 317.0),
    (386.0, 396.0),
    (480.0, 490.0),
)
STATIC_INTERVALS = (
    (0.0, 39.0),
    (114.0, 136.0),
    (217.0, 239.0),
    (304.0, 323.0),
    (383.0, 401.0),
    (461.0, 546.0),
)
LOOP_QUALITY_NOTES = (
    "32-39s 起点静止标注区出现位姿偏移；首圈边界需复核",
    "131-136s 下一圈起点前出现位姿偏移；圈界需复核",
    "233-239s 与 304-306s 出现位姿偏移；圈界需复核",
    "选定窗口内稳定；圈段仍按包库近似时间切分",
    "396-401s 下一圈起点前出现位姿偏移；圈界需复核",
)
B3_LATERAL_GROUND_TRUTH_M = -0.020


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
        / "B3_teleop_loop_x5_2026-09-23",
    )
    parser.add_argument(
        "--odometry-bag", type=Path, default=default_dir / "B3_fastlio_odom"
    )
    parser.add_argument("--map", type=Path, default=default_dir / "B3_fastlio_map.pcd")
    parser.add_argument("--output-dir", type=Path, default=default_dir)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    source_lidar_stamps = read_topic_stamps(args.source_bag, "/livox/lidar")
    source_imu_stamps = read_topic_stamps(args.source_bag, "/livox/imu")
    source_start = source_lidar_stamps[0]
    stamps, recording_stamps, positions, quaternions, frames, child_frames = (
        read_odometry(args.odometry_bag)
    )
    received_dt = np.diff(stamps)
    backwards = int(np.sum(received_dt < 0))
    repeated = int(np.sum(received_dt == 0))
    record_nonincreasing = int(np.sum(np.diff(recording_stamps) <= 0))
    chronological = np.argsort(stamps, kind="stable")
    stamps = stamps[chronological]
    positions = positions[chronological]
    quaternions = quaternions[chronological]
    elapsed = stamps - source_start
    sample_dt = np.diff(stamps)
    rotations = Rotation.from_quat(quaternions)
    base_positions = positions + rotations.apply(IMU_TO_BASE_IN_IMU)
    step_distance = np.linalg.norm(np.diff(base_positions, axis=0), axis=1)
    step_time = (elapsed[:-1] + elapsed[1:]) / 2
    static_interval_jumps = []
    for interval in STATIC_INTERVALS:
        mask = (step_time >= interval[0]) & (step_time < interval[1])
        if not mask.any():
            continue
        interval_step_distances = step_distance[mask]
        interval_step_times = step_time[mask]
        largest = int(np.argmax(interval_step_distances))
        static_interval_jumps.append(
            (
                interval,
                float(interval_step_times[largest]),
                float(interval_step_distances[largest]),
                int(np.sum(interval_step_distances > 0.010)),
            )
        )

    def window_stats(window):
        mask = (elapsed >= window[0]) & (elapsed < window[1])
        if mask.sum() < 2:
            raise RuntimeError(
                f"Not enough odometry in B3 interval {window[0]}-{window[1]}s"
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

    windows = [window_stats(window) for window in LOOP_WINDOWS]
    lap_results = []
    for lap_index, (start, end) in enumerate(zip(windows, windows[1:]), start=1):
        displacement = start["mean_rotation"].inv().apply(
            end["mean_position"] - start["mean_position"]
        )
        angle = (start["mean_rotation"].inv() * end["mean_rotation"]).as_euler(
            "xyz", degrees=True
        )
        lap_results.append((displacement, angle))
    total_displacement = windows[0]["mean_rotation"].inv().apply(
        windows[-1]["mean_position"] - windows[0]["mean_position"]
    )
    total_angle = (
        windows[0]["mean_rotation"].inv() * windows[-1]["mean_rotation"]
    ).as_euler("xyz", degrees=True)
    total_xy = float(np.linalg.norm(total_displacement[:2]))
    lateral_error = total_displacement[1] - B3_LATERAL_GROUND_TRUTH_M

    body_tum = args.output_dir / "B3_fastlio_trajectory_body.tum"
    base_tum = args.output_dir / "B3_fastlio_trajectory_base_nominal.tum"
    write_tum(body_tum, stamps, positions, quaternions)
    write_tum(base_tum, stamps, base_positions, quaternions)

    csv_path = args.output_dir / "B3_fastlio_loop_drift.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "圈序",
                "起点窗口_s",
                "终点窗口_s",
                "起点样本数",
                "终点样本数",
                "前向位移_m",
                "左向位移_m",
                "上向位移_m",
                "水平位移模长_m",
                "roll差_deg",
                "pitch差_deg",
                "yaw差_deg",
                "累计前向位移_m",
                "累计左向位移_m",
                "累计上向位移_m",
                "质量提示",
            )
        )
        for index, (displacement, angle) in enumerate(lap_results, start=1):
            cumulative = windows[0]["mean_rotation"].inv().apply(
                windows[index]["mean_position"] - windows[0]["mean_position"]
            )
            writer.writerow(
                (
                    index,
                    f"{LOOP_WINDOWS[index - 1][0]:.0f}-{LOOP_WINDOWS[index - 1][1]:.0f}",
                    f"{LOOP_WINDOWS[index][0]:.0f}-{LOOP_WINDOWS[index][1]:.0f}",
                    windows[index - 1]["count"],
                    windows[index]["count"],
                    *[f"{value:.9g}" for value in displacement],
                    f"{np.linalg.norm(displacement[:2]):.9g}",
                    *[f"{value:.9g}" for value in angle],
                    *[f"{value:.9g}" for value in cumulative],
                    LOOP_QUALITY_NOTES[index - 1],
                )
            )

    trajectory_plot = args.output_dir / "B3_fastlio_trajectory.png"
    figure, axes = plt.subplots(1, 2, figsize=(13, 6))
    scatter = axes[0].scatter(
        base_positions[:, 0] - base_positions[0, 0],
        base_positions[:, 1] - base_positions[0, 1],
        c=elapsed,
        s=2,
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
    figure.colorbar(scatter, ax=axes[0], label="B3 elapsed time (s)")
    axes[1].plot(elapsed, (base_positions - base_positions[0])[:, 2], color="purple")
    axes[1].set_xlabel("B3 elapsed time (s)")
    axes[1].set_ylabel("Nominal base_link height change (m)")
    axes[1].grid(True, linestyle=":")
    figure.suptitle("B3 FAST-LIO Five-Loop Trajectory (Nominal Extrinsic)")
    figure.tight_layout()
    figure.savefig(trajectory_plot, dpi=160)
    plt.close(figure)

    map_points, map_count = read_pcd_xyz(args.map)
    map_points = map_points[np.isfinite(map_points).all(axis=1)]
    map_plot = args.output_dir / "B3_fastlio_map.png"
    figure, axis = plt.subplots(figsize=(9, 8))
    point_plot = axis.scatter(
        map_points[:, 0],
        map_points[:, 1],
        c=map_points[:, 2],
        s=0.3,
        cmap="terrain",
        rasterized=True,
    )
    axis.set_xlabel("camera_init X (m)")
    axis.set_ylabel("camera_init Y (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, linestyle=":")
    figure.colorbar(point_plot, ax=axis, label="camera_init Z (m)")
    figure.suptitle(f"B3 FAST-LIO Accumulated Map ({len(map_points):,} finite points)")
    figure.tight_layout()
    figure.savefig(map_plot, dpi=160)
    plt.close(figure)

    report_path = args.output_dir / "B3_fastlio_report.txt"
    report_lines = [
        "B3 FAST-LIO 五圈漂移与闭环初步测量",
        "==================================",
        "用途：整理 B3 五圈 FAST-LIO 离线回放的时间戳质量、逐圈静止点",
        "位姿差、累计位移和地图。FAST-LIO 不含回环优化；不作为最终验收。",
        "",
        f"输入包：{args.source_bag}",
        f"FAST-LIO 输出包：{args.odometry_bag}",
        "FAST-LIO 配置：b3_mid360.yaml；/clock 回放，use_sim_time=true；",
        "固定名义 IMU-LiDAR 外参，关闭在线外参估计；地图经 /map_save 保存。",
        "",
        f"原始 B3 LiDAR 帧数：{len(source_lidar_stamps)}",
        f"原始 B3 IMU 样本数：{len(source_imu_stamps)}",
        f"FAST-LIO /Odometry 样本数：{len(stamps)}",
        f"输出坐标系：{', '.join(sorted(frames))} -> {', '.join(sorted(child_frames))}",
        f"header 时间覆盖：{elapsed[0]:.6f} 至 {elapsed[-1]:.6f} 秒"
        "（相对 B3 首个 LiDAR header）",
        f"输出消息原始顺序 header 倒退/重复数：{backwards} / {repeated}",
        f"rosbag 接收时间非递增数：{record_nonincreasing}",
        f"按 header 排序后的采样间隔均值/最小/最大："
        f"{sample_dt.mean():.6f} / {sample_dt.min():.6f} / {sample_dt.max():.6f} 秒",
        "输出按 header 时间排序后分析和导出；若原始消息顺序有倒退，",
        "本结果只作离线轨迹诊断，不证明实时连续性。",
        f"输入 LiDAR header 倒退/重复数："
        f"{int(np.sum(np.diff(source_lidar_stamps) < 0))} / "
        f"{int(np.sum(np.diff(source_lidar_stamps) == 0))}",
        f"输入 IMU header 倒退/重复数："
        f"{int(np.sum(np.diff(source_imu_stamps) < 0))} / "
        f"{int(np.sum(np.diff(source_imu_stamps) == 0))}",
        "",
        "README 标注为静止/长停顿的区间，相邻位姿跳变诊断（名义 base_link）：",
    ]
    for interval, at_time, distance, count in static_interval_jumps:
        report_lines.append(
            f"  {interval[0]:.0f}-{interval[1]:.0f}s：最大相邻位移 "
            f"{distance * 1000:.1f} mm（约 {at_time:.1f}s）；"
            f">10 mm 相邻跳变 {count} 次"
        )
    report_lines.extend(
        (
            "这些只是轨迹输出与包库低运动时间段的对照；不能单独认定物理移动或根因。",
            "",
            "逐圈结果有效性提示：圈段边界来自近似时间；标注低运动区间内的位姿变化",
            "可能是实际提前/延后移动，也可能是估计异常，现有数据不能区分。",
            "因此首圈、第二圈、第三圈和第五圈的窗口差包含邻近边界位姿变化，",
            "只能作为初步停顿窗口差值，不能当作已验证的单圈漂移或累计闭环验收值。",
            "第四圈两端窗口本身稳定，但圈界仍是近似切分。",
            "",
            "静止窗口（相对首个 LiDAR header 秒）：",
        )
    )
    for index, (window, stats) in enumerate(zip(LOOP_WINDOWS, windows)):
        report_lines.append(
            f"  W{index}: {window[0]:.0f}-{window[1]:.0f}s，{stats['count']} 样本；"
            f"位置标准差 {vector(stats['position_std'] * 1000)} mm；"
            f"姿态标准差 {vector(stats['orientation_std_deg'])} 度"
        )
    report_lines.extend(
        (
            "",
            "逐圈位移为相邻静止窗口的均值差，转换到该圈起点 IMU/body 轴向；",
            "base_link 位置使用未标定的名义外参换算。",
        )
    )
    for index, (displacement, angle) in enumerate(lap_results, start=1):
        report_lines.append(
            f"  第 {index} 圈：前/左/上 {vector(displacement * 1000)} mm；"
            f"水平 {np.linalg.norm(displacement[:2]) * 1000:.2f} mm；"
            f"roll/pitch/yaw {vector(angle)} 度"
        )
    report_lines.extend(
        (
            "",
            f"首尾静止窗口累计位移（起点 IMU/body 轴向，base_link 名义外参）："
            f"{vector(total_displacement * 1000)} mm",
            f"首尾水平位移模长：{total_xy * 1000:.2f} mm",
            f"首尾 roll/pitch/yaw 差：{vector(total_angle)} 度",
            "B3 卷尺实测五圈累计终点相对起点向右 20 mm；按 ROS +Y 向左，",
            f"真值左向分量为 -20.00 mm，估计为 {total_displacement[1] * 1000:.2f} mm，",
            f"该分量差值为 {lateral_error * 1000:.2f} mm。",
            "真值未测前后位移和航向角；这只是单轴对照，不足以评定完整闭环误差。",
            "",
            f"PCD 地图：{args.map}",
            f"点数：{map_count}（有限 XYZ 点 {len(map_points)}），坐标系 camera_init。",
            "地图预览中如见主结构以外稀疏点簇，正式建图前应在 CloudCompare/RViz 复核。",
            "",
            f"轨迹：{body_tum.name}（camera_init -> body）；"
            f"{base_tum.name}（名义外参换算，仅供参考）",
            f"图表：{trajectory_plot.name}；{map_plot.name}",
            "",
            "判读限制：",
            "1. FAST-LIO2 不执行回环优化，五圈漂移保留在连续 odom 轨迹中。",
            "2. README 中各圈时间范围为近似值；静止区间邻近边界存在位姿跳变，"
            "各圈有效性提示见上文及 CSV。",
            "3. base_link 外参未标定；地面振动、动态物体和静止窗口均值也影响结果。",
            "4. 只有累计向右 2 cm 的卷尺分量真值，没有前后位移或航向真值。",
            "5. 本结果是单次离线回放，不代表五次实车 R3 验收或实时性能。",
        )
    )
    report_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")
    print(f"Wrote {report_path}")
    print(f"Wrote {csv_path}")
    print(f"Wrote {body_tum}")
    print(f"Wrote {base_tum}")
    print(f"Wrote {trajectory_plot}")
    print(f"Wrote {map_plot}")


if __name__ == "__main__":
    main()
