#!/usr/bin/env python3
"""分析 B1 原始 IMU rosbag，并重现统计报告、逐分钟 CSV 和 Allan 图表。"""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_imu(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    imu_type = get_message("sensor_msgs/msg/Imu")
    accelerations = []
    angular_velocities = []
    timestamps = []

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != "/livox/imu":
            continue
        message = deserialize_message(serialized, imu_type)
        accelerations.append(
            (
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            )
        )
        angular_velocities.append(
            (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            )
        )
        timestamps.append(
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        )

    return (
        np.asarray(accelerations),
        np.asarray(angular_velocities),
        np.asarray(timestamps),
    )


def vector_summary(values):
    return {
        "mean": values.mean(axis=0),
        "std": values.std(axis=0, ddof=1),
        "variance": values.var(axis=0, ddof=1),
        "min": values.min(axis=0),
        "max": values.max(axis=0),
        "p01": np.percentile(values, 1, axis=0),
        "p99": np.percentile(values, 99, axis=0),
    }


def fmt_vector(values, digits=8):
    return "[" + ", ".join(f"{value:.{digits}g}" for value in values) + "]"


def overlapping_allan_deviation(values, sample_period):
    taus = []
    deviations = []
    cluster_sizes = []
    cumulative = np.vstack(
        (np.zeros((1, values.shape[1])), np.cumsum(values, axis=0))
    )
    max_cluster = len(values) // 4
    cluster_size = 1
    while cluster_size <= max_cluster:
        moving_means = (
            cumulative[cluster_size:] - cumulative[:-cluster_size]
        ) / cluster_size
        differences = moving_means[cluster_size:] - moving_means[:-cluster_size]
        allan_variance = 0.5 * np.mean(differences**2, axis=0)
        taus.append(cluster_size * sample_period)
        deviations.append(np.sqrt(allan_variance))
        cluster_sizes.append(cluster_size)
        cluster_size *= 2

    return np.asarray(taus), np.asarray(deviations), np.asarray(cluster_sizes)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "bag",
        nargs="?",
        type=Path,
        default=Path(
            "/home/xiaowu/active_perception/交付/"
            "B1_static_10min_2026-09-23"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent,
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    acc, gyr, stamps = read_imu(args.bag)
    if len(stamps) == 0:
        raise RuntimeError(f"No /livox/imu messages found in {args.bag}")

    relative_time = stamps - stamps[0]
    acc_stats = vector_summary(acc)
    gyr_stats = vector_summary(gyr)
    dt = np.diff(stamps)
    sample_period = np.mean(dt)
    scale = 9.80665 / np.linalg.norm(acc_stats["mean"])
    acc_cov_si = acc_stats["variance"] * scale**2
    gyr_cov = gyr_stats["variance"]
    acc_allan_tau, acc_allan_dev, allan_clusters = overlapping_allan_deviation(
        acc, sample_period
    )
    gyr_allan_tau, gyr_allan_dev, _ = overlapping_allan_deviation(
        gyr, sample_period
    )
    allan_extrema = []
    for sensor, axis_labels, taus, deviations in (
        ("加速度计", ("X", "Y", "Z"), acc_allan_tau, acc_allan_dev),
        ("陀螺仪", ("X", "Y", "Z"), gyr_allan_tau, gyr_allan_dev),
    ):
        for axis_index, axis_label in enumerate(axis_labels):
            index = int(np.argmin(deviations[:, axis_index]))
            allan_extrema.append(
                f"{sensor} {axis_label} 轴：tau={taus[index]:.5g} 秒，"
                f"Allan 偏差={deviations[index, axis_index]:.8g}"
            )
    first_minute = relative_time < 60
    last_minute = relative_time >= relative_time[-1] - 60
    acc_norm = np.linalg.norm(acc, axis=1)
    gyro_norm = np.linalg.norm(gyr, axis=1)
    tilt_degrees = np.degrees(
        np.arctan2(
            np.linalg.norm(acc_stats["mean"][:2]),
            abs(acc_stats["mean"][2]),
        )
    )
    acc_slope = np.array(
        [np.polyfit(relative_time, acc[:, axis], 1)[0] for axis in range(3)]
    )
    gyr_slope = np.array(
        [np.polyfit(relative_time, gyr[:, axis], 1)[0] for axis in range(3)]
    )

    csv_path = args.output_dir / "B1_imu_1min.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "分段序号",
                "起始时间_s",
                "结束时间_s",
                "样本数",
                "加速度均值_x_g",
                "加速度均值_y_g",
                "加速度均值_z_g",
                "加速度标准差_x_g",
                "加速度标准差_y_g",
                "加速度标准差_z_g",
                "角速度均值_x_rad_s",
                "角速度均值_y_rad_s",
                "角速度均值_z_rad_s",
                "角速度标准差_x_rad_s",
                "角速度标准差_y_rad_s",
                "角速度标准差_z_rad_s",
            ]
        )
        for start in np.arange(0, relative_time[-1], 60):
            stop = min(start + 60, relative_time[-1])
            mask = (relative_time >= start) & (relative_time < stop)
            if not np.any(mask):
                continue
            writer.writerow(
                [
                    int(start // 60) + 1,
                    f"{start:.3f}",
                    f"{stop:.3f}",
                    int(mask.sum()),
                    *[f"{v:.9g}" for v in acc[mask].mean(axis=0)],
                    *[f"{v:.9g}" for v in acc[mask].std(axis=0, ddof=1)],
                    *[f"{v:.9g}" for v in gyr[mask].mean(axis=0)],
                    *[f"{v:.9g}" for v in gyr[mask].std(axis=0, ddof=1)],
                ]
            )

    allan_csv_path = args.output_dir / "B1_imu_allan.csv"
    with allan_csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "聚类样本数",
                "平均采样周期_s",
                "聚类时间_tau_s",
                "加速度Allan偏差_x_g",
                "加速度Allan偏差_y_g",
                "加速度Allan偏差_z_g",
                "角速度Allan偏差_x_rad_s",
                "角速度Allan偏差_y_rad_s",
                "角速度Allan偏差_z_rad_s",
            ]
        )
        for index, cluster_size in enumerate(allan_clusters):
            writer.writerow(
                [
                    int(cluster_size),
                    f"{sample_period:.12g}",
                    f"{acc_allan_tau[index]:.12g}",
                    *[f"{v:.12g}" for v in acc_allan_dev[index]],
                    *[f"{v:.12g}" for v in gyr_allan_dev[index]],
                ]
            )

    allan_plot_path = args.output_dir / "B1_imu_allan.png"
    figure, axes = plt.subplots(1, 2, figsize=(13, 5))
    for axis, data, title, unit in (
        (axes[0], acc_allan_dev, "Accelerometer", "g"),
        (axes[1], gyr_allan_dev, "Gyroscope", "rad/s"),
    ):
        for component, label in enumerate(("X", "Y", "Z")):
            axis.loglog(
                acc_allan_tau,
                data[:, component],
                marker="o",
                markersize=3,
                label=label,
            )
        axis.set_title(title)
        axis.set_xlabel("Averaging time tau (s)")
        axis.set_ylabel(f"Overlapping Allan deviation ({unit})")
        axis.grid(True, which="both", linestyle=":")
        axis.legend(title="Axis")
    figure.suptitle("B1 Static IMU Overlapping Allan Deviation (Noise Characterization)")
    figure.tight_layout()
    figure.savefig(allan_plot_path, dpi=160)
    plt.close(figure)

    report_path = args.output_dir / "B1_imu_stats.txt"
    with report_path.open("w", encoding="utf-8") as report:
        report.write(
            f"""B1 IMU 静止数据统计
====================
用途：汇总 B1 原始 IMU 的单位、均值、样本方差、时间变化和 Allan 偏差；
用于理解数据与提供 FAST-LIO 调参参考，不代表最终 IMU 噪声参数。

数据来源：{args.bag}
统计话题：/livox/imu (sensor_msgs/msg/Imu)
样本数：{len(stamps)}
消息时间戳范围：{stamps[0]:.9f} 至 {stamps[-1]:.9f} 秒
消息时间戳跨度：{relative_time[-1]:.9f} 秒
加速度单位：g（原始消息单位）
角速度单位：rad/s

一、全程均值、标准差和样本方差（按 x、y、z）
加速度均值（g）：{fmt_vector(acc_stats['mean'])}
加速度标准差（g）：{fmt_vector(acc_stats['std'])}
加速度样本方差（g^2）：{fmt_vector(acc_stats['variance'])}
角速度均值（rad/s）：{fmt_vector(gyr_stats['mean'])}
角速度标准差（rad/s）：{fmt_vector(gyr_stats['std'])}
角速度样本方差（(rad/s)^2）：{fmt_vector(gyr_cov)}

二、重力与静止零偏参考
加速度均值模长：{np.linalg.norm(acc_stats['mean']):.9f} g
相对 +Z 轴的倾角估计：{tilt_degrees:.5f} 度
陀螺仪静止均值（可作零偏初值参考）：{fmt_vector(gyr_stats['mean'])} rad/s
加速度均值是静止时的比力测量，包含重力投影，不能直接当成纯加速度计零偏。

三、角速度均值随时间的变化
前 60 秒角速度均值（rad/s）：{fmt_vector(gyr[first_minute].mean(axis=0))}
后 60 秒角速度均值（rad/s）：{fmt_vector(gyr[last_minute].mean(axis=0))}
全程线性拟合斜率（每分钟，rad/s）：{fmt_vector(gyr_slope * 60)}
这表明静止均值随时间有变化；全程均值只适合作为统计参考，不应不加验证地当作恒定零偏。
分段统计见 B1_imu_1min.csv。

四、为 FAST-LIO 参数提供的经验参考
按 FAST-LIO 的加速度归一化比例 9.80665 / |均值加速度| 换算后，
各轴加速度样本方差参考值（(m/s^2)^2）：{fmt_vector(acc_cov_si)}
角速度各轴样本方差参考值（(rad/s)^2）：{fmt_vector(gyr_cov)}
各轴方差的平均值参考：acc_cov={acc_cov_si.mean():.8g},
gyr_cov={gyr_cov.mean():.8g}。
这些是本包全程离散样本方差，不是经 Allan 方差得到的噪声密度。
IMU 含振动、短期噪声及慢变零偏；应作为参数试验参考，不应直接替代
FAST-LIO 配置默认值。需要在 B2/B3 上对照轨迹稳定性后再确定最终配置。

五、时间戳与数据范围
时间戳间隔均值：{dt.mean():.9f} 秒；中位数：{np.median(dt):.9f} 秒
时间戳间隔标准差：{dt.std(ddof=1):.9f} 秒
最小/最大间隔：{dt.min():.9f} / {dt.max():.9f} 秒
非递增间隔数：{int(np.sum(dt <= 0))}
大于 10 ms 的间隔数：{int(np.sum(dt > 0.010))}
加速度模长均值/标准差/范围（g）：{acc_norm.mean():.8f} / {acc_norm.std(ddof=1):.8f} / {acc_norm.min():.8f}–{acc_norm.max():.8f}
角速度模长均值/标准差/最大值（rad/s）：{gyro_norm.mean():.8f} / {gyro_norm.std(ddof=1):.8f} / {gyro_norm.max():.8f}

六、重叠 Allan 偏差
平均采样周期为 {sample_period:.9f} 秒；按 1、2、4……个样本的二次幂
聚类时间计算，最大聚类大小不超过总样本数的四分之一。曲线用于观察
不同平均时间尺度的噪声变化，本报告不强行套用特定传感器噪声模型。
本采样范围内各轴观测到的最小 Allan 偏差（仅描述曲线，不直接等同于
偏置不稳定性参数）：
{chr(10).join(allan_extrema)}
数据表：B1_imu_allan.csv；曲线图：B1_imu_allan.png。

注意：以上统计以整段“机器人静止”数据为基础。若地面振动、人员走动或
温漂影响估计，样本方差会高于传感器纯白噪声。位姿静止抖动仍需运行
FAST-LIO 并从轨迹计算，本项不是 IMU 统计的直接结果。
"""
        )

    print(f"Wrote {report_path}")
    print(f"Wrote {csv_path}")


if __name__ == "__main__":
    main()
