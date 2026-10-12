# B1 分析文件索引

本目录保存 B1 静止包的原始数据检查、IMU 统计、FAST-LIO 静止位姿测量及其复现脚本。
以下逐项说明文件用途、数据来源和阅读时的注意事项。

## 原始输入包

- [B1_static_10min_2026-09-23_0.db3](../交付/B1_static_10min_2026-09-23/B1_static_10min_2026-09-23_0.db3)：平台录制的 B1 原始 SQLite rosbag，包含 `/livox/lidar`、`/livox/imu` 和 `/tf_static`，是本目录统计与 FAST-LIO 回放的输入。数据库是二进制文件，不能安全地在开头插入说明。
- [原始包 metadata.yaml](../交付/B1_static_10min_2026-09-23/metadata.yaml)：原始 B1 rosbag 的 rosbag2 元数据，记录时长、消息数和话题类型。

## 数据说明与坐标系

- [B1_playback_record.txt](./B1_playback_record.txt)：B1 原始包的话题、消息频率、回放检查结果、静态 TF 各变换的含义，以及本目录的测量产物索引。
- [B1_tf_tree.gv](./B1_tf_tree.gv)：TF 图的 Graphviz 源文件，可编辑并重新生成图；图仅展示原包中的 `/tf_static`。
- [B1_tf_tree.pdf](./B1_tf_tree.pdf)：上述 TF 图的 PDF 阅读版，含各变换的说明。

## IMU 统计

- [B1_imu_stats.txt](./B1_imu_stats.txt)：B1 `/livox/imu` 全程统计报告，包括单位、均值、标准差、方差、时间变化、时间戳间隔和 FAST-LIO 参数参考。方差是样本统计量，不等同于噪声密度或最终配置参数。
- [B1_imu_1min.csv](./B1_imu_1min.csv)：按一分钟分段的 IMU 加速度和角速度均值、标准差。第一行为列名；保留标准 CSV 结构，文件开头不插入注释行。
- [B1_imu_allan.csv](./B1_imu_allan.csv)：不同聚类时间下的重叠 Allan 偏差数值，供绘图或数值分析；它不是直接可用的 FAST-LIO 参数表。
- [B1_imu_allan.png](./B1_imu_allan.png)：IMU 加速度计和陀螺仪的重叠 Allan 偏差曲线图。
- [analyze_b1_imu.py](./analyze_b1_imu.py)：从 B1 rosbag 重算 IMU 统计报告、逐分钟 CSV、Allan CSV 和曲线图的脚本。

## FAST-LIO 静止位姿测量

- [b1_mid360.yaml](./b1_mid360.yaml)：本轮 B1 FAST-LIO 离线测量的专用配置副本；外参为名义值，不能视作标定后的最终配置。
- [B1_fastlio_static_jitter.txt](./B1_fastlio_static_jitter.txt)：FAST-LIO `/Odometry` 的离线静态位姿波动报告，含全程和剔除首分钟后的统计。输出存在 header 时间倒退，因此不用于实时性验收；换算到 `base_link` 的外参也尚未标定。
- [B1_fastlio_1min.csv](./B1_fastlio_1min.csv)：按 header 时间排序后，以一分钟为区间统计的名义 `base_link` 位置与姿态均值、标准差。第一行为列名；不添加会影响 CSV 读取的注释行。
- [B1_fastlio_pose.png](./B1_fastlio_pose.png)：按 `/Odometry` header 时间排序绘制的名义 `base_link` 位置和姿态变化图；位置使用中位数为参考，外参为名义值。
- [analyze_b1_fastlio_odometry.py](./analyze_b1_fastlio_odometry.py)：读取 FAST-LIO 输出 rosbag，计算离线静止位姿波动，并生成报告、逐分钟 CSV 和曲线图。

## 时间戳检查

- [B1_timestamp_order.txt](./B1_timestamp_order.txt)：对比原始 B1 LiDAR/IMU 与半速 FAST-LIO `/Odometry` 的 header 时间及 rosbag 记录时间顺序；显示异常出现在哪份记录中，但不推断具体根因。
- [check_b1_timestamp_order.py](./check_b1_timestamp_order.py)：重算上述时间戳顺序报告的脚本。

## FAST-LIO 输出 rosbag

- [B1_fastlio_odometry/](./B1_fastlio_odometry/)：首轮正常回放速率的 FAST-LIO 输出记录目录，用于诊断，不作为最终静止位姿统计数据。
  - [metadata.yaml](./B1_fastlio_odometry/metadata.yaml)：该 rosbag 的 rosbag2 元数据和用途说明。
  - [B1_fastlio_odometry_0.db3](./B1_fastlio_odometry/B1_fastlio_odometry_0.db3)：二进制 SQLite rosbag，保存该轮 `/Odometry` 消息。数据库格式不适合在文件开头插入说明，文件信息见相邻的 `metadata.yaml`。
- [B1_fastlio_odometry_slow/](./B1_fastlio_odometry_slow/)：半速完整回放的 FAST-LIO 输出记录目录；用于当前离线静态位姿分布测量。
  - [metadata.yaml](./B1_fastlio_odometry_slow/metadata.yaml)：该 rosbag 的 rosbag2 元数据和用途说明。
  - [B1_fastlio_odometry_slow_0.db3](./B1_fastlio_odometry_slow/B1_fastlio_odometry_slow_0.db3)：二进制 SQLite rosbag，保存 6,143 条 `/Odometry` 消息。数据库格式不适合在文件开头插入说明，文件信息见相邻的 `metadata.yaml`。

## 文件格式说明

报告和脚本在文件开头说明用途；YAML 配置及 rosbag 元数据用注释说明。CSV 保持标准列标题作为第一行，避免破坏常规解析器。PNG/PDF 属于图像或排版文档，说明放在图标题或本索引中；SQLite `.db3` 是二进制 rosbag，说明放在相邻元数据和本索引中。
