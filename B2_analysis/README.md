# B2 FAST-LIO 单圈测量文件索引

本目录记录已校验的 B2 单圈 rosbag 经 FAST-LIO2 离线回放后的里程计、点云地图和闭环初步对照。
输出 `/Odometry` 的 header 时间存在倒退，因此这里的轨迹与位移统计只用于离线诊断，不作为实时连续性或最终验收结论。

## 输入与配置

- [B2 原始包](../交付/B2_teleop_loop_tilt0_2026-09-23/)：遥控慢速绕实验室一圈并回到地面标记的输入 rosbag；原包 SHA-256 校验已通过。
- [b2_mid360.yaml](./b2_mid360.yaml)：本次测量使用的 FAST-LIO 配置。`use_sim_time=true`，在线外参估计关闭，使用名义 IMU-LiDAR 外参，并启用 PCD 地图保存。

## 本次测量输出

- [B2_fastlio_report.txt](./B2_fastlio_report.txt)：B2 输入/输出时间戳检查、起终点静止窗口位姿差、卷尺已知左向分量对照、地图点数和结果限制。
- [B2_fastlio_odom_saved/](./B2_fastlio_odom_saved/)：本次明确配置下的 FAST-LIO `/Odometry` rosbag。
  - [metadata.yaml](./B2_fastlio_odom_saved/metadata.yaml)：rosbag2 元数据及用途说明。
  - [B2_fastlio_odom_saved_0.db3](./B2_fastlio_odom_saved/B2_fastlio_odom_saved_0.db3)：二进制 SQLite rosbag，保存本次 1,710 条 `/Odometry`；数据库文件本身不能安全插入说明。
- [B2_fastlio_trajectory_body.tum](./B2_fastlio_trajectory_body.tum)：TUM trajectory 格式的 `camera_init -> body` 轨迹；`body` 是雷达 IMU 坐标系。按 header 时间排序输出。
- [B2_fastlio_trajectory_base_nominal.tum](./B2_fastlio_trajectory_base_nominal.tum)：用尚未标定的名义 IMU-to-`base_link` 外参换算的参考轨迹；不能替代标定轨迹。
- [B2_fastlio_map.pcd](./B2_fastlio_map.pcd)：通过本次 FAST-LIO 运行中的 `/map_save` 服务保存的 `camera_init` 全局点云地图，137,052 点。
- [B2_fastlio_trajectory.png](./B2_fastlio_trajectory.png)：名义 `base_link` XY 轨迹和高度变化图。
- [B2_fastlio_map.png](./B2_fastlio_map.png)：点云地图俯视图，颜色表示高度。
- [analyze_b2_fastlio.py](./analyze_b2_fastlio.py)：从 B2 输入包、FAST-LIO 输出 rosbag 和 PCD 重算报告、TUM 轨迹与图表。

## 读结果时的注意事项

- B2 原始 LiDAR 与 IMU header 时间严格递增；本次 FAST-LIO 输出的 `/Odometry` 在 rosbag 接收顺序中有 85 次 header 时间倒退、没有重复时间戳。rosbag 接收时间本身递增。
- 报告和 TUM 文件按 header 时间排序后分析/导出；因此这份结果不验证原始输出的实时顺序连续性。
- 起点和终点窗口分别取 B2 开始后 10–20 秒、结束前对应的 160–170 秒区间；这些区间是依据 IMU 低运动时段选择，不是独立真值。
- 包库卷尺只测得闭环向左 7 mm，没有前后位移和航向真值。报告只比较左向单轴分量，不据此宣称完整闭环误差达标或不达标。
- FAST-LIO2 不含回环优化；结果是单次离线回放。要做 R3 验收还需按工作说明重复实车闭环五次，并量测位置与航向。
