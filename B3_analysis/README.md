# B3 FAST-LIO 五圈测量文件索引

本目录用于分析已校验的 B3 五圈闭环 rosbag。FAST-LIO2 不执行回环优化，因此结果用于查看连续里程计中的逐圈漂移与累计回到起点偏差，不是最终 R3 实车验收。

## 输入与配置

- [B3 原始包](../交付/B3_teleop_loop_x5_2026-09-23/)：沿 B2 路线遥控行驶五圈的原始 LiDAR/IMU rosbag；SHA-256 校验通过。
- [b3_mid360.yaml](./b3_mid360.yaml)：B3 专用 FAST-LIO 配置。复用 B2 的固定名义 IMU-LiDAR 外参与算法参数，使用 `/clock` 和 `use_sim_time=true`，启用 PCD 地图保存。

## 测量输出

- [B3_fastlio_report.txt](./B3_fastlio_report.txt)：输入/输出时间戳检查、五圈逐圈位移、首尾累计偏差、与卷尺已知横向分量的对照，以及结果限制。
- [B3_fastlio_loop_drift.csv](./B3_fastlio_loop_drift.csv)：逐圈位移和累计位移表。静止窗口采用初始 20–30 秒、以及各圈后停顿段中的 118–128、221–231、307–317、386–396、480–490 秒；表中每圈附质量提示。README 标注的低运动区间边缘有明显位姿变化，圈段边界为近似值，首、二、三、五圈及累计值只能作为初步窗口差，不能用于验收。
- [B3_fastlio_odom/](./B3_fastlio_odom/)：本次 FAST-LIO `/Odometry` rosbag 输出目录，内含 rosbag2 `metadata.yaml` 和二进制 SQLite 数据库。
- [B3_fastlio_trajectory_body.tum](./B3_fastlio_trajectory_body.tum)：按 header 时间排序的 `camera_init -> body` TUM 轨迹；`body` 是雷达 IMU 坐标系。
- [B3_fastlio_trajectory_base_nominal.tum](./B3_fastlio_trajectory_base_nominal.tum)：按名义外参换算的 `base_link` 参考轨迹，外参尚未标定。
- [B3_fastlio_map.pcd](./B3_fastlio_map.pcd)：FAST-LIO `/map_save` 服务保存的累计 `camera_init` 点云地图。
- [B3_fastlio_trajectory.png](./B3_fastlio_trajectory.png)：五圈 XY 轨迹与高度变化图。
- [B3_fastlio_map.png](./B3_fastlio_map.png)：点云地图俯视图，颜色表示高度。
- [analyze_b3_fastlio.py](./analyze_b3_fastlio.py)：重算报告、逐圈 CSV、TUM 轨迹及图片的脚本。

## 解释结果时的注意事项

- 数据包 README 给出的五圈时间段和长停顿是近似切分；各圈指标由相邻静止窗口的均值位姿差估算。
- 卷尺真值只有五圈累计“向右 2 cm”，未测前后位移和航向角。只能比较累计左向分量，不应把它当完整 3D 闭环真值。
- `base_link` 结果由 FAST-LIO 的 `body` 位姿和未标定名义外参换算；环境动态、地面振动和停顿窗口选择也会影响结果。
- 若 `/Odometry` header 时间在消息接收顺序中倒退，导出的离线轨迹按 header 时间排序，不代表实时输出连续。
- 报告会列出 README 标注低运动区间内的最大相邻位姿变化；这些变化可能来自实际移动时刻与近似分段不一致，也可能来自估计异常，不能仅凭轨迹归因。
- FAST-LIO 不做闭环优化；完成离线五圈漂移分析后，仍需按工作说明进行重复实车闭环与实测位置/航向验收。
