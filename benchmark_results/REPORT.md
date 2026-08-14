# Closed-loop 与 MPPI rosbag 对比报告

## 测试条件

- 数据集：`/home/wei/bag/up_and_down`
- 回放区间：`--start-offset 105 --playback-duration 110`
- 输入话题：`/plan`、`/lio_odom_hf`、`/livox/lidar`
- 点云类型：原始 Livox 点云（`point_cloud_type:=1`）
- 路径分段、起点 Z 投影、三维参考速度投影、前向占据衰减均开启
- `enable_motion:=true`，RViz 关闭，避免渲染负载进入结果
- CPU：16 个逻辑核；pidstat 的 100% 表示占满一个逻辑核
- 每套资源数据包含约 113 个 1 Hz 样本

注意：bag 中的里程计是预录数据，控制指令不会改变回放机器人运动。因此本报告比较控制指令、安全状态和计算开销，不能替代真机或动力学仿真的闭环到达误差测试。

## 控制输出

| 指标 | Closed-loop | MPPI |
|---|---:|---:|
| 有效窗口 | 109.60 s | 109.56 s |
| 原始控制频率 | 100.00 Hz | 30.00 Hz |
| 平面速度均值 | 0.3930 m/s | 0.3589 m/s |
| 平面速度 RMS | 0.4198 m/s | 0.4047 m/s |
| 平面速度 P95 | 0.5000 m/s | 0.5768 m/s |
| 平面速度最大值 | 0.5000 m/s | 0.6077 m/s |
| 绝对角速度均值 | 0.4390 rad/s | 0.3732 rad/s |
| 绝对角速度 P95 | 0.8000 rad/s | 0.7874 rad/s |
| 线加速度 P95 | 0.5085 m/s² | 1.0287 m/s² |
| 角加速度 P95 | 1.0138 rad/s² | 1.4963 rad/s² |
| 完全停止比例 | 1.37% | 6.48% |
| 速度限幅命中比例 | 4.63% | 4.65% |
| 指令总变差/秒 | 0.6760 | 0.8658 |
| 安全门修改比例 | 0.00% | 0.44% |

MPPI 平均速度低 8.7%，停止时间约为 closed-loop 的 4.7 倍。在本次参数下，MPPI 的 P95 线加速度约高 102%，单位时间指令变差高 28%，并没有表现得比简单闭环更平滑。MPPI 的平面合速度可以超过 0.5 m/s，因为 `vx`、`vy` 是分别限幅，斜向合速度可以达到更高值。

## 控制器自身资源

| 指标 | Closed-loop controller | MPPI controller |
|---|---:|---:|
| CPU 均值 | 2.98% | 39.91% |
| CPU P95 | 4.00% | 53.40% |
| CPU 最大值 | 5.00% | 57.00% |
| RSS 均值 | 31.70 MiB | 117.22 MiB |
| RSS 最大值 | 31.91 MiB | 153.65 MiB |

MPPI 控制器自身平均 CPU 是 closed-loop 的约 13.4 倍，平均 RSS 是约 3.70 倍。MPPI RSS 在启动后由约 32 MiB 增长到 120–140 MiB，主要来自候选控制序列、状态轨迹和碰撞查询缓存。

## 整个导航栈资源

统计包含规划器、当前控制器、安全门、真机输入适配器和地形路径分段器，不包含遥测记录器和 rosbag player。

| 指标 | Closed-loop 栈 | MPPI 栈 |
|---|---:|---:|
| CPU 均值 | 63.80% | 138.35% |
| CPU P95 | 88.00% | 163.00% |
| CPU 最大值 | 102.00% | 168.00% |
| RSS 均值 | 458.22 MiB | 552.01 MiB |
| RSS 最大值 | 461.68 MiB | 590.84 MiB |

MPPI 使整栈平均 CPU 增加约 116.8%，平均 RSS 增加约 20.5%。超过 100% CPU 表示栈同时使用了超过一个逻辑核，并非统计错误。

## 公共节点资源与规划稳定性

| 节点/指标 | Closed-loop 轮次 | MPPI 轮次 |
|---|---:|---:|
| `scan_planner_node` CPU 均值 | 40.66% | 78.85% |
| `scan_planner_node` RSS 均值 | 284.96 MiB | 291.57 MiB |
| 局部重规划失败日志 | 17 | 44 |
| 轨迹拒绝日志 | 7 | 20 |
| A* 失败摘要 | 12 | 44 |

两轮使用相同规划参数，但 MPPI 轮次中规划器 CPU 和失败数明显增大。这说明当前机器上 MPPI 与 SCAN 规划器存在明显的 CPU 竞争/调度耦合；它也意味着“规划器完全相同，所以只需比较控制器进程”并不成立。该结果建议进一步限制 MPPI 工作线程或候选数，然后复测。

MPPI 状态转换日志包括：

- `tracking`：35 次
- `no_safe_trajectory`：32 次
- `odometry_timeout`：3 次
- `waiting_for_trajectory`：2 次
- `map_timeout`：1 次
- `trajectory_timeout`：1 次

`no_safe_trajectory` 频繁出现，是 MPPI 停止比例较高的直接原因之一。

## 结论

在当前参数和这段 bag 上，closed-loop 控制器更轻量、输出更平滑、连续运动比例更高。MPPI 提供显式预测碰撞和候选轨迹安全选择，但当前代价是明显更高的 CPU/RSS、更多停止和更大的指令变化；其安全优势无法仅通过这个非闭环 bag 量化。

建议下一轮 MPPI 调参优先尝试：降低 `batch_size`、减少 `worker_threads`、适当增加 `noise_smoothing`、降低横向噪声，并检查 `no_safe_trajectory` 时 footprint 与 `/grid_map/occupancy_inflate` 的关系。最终优劣应在 Gazebo动力学闭环或真机低速测试中，以横向误差、到达时间、最小净空和急停次数验证。
