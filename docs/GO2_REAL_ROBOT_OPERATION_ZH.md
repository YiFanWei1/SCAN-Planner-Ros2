# SCAN-Planner + Go2 真机适配与操作说明

本文档对应当前仓库实现，目标是将真机雷达点云、LIO 定位和全局路径接入
SCAN-Planner，并将规划得到的时间参数化 B-spline 转换为 Go2 SDK 可执行的
机体系速度。

## 1. 系统数据链路

```text
/plan (nav_msgs/Path，完整全局路径)
              │
              ▼
terrain_path_segmenter
  - 按沿程 Z 线性误差自动分段
  - 根据实际定位选择当前段
              │
              ▼
/terrain_path/current (nav_msgs/Path)
              │
              ├──────────────┐
              ▼              ▼
SCAN-Planner            新路径触发地图刷新
  ▲   ▲                 清图后等待新点云融合
  │   │
  │   └── /LIO/odom_vehicle 或等价定位
  └────── /LIO/clouds_lidar 或等价点云
              │
              ▼
/planning/bspline (scan_planner_msgs/Bspline)
              │
              ▼
closed_loop_controller
  - B-spline 位置/速度求值
  - 位置反馈 + 速度前馈
  - 世界系速度转换到机体系
              │
              ▼
/cmd_vel (geometry_msgs/Twist)
              │
              ▼
Go2 SDK 速度桥（真机需提供）
              │
              ▼
Go2 vx / vy / wz
```

SCAN-Planner 不直接调用 Go2 SDK。它只发布 `/planning/bspline`。当前仓库中的
`closed_loop_controller` 负责生成 `/cmd_vel`，真机还需要一个 SDK 速度桥接节点。

## 2. 当前默认话题

| 用途 | 当前默认话题 | 类型 |
|---|---|---|
| 完整全局路径 | `/plan` | `nav_msgs/msg/Path` |
| 分段后的当前路径 | `/terrain_path/current` | `nav_msgs/msg/Path` |
| 全部分段可视化 | `/terrain_path/segments` | `visualization_msgs/msg/MarkerArray` |
| 当前分段目标 | `/terrain_path/current_goal` | `geometry_msgs/msg/PoseStamped` |
| 真机车体定位 | `/LIO/odom_vehicle` | `nav_msgs/msg/Odometry` |
| 真机雷达位姿 | `/LIO/odom_imu` | `nav_msgs/msg/Odometry` |
| 真机雷达点云 | `/LIO/clouds_lidar` | `sensor_msgs/msg/PointCloud2` |
| SCAN 局部轨迹 | `/planning/bspline` | `scan_planner_msgs/msg/Bspline` |
| 速度命令 | `/cmd_vel` | `geometry_msgs/msg/Twist` |

如果真机话题名不同，应在 launch 中 remap，不建议直接修改算法内部话题名。

## 3. 编译与环境

```bash
cd /home/wei/github_code/SCAN-Planner-Ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每个新终端都需要执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wei/github_code/SCAN-Planner-Ros2/install/setup.bash
```

## 4. 上机前的数据检查

### 4.1 检查话题与频率

```bash
ros2 topic list
ros2 topic hz /LIO/clouds_lidar
ros2 topic hz /LIO/odom_vehicle
ros2 topic hz /plan
```

建议：

- 点云不低于 10 Hz；
- 定位不低于 20 Hz，推荐 50 Hz 或以上；
- 点云和定位时间差不超过 50 ms；
- 全局路径至少包含两个有限的 XYZ 点。

### 4.2 检查消息坐标系

```bash
ros2 topic echo /LIO/clouds_lidar --once --field header
ros2 topic echo /LIO/odom_vehicle --once
ros2 topic echo /plan --once --field header
```

必须确认：

1. `/plan`、定位以及 SCAN 使用的世界坐标定义一致；
2. 点云是雷达坐标系还是世界坐标系；
3. 四元数有效且已归一化；
4. Z 轴朝上；
5. yaw 正方向符合 ROS 右手系约定。

### 4.3 确认路径 Z 的含义

当前分段节点保持输入路径的原始 Z。SCAN 的参考路径处理会增加
`grid_map.body_height`。

因此当前约定为：

```text
/plan.pose.position.z = 地形表面高度
SCAN内部目标高度      = 地形高度 + body_height
```

如果 `/plan` 的 Z 已经是机身中心高度，必须将 `body_height` 设为 0，不能重复增加。

## 5. 先只验证路径分段

不启动 SCAN 和 SDK：

```bash
ros2 launch terrain_path_segmenter visualize_segments.launch.py \
  input_path:=/plan \
  body_pose:=/LIO/odom_vehicle \
  fixed_frame:=map \
  use_sim_time:=false
```

RViz 中：

- 灰色线：完整全局路径；
- 彩色线：自动分段；
- 黄色粗线：当前选择的路径段；
- `G1/G2/...`：各分段目标；
- 黄色大球：当前目标。

主要参数：

```text
max_linear_z_error       默认 0.04 m
slope_merge_threshold    默认 0.04
segment_reached_tolerance 默认 0.25 m
```

真机路径存在 Z 抖动时，应先适当增大 `max_linear_z_error`，避免产生大量很短的段。

## 6. 启动 SCAN 真机模式

保持路径分段节点运行，在新终端启动：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=3 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  initial_path_topic:=/terrain_path/current \
  clear_map_on_new_path:=true \
  fresh_observations_before_planning:=2 \
  use_sim_time:=false
```

真机模式当前默认映射为：

```text
body_pose  -> /LIO/odom_vehicle
sensor_pose -> /LIO/odom_imu
cloud      -> /LIO/clouds_lidar
initial_path -> /terrain_path/current
cmd_vel    -> /cmd_vel
```

首次测试时不要让 `/cmd_vel` 接入 Go2 SDK，只观察规划和控制输出。

### 6.1 新路径地图刷新

启用后，每次收到新分段都会出现：

```text
[MapRefresh] Cleared occupancy and inflation
[MapRefresh] Deferred reference path until 2 fresh map observations
[MapRefresh] Fresh map ready
Reference path accepted
```

如果持续出现 `Still waiting for fresh observations`，说明点云或点云定位同步没有正常工作，
此时不会在空地图上接受新路径。

## 7. 雷达外参与点云设置

点云处于雷达坐标系时：

```yaml
grid_map:
  cloud_is_world: false
  need_extrinsic: true
  lidar_extrinsic_x: 0.0
  lidar_extrinsic_y: 0.0
  lidar_extrinsic_z: 0.0
  lidar_extrinsic_roll: 0.0
  lidar_extrinsic_pitch: 0.0
  lidar_extrinsic_yaw: 0.0
```

这些值必须替换为真机标定结果。外参含义是雷达相对于用于定位的机体/IMU 坐标系的
固定变换。不要直接复用 Gazebo 的 `z=0.30 m`，除非真机安装位置完全一致。

点云若已经由 LIO 转到世界坐标系，应使用：

```yaml
grid_map:
  cloud_is_world: true
  need_extrinsic: false
```

不要同时对世界系点云再次应用雷达外参。

## 8. 地面点处理

Gazebo 中的 `min_obstacle_height_below_sensor=-1.0` 是为了观察完整地面点所做的实验配置，
不应直接作为真机最终方案。关闭过滤会让地面、坡面和平台有机会进入障碍 occupancy。

推荐真机使用独立地面分割：

```text
原始点云
  ├── 地面点 → 高程/可通行地形估计
  └── 非地面点 → SCAN障碍occupancy
```

如果暂时没有地面分割，应先在静止状态检查：

```bash
ros2 topic echo /grid_map/occupancy --once
ros2 topic echo /grid_map/occupancy_inflate --once
```

确认机器人机身参考高度附近没有连续地面障碍层后才能运动。

## 9. 验证 SCAN 输出轨迹

```bash
ros2 topic hz /planning/bspline
ros2 topic echo /planning/bspline --once
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
```

控制链路为：

```text
p_ref(t), v_ref(t) = 对B-spline按当前时间求值
v_world = v_ref + Kp_pos * (p_ref - p_actual)
v_body  = R_world_to_body(yaw_actual) * v_world
wz      = Kp_yaw * normalize(yaw_ref - yaw_actual)
```

`/cmd_vel.linear.x/y` 是机体系速度，不应再作为世界系速度旋转一次。

## 10. Go2 SDK 速度桥

建议单独建立 `go2_sdk_velocity_bridge`，订阅 `/cmd_vel`，经过最后一道安全限制后调用
Go2 SDK。概念代码：

```cpp
void cmdVelCallback(const geometry_msgs::msg::Twist &msg)
{
  if (!odomFresh() || !trajectoryFresh() || emergencyStopPressed())
  {
    sdkMove(0.0, 0.0, 0.0);
    return;
  }

  const double vx = clamp(msg.linear.x,  -max_vx, max_vx);
  const double vy = clamp(msg.linear.y,  -max_vy, max_vy);
  const double wz = clamp(msg.angular.z, -max_wz, max_wz);
  sdkMove(rateLimit(vx), rateLimit(vy), rateLimit(wz));
}
```

必须根据所用 Unitree SDK 版本确认：

- `vx` 前向正方向；
- `vy` 左向或右向正方向；
- `wz` 顺/逆时针方向；
- 单位是否为 m/s 和 rad/s；
- SDK 是否要求固定频率持续发送；
- 进入运动模式和退出运动模式的正确 API。

## 11. 推荐初始安全参数

首次落地测试建议：

```yaml
max_vx: 0.20
max_vy: 0.10
max_wz: 0.30
max_linear_acceleration: 0.20
max_angular_acceleration: 0.40
trajectory_timeout: 0.50
odometry_timeout: 0.20
pointcloud_timeout: 0.50
sdk_command_timeout: 0.20
```

任意关键数据超时后，应持续向 SDK 发送零速度，而不是保留最后一条命令。

## 12. 真机测试顺序

1. 将 Go2 架空或使用保护架，确认 SDK 三个速度方向。
2. 只启动雷达和定位，在 RViz 检查点云与定位是否同步。
3. 只启动路径分段，检查 3D 路径、分段和当前目标。
4. 启动 SCAN，但不连接 SDK，检查 occupancy 和 `/planning/bspline`。
5. 启动闭环控制器，仅打印 `/cmd_vel`，验证符号、限幅和超时停车。
6. 接入 SDK，架空测试前进、横移、旋转和零速度。
7. 平地以不超过 0.1～0.2 m/s 低速测试。
8. 再测试缓坡，并记录定位 Z、路径 Z、B-spline Z 和实际速度。
9. 最后测试障碍绕行与更陡坡面。

## 13. 常用诊断

```bash
# 检查实际连接关系
ros2 topic info /terrain_path/current -v
ros2 topic info /planning/bspline -v
ros2 topic info /cmd_vel -v

# 检查坐标变换
ros2 run tf2_ros tf2_echo map base

# 检查规划错误
rg '\[MapRefresh\]|\[AStarDiag\]|\[ReplanDiag\]|EMERGENCY_STOP' <日志文件>

# 立即人工停车（SDK桥应订阅或提供等价急停接口）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

人工发布一次零速度不能替代 SDK 看门狗；SDK 桥必须在命令超时后自行持续停车。

## 14. 当前仿真复现

当前完整仿真入口：

```bash
cd /home/wei/github_code/SCAN-Planner-Ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch terrain_path_segmenter gazebo_segmented_demo.launch.py
```

该演示包括 Gazebo、3D LiDAR、路径自动分段、SCAN remap、新路径地图刷新、
B-spline 闭环控制和 RViz。Gazebo 中使用的是平面运动学近似，不代表真机四足步态动力学。

