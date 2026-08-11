# SCAN-Planner ROS 2 Jazzy 远程部署手册

本文记录在 Ubuntu 24.04、ROS 2 Jazzy 主机上部署本仓库的完整流程。验证主机为
`langyi@192.168.223.134`，验证工作空间为
`/home/langyi/wyf/SCAN-Planner-Ros2`。

## 1. 基础环境

目标机需要预先配置 Ubuntu 24.04 和 ROS 2 Jazzy 软件源。以下命令不会写入
`.bashrc`，每个新终端都应先加载 ROS 环境：

```bash
source /opt/ros/jazzy/setup.bash
```

安装 Git，并更新软件包索引：

```bash
sudo apt-get update
sudo apt-get install -y git
```

## 2. 复制工作空间

在目标机创建目录：

```bash
mkdir -p /home/langyi/wyf/SCAN-Planner-Ros2
```

在源主机执行。此命令保留源码、资源和 Git 历史，但不复制本机编译产物：

```bash
rsync -a --info=progress2 \
  --exclude=/build/ \
  --exclude=/install/ \
  --exclude=/log/ \
  --exclude=/.pytest_cache/ \
  /home/wei/github_code/SCAN-Planner-Ros2/ \
  langyi@192.168.223.134:/home/langyi/wyf/SCAN-Planner-Ros2/
```

核对仓库和 ROS 包数量：

```bash
cd /home/langyi/wyf/SCAN-Planner-Ros2
git status --short
git log -1 --oneline
find src -name package.xml -print | wc -l
```

当前仓库应包含 15 个 ROS 包。

## 3. 安装依赖

首次使用 rosdep 时初始化。若系统已经存在 rosdep 源，则跳过 `rosdep init`：

```bash
source /opt/ros/jazzy/setup.bash

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro jazzy
```

在工作空间根目录按 package.xml 安装依赖：

```bash
cd /home/langyi/wyf/SCAN-Planner-Ros2
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
  --skip-keys ament_python
```

`ament_python` 是 `scan_planner_analysis` 的构建类型，而不是可由 rosdep 安装的
系统包，因此这里明确跳过。其余依赖仍由 rosdep 检查和安装。

本次验证中，rosdep 实际需要的直接软件包如下；当 rosdep 不可用时可作为手动
安装的后备命令：

```bash
sudo apt-get install -y \
  libglm-dev \
  ros-jazzy-controller-manager \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-xacro
```

验证依赖是否完整：

```bash
rosdep check --from-paths src --ignore-src --rosdistro jazzy \
  --skip-keys ament_python
```

若 apt 报告系统原本就存在 broken dependency，先只模拟修复并确认没有意外删除：

```bash
sudo apt-get --fix-broken install --simulate
```

确认模拟结果后再执行：

```bash
sudo apt-get --fix-broken install -y
```

这是故障恢复步骤，不是每台主机都必须执行。本次目标机原有 `ikuuuvpn` 缺少
`libkeybinder-3.0-0`，修复命令只补装了该库；随后 rosdep 安装成功。

## 4. Release 编译与测试

```bash
cd /home/langyi/wyf/SCAN-Planner-Ros2
source /opt/ros/jazzy/setup.bash

set -o pipefail
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  2>&1 | tee /home/langyi/wyf/scan_planner_build.log
```

编译完成后加载工作空间并运行测试：

```bash
source install/setup.bash
colcon test --event-handlers console_direct+ \
  2>&1 | tee /home/langyi/wyf/scan_planner_test.log
colcon test-result --verbose \
  2>&1 | tee /home/langyi/wyf/scan_planner_test_result.log
```

本次验证结果为 15 个包全部完成 Release 编译。测试共 46 项；路径发布器的
launch 测试首次收尾超时，单独复跑后通过，其余测试一次通过。

## 5. 真机链路启动

每个终端先执行：

```bash
cd /home/langyi/wyf/SCAN-Planner-Ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

在有图形桌面的目标机启动整套系统：

```bash
ros2 launch scan_planner real_go2_livox.launch.py \
  rviz:=true \
  enable_motion:=false
```

首次验证务必保持 `enable_motion:=false`。确认点云、里程计、全局路径、局部轨迹
及速度方向正确后，再由操作人员显式改为 `true`。

无图形桌面的 SSH 冒烟测试使用：

```bash
ros2 launch scan_planner real_go2_livox.launch.py \
  rviz:=false \
  analysis:=false \
  enable_motion:=false
```

## 6. up_and_down bag 回放

将 bag 复制到目标机：

```bash
mkdir -p /home/langyi/wyf/bags/up_and_down
rsync -a --partial --append-verify --info=progress2 \
  /home/wei/bag/up_and_down/ \
  langyi@192.168.223.134:/home/langyi/wyf/bags/up_and_down/
```

建议在两个远端终端中设置相同且未占用的测试域，避免与现场 ROS 网络互相影响：

```bash
export ROS_DOMAIN_ID=91
```

终端一启动规划系统：

```bash
cd /home/langyi/wyf/SCAN-Planner-Ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=91

ros2 launch scan_planner real_go2_livox.launch.py \
  rviz:=false \
  analysis:=false \
  enable_motion:=false
```

终端二回放 bag：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=91

ros2 bag play /home/langyi/wyf/bags/up_and_down \
  --start-offset 15 \
  --clock 10 \
  --topics /plan /lio_odom_hf /livox/lidar
```

`--clock 10` 表示以 10 Hz 发布 `/clock`，不是进度条。查看当前 bag 时间可在第三个
终端执行：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=91
ros2 topic echo /clock
```

重点检查话题：

```bash
ros2 topic hz /scan_planner/body_pose
ros2 topic hz /scan_planner/cloud
ros2 topic hz /scan_planner/initial_path
ros2 topic hz /planning/bspline
ros2 topic hz /planning/bspline_path
ros2 topic echo /cmd_vel_smoothed
```

在 `enable_motion:=false` 时，规划链路仍会产生局部轨迹和原始控制速度，但安全门
最终输出 `/cmd_vel_smoothed` 应保持零速度。

## 7. 本次目标机日志

首次迁移的完整日志保存在：

```text
/home/langyi/wyf/setup_logs/01_apt_update.log
/home/langyi/wyf/setup_logs/02_install_git.log
/home/langyi/wyf/setup_logs/03_rosdep_init.log
/home/langyi/wyf/setup_logs/04_rosdep_update.log
/home/langyi/wyf/setup_logs/05_rosdep_simulate.log
/home/langyi/wyf/setup_logs/07_fix_broken_simulate.log
/home/langyi/wyf/setup_logs/08_fix_broken_install.log
/home/langyi/wyf/setup_logs/09_rosdep_install_retry.log
/home/langyi/wyf/setup_logs/10_colcon_build_release.log
/home/langyi/wyf/setup_logs/11_colcon_test.log
/home/langyi/wyf/setup_logs/12_colcon_test_result.log
/home/langyi/wyf/setup_logs/13_reference_path_launch_retest.log
/home/langyi/wyf/setup_logs/14_reference_path_launch_retest_result.log
```
