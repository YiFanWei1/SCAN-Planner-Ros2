/**
 * @file scan_replan_fsm.cpp
 * @brief SCAN 局部轨迹重规划有限状态机实现。
 *
 * 状态机流程图（Mermaid；可粘贴到支持 Mermaid 的 Markdown 查看器中渲染）：
 *
 * ```mermaid
 * stateDiagram-v2
 *     [*] --> INIT
 *     INIT --> INIT: 尚未收到里程计或启动触发
 *     INIT --> WAIT_TARGET: 已有里程计且已触发
 *     WAIT_TARGET --> WAIT_TARGET: 尚无有效目标
 *     WAIT_TARGET --> GEN_NEW_TRAJ: 收到有效目标/参考路径
 *     GEN_NEW_TRAJ --> GEN_NEW_TRAJ: 地图未就绪或局部规划失败（限频重试）
 *     GEN_NEW_TRAJ --> EXEC_TRAJ: 首条局部 B 样条生成成功
 *     EXEC_TRAJ --> REPLAN_TRAJ: 离开起点区且尚未接近终点
 *     REPLAN_TRAJ --> EXEC_TRAJ: 从当前状态重规划成功
 *     REPLAN_TRAJ --> REPLAN_TRAJ: 重规划失败（限频重试）
 *     EXEC_TRAJ --> GEN_NEW_TRAJ: 到达当前预设航点，切换下一航点
 *     EXEC_TRAJ --> WAIT_TARGET: 轨迹结束/最终航点完成
 *     GEN_NEW_TRAJ --> EMERGENCY_STOP: 连续失败达到上限
 *     REPLAN_TRAJ --> EMERGENCY_STOP: 连续失败达到上限
 *     EXEC_TRAJ --> EMERGENCY_STOP: 安全检测发现临近碰撞且重规划失败
 *     EXEC_TRAJ --> REPLAN_TRAJ: 安全检测发现较远碰撞且重规划失败
 *     EMERGENCY_STOP --> GEN_NEW_TRAJ: 停稳后故障保护重试
 *     EMERGENCY_STOP --> WAIT_TARGET: 停稳后放弃旧目标，等待新目标
 * ```
 *
 * 两个定时器协同工作：execFSMCallback() 每 10 ms 推进主状态机；
 * checkCollisionCallback() 每 50 ms 前瞻检查当前局部轨迹，并可抢先触发重规划或急停。
 */

#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/reference_path_utils.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
  // 读取 ROS 2 参数；若参数尚未声明，则先用默认值声明，兼容未开启自动声明参数的节点。
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    // has_parameter() 可避免重复声明导致 ParameterAlreadyDeclaredException。
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    // get_value<T>() 将参数值按调用处指定的模板类型取出。
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    // 保存节点裸指针；该 FSM 的生命周期必须短于其所属 ROS 2 节点。
    node_ = node;
    // 从第 0 个航点开始，并将所有运行标志复位到“尚未启动”的确定状态。
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;  // true 表示进入急停后尚未发布本轮停止轨迹。
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();  // 冻结补偿按增量时间计算，初始化基准时刻。

    /* 状态机参数：负默认值用于暴露缺失配置；具有安全意义的参数则提供保守默认值。 */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    replan_retry_interval_ = load_parameter<double>(node_, "fsm.replan_retry_interval", 0.1);
    replan_retry_interval_ = std::max(0.01, replan_retry_interval_);  // 最快 100 Hz，避免失败时忙循环。
    auto_retry_after_failures_ =
        load_parameter<bool>(node_, "fsm.auto_retry_after_failures", false);
    failure_retry_cooldown_ =
        std::max(0.1, load_parameter<double>(node_, "fsm.failure_retry_cooldown", 1.0));
    // 将上次尝试时间回拨一个间隔，使首次进入规划状态时可以立即尝试。
    last_replan_attempt_time_ = node_->now() - rclcpp::Duration::from_seconds(replan_retry_interval_);
    failure_emergency_start_time_ = node_->now();
    last_odom_receive_time_ = node_->now();
    last_target_receive_time_ = node_->now();
    last_successful_traj_time_ = node_->now();
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.0);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");

    // 预设航点参数以 [x0,y0,z0,x1,y1,z1,...] 的扁平数组表示。
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);  // 每三个标量组成一个三维航点。
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* 初始化可视化与规划管理器；规划管理器内部继续初始化地图、优化器等核心模块。 */
    visualization_.reset(new PlanningVisualization(node_, self_inflation_frame_id_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* 注册回调：主 FSM 100 Hz，安全前瞻 20 Hz。 */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    // 里程计使用 SensorDataQoS，以低延迟和允许丢弃旧数据为优先。
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    // 下游执行器冻结时，同步冻结本地轨迹的逻辑时间，防止规划器误判轨迹已经执行完。
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(10).reliable().transient_local());

    // 三种导航模式只订阅各自所需的目标来源，避免同一时刻接收相互冲突的目标。
    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    // 复制配置航点到局部变量，便于在不修改原始参数的情况下展示和激活任务。
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    // 在 RViz 中逐个显示航点；索引 i 同时作为 Marker ID，保证各点不会互相覆盖。
    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;  // 保存当前实际执行的航点序列。
    current_wp_ = 0;
    trigger_ = true;          // 允许 INIT 状态继续向 WAIT_TARGET 转移。
    init_pt_ = odom_pos_;     // 记录整次任务的初始位置。

    // 先为第一个航点建立全局轨迹，成功后再让 FSM 生成局部轨迹。
    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    // 防御空智能指针，避免异常发布者导致解引用崩溃。
    if (!msg)
      return;

    // 手动目标的 z 高度取首次里程计高度，因此首次位姿到来前不能接受 RViz 二维目标。
    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    // 将单个 PoseStamped 包装成单点 Path，复用统一的 waypointCallback() 处理流程。
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    // 空消息或无位姿的消息不具备规划意义；日志限频避免错误输入刷屏。
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    // 保留上游约定：z<-0.1 的目标视为无效/取消信号，不触发规划。
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    RCLCPP_DEBUG(node_->get_logger(), "Waypoint trigger received");
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    // RViz 的 2D Nav Goal 只采用 x/y，z 固定为机器人首次里程计高度。
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    // 以实时里程计位置/速度为起点，终点速度和加速度均约束为零。
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 若目标落在膨胀障碍内，则尝试沿全局轨迹回退到最近的自由点。
    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;  // 仅用于可视化采样，不改变真实轨迹分辨率。
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();       // 当前接口要求最终静止。
      have_target_ = true;     // WAIT_TARGET 可以进入生成轨迹状态。
      have_new_target_ = true; // 通知局部规划器重新初始化，而非沿用旧目标初始化。

      /*** FSM ***/
      // 空闲时生成首条轨迹；执行中收到新目标时从当前运动状态平滑重规划。
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    // 首点是参考路径起点，后续点才是传给全局插值器的途经点/终点。
    if (waypoints.size() < 2)
    {
      RCLCPP_WARN(node_->get_logger(), "Reference path requires at least two points");
      return false;
    }

    end_pt_ = waypoints.back();  // 最后一个参考点是本次导航最终目标。
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() + 1, waypoints.end());

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    // 参考路径两端均按零速度、零加速度边界生成平滑的全局轨迹。
    bool success = planner_manager_->planGlobalTrajWaypoints(
        waypoints.front(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        reference_waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;  // 按时间采样全局轨迹供 RViz 显示。
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    // 防止航点索引越界；这里也覆盖空 active_waypoints_ 的情况。
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];  // 将当前序列点提升为本段全局目标。
    // 优先使用有效旧轨迹的导数作为速度/加速度，以减少新旧轨迹切换突变。
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    // 规划结果的末端可能因地图更新变成占用，此处统一实施终点回退。
    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    // 目前只有 PRESET_TARGET 支持“到一个点后自动切换下一点”的序列行为。
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    // 使用共享指针副本延长本函数执行期间地图对象的生命周期。
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    // 无地图或退化的极短轨迹无法检查；保持原目标交由后续流程处理。
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    // 双圆柱膨胀模型与朝向相关，因此用轨迹末段方向估计机器人偏航角。
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    // 从终点反向搜索，选取最靠近原目标的自由采样点，尽量保留任务进度。
    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;  // 截短全局轨迹，使后续局部目标搜索不会越过新终点。
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    // 参考路径为空时忽略；限频日志适合上游周期发布空路径的场景。
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    // prepareReferenceWaypoints() 需要用当前机器人状态校验/衔接路径，故必须先有里程计。
    if (!have_odom_)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "No odometry yet; ignoring initial_path");
      return;
    }

    std::vector<Eigen::Vector3d> waypoints;
    std::string path_error;
    // 清洗参考路径、补偿机体高度，并以 0.5 m 尺度处理相邻参考点。
    if (!prepareReferenceWaypoints(*msg, body_height_, 0.5, waypoints, &path_error))
    {
      RCLCPP_WARN(node_->get_logger(), "Ignoring initial_path: %s", path_error.c_str());
      return;
    }

    trigger_ = true;
    last_target_receive_time_ = node_->now();  // 保存诊断时间，不代表局部轨迹已经成功。
    end_pt_ = waypoints.back();
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      // 更新全局参考路径不等于局部重规划成功。/plan 可能持续刷新；若在这里清零失败计数，
      // 会掩盖连续的 A* 失败，使“达到上限后急停/重试”策略永远无法触发。
      // 因此只有真正发布有效局部 B 样条时，状态机才清零 replan_fail_count_。
      last_replan_attempt_time_ = node_->now() - rclcpp::Duration::from_seconds(replan_retry_interval_);
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_DEBUG(node_->get_logger(), "Reference path accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    last_odom_receive_time_ = node_->now();  // 用接收时刻而非消息时间戳衡量通信新鲜度。
    // 提取世界坐标系下的位置。
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    // RViz 2D 目标本身没有可靠高度，首次里程计到达时锁定本次运行的目标高度。
    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    // 提取线速度，供轨迹起点连续性约束使用。
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // 当前未从里程计估计加速度；规划起点加速度由旧轨迹给出或置零。
    // odom_acc_ = estimateAcc(msg);

    // ROS 四元数消息转换为 Eigen 四元数，后续用于计算机器人航向角。
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();  // 每次位姿更新时同步刷新机体双圆柱包络。
    // 预设模式只自动启动一次，避免每条里程计消息重复重置航点任务。
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    // true 表示下游控制器暂时没有推进轨迹时间。
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    // 计算距上次主/安全回调执行的墙钟增量；两个定时器共享同一时间基准。
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    // 时间回拨或长时间调度停顿时不补偿，避免一次性将轨迹起点推到不合理的未来。
    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    // 起点时间向后平移 dt，令 (now-start_time) 保持不变，即冻结轨迹逻辑进度。
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    // 旋转矩阵第一列是机体 x 轴在世界坐标系中的朝向。
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    // 水平投影退化时返回 0，避免 atan2(0,0) 产生没有物理意义的航向。
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    // 仅使用 xy 位移估计路径切向航向，z 变化不影响地面机体偏航角。
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    // 零长度线段无法给出方向，退化为当前里程计航向。
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    // 配置异常为负时钳制到 0；Marker 高度至少为 1 mm，保证 RViz 可渲染。
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);  // 超过两次安全周期未刷新则自动消失。

    // z 中心由机器人位置加上下膨胀高度的不对称偏置得到。
    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    // 沿机体前后方向放置两个圆柱，近似非圆形机身占用包络。
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;  // 前圆柱使用蓝色。
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    // 后圆柱使用橙色，使两个重叠圆柱在 RViz 中仍可区分。
    marker.color.r = 1.0;
    marker.color.g = 0.45;
    marker.color.b = 0.1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    // 统计同一状态被连续“再次设置”的次数；首轮规划采用确定性初始化，后续失败重试可随机化。
    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    // 状态名称数组保留用于调试；枚举实际只有 6 项，多出的容量不会被访问。
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    // 当前版本关闭了状态切换日志，用 void 转换抑制未使用变量告警。
    (void)pos_call;
    (void)pre_s;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    // 同时返回连续次数和当前状态，便于调用者确认计数所属状态。
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    // 枚举值按声明顺序直接映射为可读名称。
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    RCLCPP_DEBUG(node_->get_logger(), "FSM state: %s", state_str[int(exec_state_)].c_str());
  }

  void SCANReplanFSM::execFSMCallback()
  {
    // 若执行器被冻结，先修正轨迹起点时间，再计算本轮轨迹进度。
    updateLocalTrajTimeFreeze();

    // 10 ms 回调累计 100 次约为 1 秒，以 DEBUG 级别周期打印当前状态。
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      // if (have_target_)
      // {
      //   const rclcpp::Time now = node_->now();
      //   const double traj_gap = have_successful_traj_
      //                               ? (now - last_successful_traj_time_).seconds()
      //                               : (now - last_target_receive_time_).seconds();
      //   if (traj_gap > 1.0)
      //   {
      //     static const char *state_names[] = {
      //         "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
      //     const int state_index = static_cast<int>(exec_state_);
      //     const char *state_name = state_index >= 0 && state_index < 6
      //                                  ? state_names[state_index]
      //                                  : "UNKNOWN";
      //     const double odom_age = have_odom_
      //                                 ? std::max(0.0, (now - last_odom_receive_time_).seconds())
      //                                 : std::numeric_limits<double>::infinity();
      //     const double cloud_age = planner_manager_->grid_map_->getLastCloudAge();
      //     const LocalTrajData &local = planner_manager_->local_data_;
      //     const double local_elapsed = local.start_time_.seconds() > 1e-5
      //                                      ? (now - local.start_time_).seconds()
      //                                      : -1.0;
      //     RCLCPP_WARN(node_->get_logger(),
      //                 "[LocalPathGap] age=%.2fs state=%s target=%d new_target=%d failures=%d "
      //                 "odom_age=%.3fs cloud_age=%.3fs local_elapsed=%.2fs local_duration=%.2fs "
      //                 "odom=[%.3f %.3f %.3f] target=[%.3f %.3f %.3f]",
      //                 traj_gap, state_name, have_target_, have_new_target_, replan_fail_count_,
      //                 odom_age, cloud_age, local_elapsed, local.duration_,
      //                 odom_pos_(0), odom_pos_(1), odom_pos_(2),
      //                 end_pt_(0), end_pt_(1), end_pt_(2));
      //   }
      // }
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      // 初始化阶段必须同时等到里程计和任一导航模式产生的启动触发。
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      // 空闲等待有效全局目标；目标回调负责设置 have_target_。
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      // 不允许基于全零的未初始化占用栅格规划。目标可能早于首帧激光融合到达；
      // 若无此门控，首条轨迹可能穿墙，只能等后续安全检查再纠正。
      if (!planner_manager_->grid_map_->occupancyMapReady())
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Waiting for occupancy map initialization (%d fused cloud updates)",
            planner_manager_->grid_map_->completedOccupancyUpdates());
        break;
      }

      // 失败重试限频，防止 100 Hz FSM 持续调用代价高昂的搜索与优化。
      if ((node_->now() - last_replan_attempt_time_).seconds() < replan_retry_interval_)
        break;
      last_replan_attempt_time_ = node_->now();

      // 位置始终取实时里程计；若旧轨迹仍有效，则复用其速度/加速度以保证连续性。
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      // 第一次尝试使用确定性多项式初始化；停留在本状态后的重试启用随机初始化以跳出局部最优。
      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {
        // 只有有效局部轨迹生成并发布成功，才清零连续失败计数。
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;  // 为未来可能进入急停准备“一次性发布”标志。
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      // 与首轨迹生成共享同一重试时间戳，安全定时器也无法绕过该限频。
      if ((node_->now() - last_replan_attempt_time_).seconds() < replan_retry_interval_)
        break;
      last_replan_attempt_time_ = node_->now();

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* 根据当前局部轨迹进度决定继续执行、切换航点、结束任务或触发重规划。 */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);  // 上界钳制，避免在 B 样条定义域外求值。

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      // 序列模式下，实际里程计距当前航点小于 0.5 m 时可提前规划下一段。
      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      // 当前局部轨迹时间耗尽：序列模式切换下一航点，否则认为本任务完成并等待新目标。
      if (t_cur > info->duration_ - 1e-2)
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;  // 防止 WAIT_TARGET 立即使用已经完成的旧目标再次启动。

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      // 已进入终点附近的禁止重规划区：继续执行当前轨迹，避免终点处频繁抖动。
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      // 尚未离开轨迹起点附近的重规划阈值：先执行一段，避免刚发布就重复规划。
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        // 已离开起点且未接近终点，滚动进入下一轮局部重规划。
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      // 每次进入急停仅发布一次零速停止轨迹，避免 100 Hz 重复覆盖控制器命令。
      if (flag_escape_emergency_)
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        // 碰撞触发的普通急停：停稳后保留目标并重新生成局部轨迹。
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          // 连续失败上限触发的悬停：根据策略在冷却后重试旧目标，或丢弃旧目标等待新任务。
          const bool cooldown_elapsed =
              (node_->now() - failure_emergency_start_time_).seconds() >= failure_retry_cooldown_;
          if (auto_retry_after_failures_ && have_target_ && cooldown_elapsed)
          {
            RCLCPP_WARN(node_->get_logger(),
                        "Failure cooldown elapsed; retrying the retained target from current odometry");
            need_hover_stop_ = false;
            replan_fail_count_ = 0;
            last_replan_attempt_time_ =
                node_->now() - rclcpp::Duration::from_seconds(replan_retry_interval_);
            changeFSMExecState(GEN_NEW_TRAJ, "EMERGENCY_RETRY");
          }
          else if (!auto_retry_after_failures_ || !have_target_)
          {
            RCLCPP_INFO(node_->get_logger(),
                        "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
            need_hover_stop_ = false;
            have_target_ = false;
            trigger_ = false;
            changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
          }
        }
      }

      flag_escape_emergency_ = false;  // 标记停止轨迹已经发布。
      break;
    }
    }

    // 统一在每轮状态处理后检查连续失败上限，必要时覆盖当前状态进入急停。
    finishProcess();

    // 发布规划诊断数据；即使本轮没有生成轨迹，也保持外部显示更新。
    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    // 连续失败达到配置上限时停止继续搜索，先发布急停轨迹保证安全。
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      if (auto_retry_after_failures_)
        RCLCPP_WARN(node_->get_logger(),
                    "Replan failed %d times; emergency stop, then retry retained target after %.2fs",
                    replan_fail_count_, failure_retry_cooldown_);
      else
        RCLCPP_WARN(node_->get_logger(),
                    "Replan failed %d times; emergency stop and wait for a new target",
                    replan_fail_count_);
      replan_fail_count_ = 0;       // 急停事件已消费本轮失败累计。
      need_hover_stop_ = true;      // 区分“失败过多”与普通碰撞触发的急停。
      flag_escape_emergency_ = true;// 允许 EMERGENCY_STOP 首轮发布一次停止轨迹。
      failure_emergency_start_time_ = node_->now();
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    // 计算旧局部轨迹的当前逻辑时刻，并钳制到合法参数域。
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    // 参考路径模式的控制器按几何投影跟踪样条，时间进度不能可靠代表机器人实际进度。
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      // 因此直接采用实时里程计位置/速度，起点加速度保守置零。
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      const Eigen::Vector2d to_goal = end_pt_.head<2>() - start_pt_.head<2>();
      // 非有限速度或速度背离目标都会破坏初始化，此时从静止状态重新规划更稳健。
      if (!start_vel_.allFinite() ||
          (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0))
        start_vel_.setZero();

      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Reference replan from live odom: [%.2f %.2f %.2f]",
                           start_pt_(0), start_pt_(1), start_pt_(2));

      // 从实时状态重建初始化，不保留上一段可能已经过时的控制点；
      // 确定性初始化失败后，再以随机多项式初始化补尝试一次。
      bool success = callReboundReplan(true, false);
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
          return false;
      }

      return true;
    }

    // 非参考路径模式：位置取实测值，速度/加速度取旧样条同一时刻的导数以平滑衔接。
    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    // 若旧轨迹速度朝向目标反方向，则清除导数，避免新全局轨迹先倒退再前进。
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    // 用更新后的实时起点重建通往当前终点的全局引导轨迹。
    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    // 先确定性初始化，失败后随机化初始化再尝试一次。
    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    // 默认起点：实时位置/速度与零加速度，适用于尚无可用旧轨迹的情况。
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    // 起始时间或时长未初始化，不能从旧样条提取导数。
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    // 旧轨迹尚未开始或已过期超过 0.2 s 时，不再使用其导数。
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    // 在合法时间范围内采用旧轨迹导数，使新轨迹至少保持一、二阶连续趋势。
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    // 安全回调和主 FSM 都会读取轨迹时间，必须应用相同的执行冻结补偿。
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // 无目标或无有效局部轨迹时没有可检查的对象。
    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;  // 以 10 ms 时间分辨率扫描未来轨迹。
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      // 若当前仍处于前 2/3，只检查到 2/3 分界；末段会由后续滚动重规划更新，避免过远误报。
      if (t_cur < t_2_3 && t >= t_2_3)
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      // 按相邻采样点估计机体航向，再用朝向相关的膨胀占用检测碰撞。
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        // 安全定时器运行于 20 Hz；与主 FSM 共享尝试时间戳，避免形成第二条高频优化循环。
        const auto now = node_->now();
        const bool retry_ready =
            (now - last_replan_attempt_time_).seconds() >= replan_retry_interval_;
        if (retry_ready)
          last_replan_attempt_time_ = now;

        // 若重试间隔已到，先给在线重规划一次受限机会，成功即可继续执行。
        if (retry_ready && planFromCurrentTraj())
        {
          replan_fail_count_ = 0;
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (retry_ready)
            replan_fail_count_++;
          // 碰撞时间小于安全窗口时直接急停；否则交给常规重规划状态处理。
          if (t - t_cur < emergency_time_)
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;  // 前面各分支均已 return，此处保留为防御性退出。
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // 先在全局引导轨迹上选择规划视野内的局部目标及目标速度。
    getLocalTarget();

    // have_new_target_ 或显式 flag_use_poly_init 会要求优化器重新构造多项式初值；
    // flag_randomPolyTraj 控制该初值是否加入随机扰动以摆脱失败的局部最优。
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;  // 新目标标志只消费一次，后续属于对同一目标的滚动重规划。

    if (!plan_success)
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Local replan failed: start=[%.3f %.3f %.3f], target=[%.3f %.3f %.3f]",
                           start_pt_(0), start_pt_(1), start_pt_(2),
                           local_target_pt_(0), local_target_pt_(1), local_target_pt_(2));

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;  // reboundReplan 成功后已写入最新局部轨迹。

      /* 将内部均匀 B 样条转换为 ROS 消息并发布给轨迹执行器。 */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      // 消息按控制点列顺序存储三维位置。
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      // 完整复制节点向量，执行端据此还原同一条 B 样条。
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);
      last_successful_traj_time_ = node_->now();  // 仅作轨迹输出新鲜度诊断。
      // have_successful_traj_ = true;

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    // 规划管理器在 stop_pos 处生成一条零速度/悬停 B 样条，并写入 local_data_。
    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* 发布格式与正常局部轨迹完全一致，执行器无需区分急停消息类型。 */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;  // EmergencyStop 当前没有失败返回通道，生成并发布后即视为成功。
  }

  void SCANReplanFSM::getLocalTarget()
  {
    // 读取动力学限制和当前全局轨迹时长，用于确定采样步长、视野和制动距离。
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    const double duration = planner_manager_->global_data_.global_duration_;
    // 使一个 planning_horizon_ 大约包含 20 个时间采样；零/极小速度时使用安全下限。
    double t_step = max_vel > 1e-6 ? planning_horizon_ / 20.0 / max_vel : 0.01;
    t_step = std::max(t_step, 0.01);

    // 第一遍遍历：找到 start_pt_ 在全局轨迹上的最近采样点，作为当前进度投影。
    double t_proj = 0.0;
    double min_dist_to_start = 9999.0;
    for (double t = 0.0; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist_to_start = (pos_t - start_pt_).norm();
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        t_proj = t;
      }
    }

    // 第二遍从投影点沿轨迹累计弧长，选择一个 planning_horizon_ 距离外的局部目标。
    double target_t = duration;
    double total_dist = 0.0;
    bool target_found = false;
    Eigen::Vector3d prev_pos = planner_manager_->global_data_.getPosition(t_proj);
    local_target_pt_ = end_pt_;  // 若剩余弧长不足视野，则直接使用最终目标。

    for (double t = t_proj; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      total_dist += (pos_t - prev_pos).norm();
      if (total_dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        target_found = true;
        break;
      }
      prev_pos = pos_t;
    }
    // 保存全局轨迹进度，供其他规划模块诊断或继续截取使用。
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration;

    // 局部目标占用检查同样使用从机器人当前位置指向目标的估计航向。
    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    // 目标被占用时，以原 target_t 为中心沿全局轨迹时间轴向前、向后交替搜索自由点。
    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= duration; dt += t_step)
      {
        // 优先向前搜索，以尽可能保持导航进度。
        double t_forward = target_t + dt;
        if (t_forward <= duration)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        // 前方没有自由点时检查后方，但不退到当前投影进度之前。
        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, t_proj))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
    }

    // 按 v²/(2a) 估算最大速度下的制动距离；加速度无效时视为无法可靠制动。
    const double braking_distance = max_acc > 1e-6
        ? (max_vel * max_vel) / (2.0 * max_acc)
        : std::numeric_limits<double>::infinity();
    // 局部目标进入制动区后要求目标速度为零，让优化器提前减速并最终停在终点。
    if ((end_pt_ - local_target_pt_).norm() < braking_distance)
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      // 制动区外沿用全局轨迹速度方向，并将其幅值限制在最大速度内。
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
