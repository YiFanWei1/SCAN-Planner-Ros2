
#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/reference_path_utils.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    // 如果参数尚未在 ROS2 参数服务器中声明，则先用默认值声明，避免直接读取时报错。
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    // 返回已经声明或外部配置覆盖后的实际参数值。
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;                         // 保存 ROS2 节点指针，后续创建话题、定时器和读取时间都依赖它。
    current_wp_ = 0;                      // 当前正在跟踪的航点索引，预设航点模式下从第 0 个开始。
    exec_state_ = FSM_EXEC_STATE::INIT;   // 状态机初始进入 INIT，等待里程计和触发信号。
    trigger_ = false;                     // 尚未收到目标/路径触发。
    have_target_ = false;                 // 尚未生成可执行目标。
    have_odom_ = false;                   // 尚未收到机体位姿/速度。
    have_new_target_ = false;             // 暂无新目标需要让局部规划器重新初始化。
    rviz_height_ready_ = false;           // RViz 手动目标的高度需要用首帧里程计 z 值初始化。
    go2_execution_frozen_ = false;        // 默认认为 Go2 执行端没有冻结。
    flag_escape_emergency_ = true;        // 进入急停状态时允许首次发布急停轨迹。
    need_hover_stop_ = false;             // 默认不需要急停后悬停等待新目标。
    replan_fail_count_ = 0;               // 清零连续重规划失败次数。
    last_freeze_update_time_ = node_->now(); // 记录冻结补偿的上一次更新时间。

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);                 // 导航模式：1 手动目标，2 预设航点，3 参考路径。
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);    // 距离触发重规划的阈值。
    replan_period_ = load_parameter<double>(node_, "fsm.replan_period", 0.0);     // 时间触发重规划的周期；小于等于 0 时退回距离触发。
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0); // 接近终点后不再重规划的距离阈值。
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0); // 局部目标沿全局轨迹向前看的距离。
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);   // 碰撞距离当前时刻小于该时间则立即急停。
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);       // 是否启用急停后的自动恢复/等待逻辑。
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000); // 连续失败超过该次数后进入急停。
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0); // 自身膨胀体向上的高度。
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0); // 自身膨胀体向下的高度。
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0); // 双圆柱碰撞模型半径。
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0); // 前后圆柱相对中心的偏移。
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.0);    // 机体高度，用于参考路径高度修正。
    reference_path_min_distance_ =
        load_parameter<double>(node_, "fsm.reference_path_min_distance", 0.5);
    clear_map_on_new_path_ =
        load_parameter<bool>(node_, "fsm.clear_map_on_new_path", false);
    fresh_observations_before_planning_ =
        load_parameter<int>(node_, "fsm.fresh_observations_before_planning", 2);
    map_refresh_warning_timeout_ =
        load_parameter<double>(node_, "fsm.map_refresh_warning_timeout", 0.5);
    if (replan_period_ < 0.0) // 重规划周期不能为负，0 表示不使用周期触发。
      throw std::runtime_error("fsm.replan_period must be non-negative");
    if (fresh_observations_before_planning_ < 1) // 清图后至少等待一帧新观测，否则可能用空地图规划。
      throw std::runtime_error("fsm.fresh_observations_before_planning must be at least 1");
    if (map_refresh_warning_timeout_ <= 0.0) // 超时提示间隔必须为正数，避免节流逻辑异常。
      throw std::runtime_error("fsm.map_refresh_warning_timeout must be positive");
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world"); // RViz 膨胀体 marker 所在坐标系。

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET) // 预设航点模式：从参数中读取展平的 x,y,z 序列。
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3); // 每 3 个 double 组成一个三维航点。
      preset_waypoints_.resize(waypoint_num_);                    // 预分配航点数组，便于按索引填充。
      for (int i = 0; i < waypoint_num_; i++)
      {
        // 将参数中的扁平数组 [x0,y0,z0,x1,y1,z1,...] 转成 Eigen 三维点。
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_)); // 初始化轨迹、目标点等可视化模块。
    planner_manager_.reset(new SCANPlannerManager);         // 创建规划管理器，封装地图、全局/局部规划数据。
    planner_manager_->initPlanModules(node_, visualization_); // 初始化规划器内部模块并共享可视化对象。

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this)); // 100Hz 状态机主循环。
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this)); // 20Hz 前向碰撞检查。
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET) // 手动目标模式：订阅 RViz 的 2D/3D goal。
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH) // 参考路径模式：订阅外部发布的 nav_msgs/Path。
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_; // 拷贝预设航点，作为当前激活的航点序列。

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i); // 在 RViz 中显示每个预设航点。
    }

    active_waypoints_ = wps; // 保存待依次执行的航点队列。
    current_wp_ = 0;         // 从第一个航点开始规划。
    trigger_ = true;         // 置位触发信号，让 FSM 从 INIT/WAIT_TARGET 继续运行。
    init_pt_ = odom_pos_;    // 记录本轮任务的起点为当前里程计位置。

    if (planNextWaypoint()) // 先生成到第一个航点的全局轨迹。
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
    if (!msg) // 防御空消息，避免解引用崩溃。
      return;

    if (!rviz_height_ready_) // 手动目标只使用 x/y，高度需等首帧 odom 初始化。
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>(); // 复用 waypointCallback 的路径输入接口。
    path->header = msg->header;                          // 保留原 goal 的时间戳和坐标系。
    path->poses.push_back(*msg);                         // 单个 RViz goal 转成只有一个点的 Path。
    waypointCallback(path);                              // 交给统一的航点处理逻辑。
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty()) // 航点消息为空时直接忽略。
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1) // 负高度常被用作无效目标标记。
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;      // 通知 FSM 有目标输入。
    init_pt_ = odom_pos_; // 本次全局规划从当前里程计位置开始。

    bool success = false; // 记录全局轨迹是否生成成功。
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_; // RViz 目标沿用首帧机体高度。
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success) // 若原始终点落在障碍物膨胀区，则沿全局轨迹回退到最近可行点。
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0); // 显示最终采用的目标点。

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1; // 每 0.1s 采样一次全局轨迹用于可视化。
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t); // 采样点数量。
      vector<Eigen::Vector3d> gloabl_traj(i_end); // 保存全局轨迹采样点。
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t); // 按时间参数取位置点。
      }

      end_vel_.setZero();      // 终点期望速度设为 0，要求到达目标后停下。
      have_target_ = true;     // 标记当前已有可执行目标。
      have_new_target_ = true; // 通知局部规划器本轮需要按新目标重新初始化。

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET) // 若正在等待目标，则切到生成新局部轨迹。
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ) // 若正在执行旧轨迹，则触发重规划切换到新目标。
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0); // 发布全局路径可视化。
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.size() < 2) // 参考路径至少需要起点和终点两个点。
    {
      RCLCPP_WARN(node_->get_logger(), "Reference path requires at least two points");
      return false;
    }

    end_pt_ = waypoints.back(); // 全局目标取参考路径最后一个点。
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() + 1, waypoints.end()); // 起点单独传入，其余点作为约束航点。

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i); // 显示参考路径上的每个关键点。
    }

    bool success = planner_manager_->planGlobalTrajWaypoints( // 按参考路径航点生成全局轨迹。
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

    if (!adjustGlobalTargetIfOccupied()) // 若最终目标被占据，尝试收缩到轨迹上的可通行点。
      return false;

    constexpr double step_size_t = 0.1; // 可视化采样时间间隔。
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t); // 轨迹采样点数量。
    std::vector<Eigen::Vector3d> gloabl_traj(i_end); // 保存采样后的全局路径点。
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t); // 从全局轨迹按时间取点。
    }

    end_vel_.setZero();      // 参考路径终点默认要求速度为 0。
    have_target_ = true;     // 状态机可以开始生成局部轨迹。
    have_new_target_ = true; // 新参考路径到来，局部轨迹需要重新初始化。
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0); // 发布全局路径可视化。
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size()) // 当前航点索引越界，说明没有可规划目标。
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_]; // 将当前航点作为本段全局目标。
    setStartStateFromOdomOrCurrentTraj();     // 根据里程计和已有局部轨迹估计本次规划起点状态。

    bool success = planner_manager_->planGlobalTraj( // 从当前起点状态规划到当前航点。
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

    if (!adjustGlobalTargetIfOccupied()) // 若航点不可达/被占据，则尝试回退目标点。
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();      // 每段航点目标默认停靠。
    have_target_ = true;     // 已有当前航点目标。
    have_new_target_ = true; // 本段航点是新目标，需要重新初始化局部规划。
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET; // 目前只有预设航点模式按航点序列逐个推进。
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;          // 获取栅格地图，用于查询膨胀占据。
    auto &global_data = planner_manager_->global_data_; // 引用全局轨迹数据，必要时会缩短持续时间。
    const double duration = global_data.global_duration_; // 当前全局轨迹总时长。
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05; // 从终点向前搜索可行点时的采样间隔。
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt))); // 至少采样一次。
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration); // 原始全局轨迹终点。
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num); // 终点前一个采样点，用于估计朝向。
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0) // 终点未被占据，无需调整。
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num; // 当前检查点对应的轨迹时间。
      const double prev_t = duration * std::max(0, i - 1) / sample_num; // 前一采样点时间，用于估计 yaw。
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_; // 保存原目标，方便日志输出。
        end_pt_ = pt;                            // 将目标回退到找到的可通行点。
        global_data.global_duration_ = t;        // 同步缩短全局轨迹时长，避免继续追踪被占据终点。
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t); // 保证局部目标进度不超过新终点。
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
    if (!msg || msg->poses.empty()) // 外部路径为空时不触发规划。
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    if (!have_odom_) // 没有当前位姿无法确定参考路径起点和高度关系。
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "No odometry yet; ignoring initial_path");
      return;
    }

    if (clear_map_on_new_path_) // 新路径到来时先清地图，等待新观测融合后再规划。
    {
      pending_reference_path_ = msg;                 // 暂存参考路径，等地图刷新完成后再处理。
      planner_manager_->grid_map_->resetForReferencePath(); // 清理旧地图信息，避免沿新路径规划时受旧障碍残留影响。
      required_observation_sequence_ =
          planner_manager_->grid_map_->getCompletedObservationSequence() +
          static_cast<uint64_t>(fresh_observations_before_planning_); // 计算需要等待到的观测序号。
      map_refresh_request_time_ = node_->now(); // 记录开始等待新地图的时间。
      map_refresh_timeout_warned_ = false;      // 新一轮等待重新允许输出超时提醒。
      RCLCPP_INFO(node_->get_logger(),
                  "[MapRefresh] Deferred reference path until %d fresh map observations "
                  "have been fused (target_sequence=%llu)",
                  fresh_observations_before_planning_,
                  static_cast<unsigned long long>(required_observation_sequence_));
      return;
    }

    processReferencePath(msg); // 不需要清图时立即处理参考路径。
  }

  void SCANReplanFSM::processReferencePath(
      const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {

    std::vector<Eigen::Vector3d> waypoints; // 保存预处理后的三维参考航点。
    std::string path_error;                 // 保存路径预处理失败原因，便于日志说明。
    if (!prepareReferenceWaypoints(
            *msg, body_height_, reference_path_min_distance_, waypoints, &path_error))
    {
      RCLCPP_WARN(node_->get_logger(), "Ignoring initial_path: %s", path_error.c_str());
      return;
    }

    trigger_ = true;              // 参考路径有效，触发 FSM 开始/更新任务。
    end_pt_ = waypoints.back();   // 任务终点为预处理后路径最后一个点。
    bool success = planGlobalTrajByWaypoints(waypoints); // 基于整条参考路径生成全局轨迹。

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET) // 如果此前在等目标，切到新轨迹生成。
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ) // 如果正在执行，切到重规划以接入新路径。
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  bool SCANReplanFSM::processPendingReferencePath()
  {
    if (!pending_reference_path_) // 没有延迟处理的路径，主循环可继续执行。
      return false;

    const uint64_t current_sequence =
        planner_manager_->grid_map_->getCompletedObservationSequence(); // 当前已经融合完成的地图观测序号。
    if (current_sequence < required_observation_sequence_) // 观测数量还不够，继续推迟规划。
    {
      const double elapsed = (node_->now() - map_refresh_request_time_).seconds(); // 已等待地图刷新的时长。
      if (!map_refresh_timeout_warned_ && elapsed >= map_refresh_warning_timeout_)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "[MapRefresh] Still waiting for fresh observations after %.2fs "
                    "(current=%llu target=%llu); planning remains deferred",
                    elapsed, static_cast<unsigned long long>(current_sequence),
                    static_cast<unsigned long long>(required_observation_sequence_));
        map_refresh_timeout_warned_ = true;
      }
      return true; // 返回 true 表示本次 FSM 主循环应暂停，继续等地图。
    }

    auto path = pending_reference_path_; // 取出延迟的路径消息。
    pending_reference_path_.reset();     // 清空挂起路径，避免重复处理。
    RCLCPP_INFO(node_->get_logger(),
                "[MapRefresh] Fresh map ready at observation_sequence=%llu; "
                "accepting deferred reference path",
                static_cast<unsigned long long>(current_sequence));
    processReferencePath(path); // 地图刷新完成后正式规划参考路径。
    return false;               // 本次已处理完挂起任务，FSM 可继续后续逻辑。
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x; // 更新当前位置 x。
    odom_pos_(1) = msg->pose.pose.position.y; // 更新当前位置 y。
    odom_pos_(2) = msg->pose.pose.position.z; // 更新当前位置 z。

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_) // 首帧里程计用于确定 RViz 目标高度。
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x; // 更新线速度 x。
    odom_vel_(1) = msg->twist.twist.linear.y; // 更新线速度 y。
    odom_vel_(2) = msg->twist.twist.linear.z; // 更新线速度 z。

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;              // 标记里程计已经可用。
    publishSelfInflationMarker();   // 发布自身膨胀体可视化，便于检查碰撞模型。
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_) // 预设航点模式等首帧里程计后自动启动。
    {
      preset_started_ = true;       // 避免每帧 odom 都重复启动预设航点任务。
      planGlobalTrajbyGivenWps();   // 根据参数中的航点开始规划。
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data; // 接收执行端冻结状态，用于暂停局部轨迹时间推进。
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now(); // 当前 ROS 时间。
    double dt = (now - last_freeze_update_time_).seconds(); // 距离上次冻结补偿的时间差。
    last_freeze_update_time_ = now; // 更新补偿时间戳。

    if (dt <= 0.0 || dt > 0.2) // 时间跳变或间隔过大时不补偿，避免轨迹时间被异常拉动。
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt); // 执行冻结时平移轨迹起始时间，使 t_cur 保持不变。
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0); // 取机体系 x 轴在世界系中的朝向。
    if (heading.head<2>().squaredNorm() < 1e-8) // 水平投影过小时无法可靠估计 yaw。
      return 0.0;
    return std::atan2(heading(1), heading(0)); // 根据水平朝向计算偏航角。
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1)); // 轨迹段在水平面的方向向量。
    if (diff.squaredNorm() < 1e-8) // 两点太近时退回当前里程计 yaw。
      return getOdomYaw();
    return std::atan2(diff(1), diff(0)); // 用轨迹段方向估计机器人朝向。
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_); // 可视化圆柱半径，负值保护为 0。
    const double z_up = std::max(0.0, self_inflation_z_up_);           // 机体上方膨胀高度。
    const double z_down = std::max(0.0, self_inflation_z_down_);       // 机体下方膨胀高度。
    const double height = std::max(1e-3, z_up + z_down);               // 圆柱高度至少给一个极小正值，避免 marker 不显示。

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_; // marker 坐标系。
    marker.header.stamp = node_->now();                         // 使用当前时间戳。
    marker.ns = "self_inflation";                               // marker 命名空间，便于 RViz 分组。
    marker.type = visualization_msgs::msg::Marker::CYLINDER;     // 用圆柱表示自身膨胀体。
    marker.action = visualization_msgs::msg::Marker::ADD;        // 添加或更新 marker。
    marker.pose.orientation.w = 1.0;                             // 单位四元数，无额外旋转。
    marker.scale.x = 2.0 * radius;                               // 圆柱直径 x。
    marker.scale.y = 2.0 * radius;                               // 圆柱直径 y。
    marker.scale.z = height;                                     // 圆柱高度。
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;     // 以当前里程计位置作为双圆柱模型中心。
    center(2) += 0.5 * (z_up - z_down);     // 根据上下膨胀高度调整圆柱中心高度。

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0); // 当前水平朝向单位向量。
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;      // 前圆柱中心。
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;       // 后圆柱中心。

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_) // 连续请求同一状态时累计次数，用于决定是否随机初始化。
      continuously_called_times_++;
    else
      continuously_called_times_ = 1; // 状态发生变化时重新计数。

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_); // 保存切换前状态用于日志。
    exec_state_ = new_state;      // 真正更新 FSM 执行状态。
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    if (processPendingReferencePath()) // 若正在等待清图后的新观测，本轮 FSM 暂停。
      return;

    updateLocalTrajTimeFreeze(); // 根据 Go2 冻结状态补偿局部轨迹时间。

    static int fsm_num = 0;
    fsm_num++; // 计数用于降低状态打印频率。
    if (fsm_num == 100) // 每 100 次主循环打印一次状态和等待原因。
    {
      printFSMExecState();
      if (!have_odom_) // 尚未收到里程计。
        cout << "no odom." << endl;
      if (!trigger_) // 尚未收到目标或参考路径。
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_) // 初始化阶段必须先等里程计，避免起点状态无效。
      {
        return;
      }
      if (!trigger_) // 还没有目标输入时继续停留在 INIT。
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM"); // 基础输入齐备后进入等待目标可用状态。
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_) // 已触发但全局目标还未成功生成时继续等待。
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 目标已准备好，开始生成第一段局部轨迹。
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj(); // 为新局部轨迹设置起点位置、速度和加速度。

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init; // 是否采用随机多项式初始化，连续失败时打开以跳出局部最优。
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false; // 首次尝试使用确定性初始化。
      else
        flag_random_poly_init = true;  // 重复进入该状态说明失败过，改用随机初始化。

      bool success = callReboundReplan(true, flag_random_poly_init); // 调用局部 rebound 优化器生成轨迹。
      if (success)
      {

        replan_fail_count_ = 0;                 // 成功后清零连续失败计数。
        changeFSMExecState(EXEC_TRAJ, "FSM");   // 切换到轨迹执行状态。
        flag_escape_emergency_ = true;          // 允许之后再次进入急停时发布急停轨迹。
      }
      else
      {
        replan_fail_count_++;                   // 记录本次生成失败。
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 留在生成状态，下一轮可能随机初始化。
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj()) // 从当前执行轨迹/里程计状态重新规划。
      {
        replan_fail_count_ = 0;               // 重规划成功后清零失败计数。
        changeFSMExecState(EXEC_TRAJ, "FSM"); // 回到执行状态。
      }
      else
      {
        replan_fail_count_++;                 // 记录连续重规划失败。
        changeFSMExecState(REPLAN_TRAJ, "FSM"); // 继续尝试重规划。
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_; // 当前正在执行的局部轨迹数据。
      rclcpp::Time time_now = node_->now();                 // 当前 ROS 时间。
      double t_cur = (time_now - info->start_time_).seconds(); // 当前轨迹已经执行的时间。
      t_cur = min(info->duration_, t_cur);                  // 防止超过轨迹总时长。

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur); // 当前轨迹时间对应的位置。

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5) // 预设航点模式下，接近当前航点则切到下一个航点。
      {
        current_wp_++; // 推进到下一个航点。
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 新航点全局轨迹已生成，下一步生成局部轨迹。
          return;
        }
        replan_fail_count_++; // 下一个航点规划失败，计入失败次数。
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) // 局部轨迹执行到末端。
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++; // 当前段执行完毕，切换到下一个预设航点。
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        // A local trajectory only covers a finite planning horizon.  If
        // replanning failed near the end of that horizon, reaching its
        // terminal point does not mean that the global/reference-path target
        // has been reached.  Keep the target alive and start a fresh plan
        // from odometry; otherwise Mode 3 incorrectly falls into WAIT_TARGET
        // halfway through a long route and the controller commands zero.
        const double remaining_distance = (end_pt_ - odom_pos_).norm(); // 当前里程计到全局终点的剩余距离。
        if (remaining_distance >= no_replan_thresh_) // 局部轨迹结束但离全局终点还远，需要继续规划。
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 2000,
              "Local trajectory ended %.2f m before the global target; replanning from odometry",
              remaining_distance);
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode()) // 航点序列已经全部完成，清理队列状态。
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false; // 任务完成，回到等待新目标。

        changeFSMExecState(WAIT_TARGET, "FSM"); // 等待下一次目标输入。
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_) // 局部轨迹已经接近目标，不再主动重规划。
      {
        // cout << "near end" << endl;
        return;
      }
      else if (replan_period_ > 0.0 && t_cur < replan_period_) // 周期重规划模式下，未到周期先继续执行。
      {
        // A positive period selects deterministic time-based replanning.
        // This is independent of robot speed, unlike thresh_replan.
        return;
      }
      else if (replan_period_ <= 0.0 &&
               (info->start_pos_ - pos).norm() < replan_thresh_) // 距离触发模式下，尚未走够阈值先继续执行。
      {
        // Backward-compatible distance-triggered replanning.
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM"); // 满足重规划触发条件，切到重规划状态。
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // 首次进入急停状态时发布一次急停轨迹，避免重复调用。
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1) // 普通急停后速度足够小，尝试恢复规划。
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1) // 连续失败触发的悬停急停，等待新目标。
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false; // 清除悬停等待标志。
          have_target_ = false;     // 丢弃旧目标。
          trigger_ = false;         // 需要新的触发信号。
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false; // 本轮急停轨迹已经发布过，后续不重复发布。
      break;
    }
    }

    finishProcess(); // 检查连续失败次数是否超过上限。

    data_disp_.header.stamp = node_->now(); // 更新调试显示消息时间戳。
    data_disp_pub_->publish(data_disp_);    // 发布规划状态数据，供可视化/调试使用。
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_) // 连续失败超过阈值，认为当前目标不可安全继续。
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;      // 清零计数，避免反复触发。
      need_hover_stop_ = true;     // 急停后不自动追旧目标，而是等待新目标。
      flag_escape_emergency_ = true; // 允许 EMERGENCY_STOP 状态发布急停轨迹。
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_; // 当前局部轨迹。
    rclcpp::Time time_now = node_->now();                 // 当前 ROS 时间。
    double t_cur = (time_now - info->start_time_).seconds(); // 轨迹执行时间。
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_); // 限制到有效轨迹时间范围内。

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH) // 参考路径模式尽量沿当前局部轨迹连续重规划，减少跳变。
    {
      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);     // 从当前局部轨迹位置接续。
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);    // 保持当前轨迹速度连续。
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur); // 保持当前轨迹加速度连续。

      bool success = callReboundReplan(false, false); // 第一次尝试不重新生成多项式初值。
      if (!success)
      {
        success = callReboundReplan(true, false); // 失败后使用多项式初值再试。
        if (!success)
        {
          success = callReboundReplan(true, true); // 再失败则使用随机多项式初值提高脱困概率。
          if (!success)
            return false;
        }
      }

      return true;
    }

    start_pt_ = odom_pos_;                                      // 非参考路径模式从真实里程计位置重新接入。
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);    // 速度仍参考当前局部轨迹，保证平滑。
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur); // 加速度也从当前轨迹估计。

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>(); // 当前位置指向目标的水平向量。
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero(); // 若当前速度背离目标，置零避免用错误速度继续规划。
      start_acc_.setZero(); // 同时清零加速度，降低初始状态冲突。
    }

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

    if (!adjustGlobalTargetIfOccupied()) // 刷新全局轨迹后再次确认目标未被占据。
      return false;

    bool success = callReboundReplan(true, false); // 使用多项式初值生成新的局部轨迹。
    if (!success)
    {
      success = callReboundReplan(true, true); // 失败后开启随机初值再试一次。
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_; // 默认从当前里程计位置开始。
    start_vel_ = odom_vel_; // 默认使用当前里程计速度。
    start_acc_.setZero();   // 里程计通常没有可靠加速度，这里置零。

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5) // 没有有效局部轨迹时只能用里程计。
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds(); // 当前时间落在局部轨迹上的原始参数。
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2) // 时间偏出太多时说明旧轨迹不可用于接续。
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_); // 限制到轨迹有效区间。
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);                // 用当前轨迹速度保证动态连续。
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);            // 用当前轨迹加速度保证动态连续。

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero(); // 速度方向背离目标时置零，避免规划器继续往反方向走。
      start_acc_.setZero(); // 同步清零加速度。
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze(); // 碰撞检查也要考虑执行冻结，确保检查的轨迹时间和控制端一致。

    LocalTrajData *info = &planner_manager_->local_data_; // 当前局部轨迹。
    auto map = planner_manager_->grid_map_;               // 地图用于占据查询。

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5) // 没目标或没有有效轨迹时无需检查。
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01; // 沿局部轨迹做前向碰撞采样的时间步长。
    double t_cur = (node_->now() - info->start_time_).seconds(); // 当前执行到的轨迹时间。
    double t_2_3 = info->duration_ * 2 / 3; // 只检查前 2/3 轨迹，给后段保留重规划空间。
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t); // 当前采样点位置。
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_)); // 下一采样点用于估计朝向。
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // 发现碰撞后先尝试立即重规划一次。
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY"); // 重规划成功，继续执行新轨迹。
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 障碍过近，来不及常规重规划，进入急停。
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY"); // 障碍较远，切换到普通重规划状态。
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget(); // 沿全局轨迹选取本次局部优化的目标点和目标速度。

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj); // 调用前端搜索和后端 B 样条优化。
    have_new_target_ = false; // 本轮规划已经消费新目标标志。

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_; // 取出优化后的局部轨迹数据。

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;                    // 三阶 B 样条。
      bspline.start_time = info->start_time_; // 控制端按该时间戳计算轨迹相对时间。
      bspline.traj_id = info->traj_id_;       // 轨迹编号用于区分新旧轨迹。

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint(); // 位置 B 样条控制点矩阵。
      bspline.pos_pts.reserve(pos_pts.cols());                          // 预分配消息数组容量。
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt; // ROS 消息中的三维点。
        pt.x = pos_pts(0, i);         // 控制点 x。
        pt.y = pos_pts(1, i);         // 控制点 y。
        pt.z = pos_pts(2, i);         // 控制点 z。
        bspline.pos_pts.push_back(pt); // 加入待发布的控制点列表。
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot(); // B 样条节点向量。
      bspline.knots.reserve(knots.rows());                    // 预分配节点数组容量。
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i)); // 复制每个 knot 到 ROS 消息。
      }

      bspline_pub_->publish(bspline); // 发布给控制器执行。

      visualization_->displayOptimalTraj(info->position_traj_, 0); // 在 RViz 显示优化后的局部轨迹。
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos); // 在当前位置生成一条急停 B 样条轨迹。

    auto info = &planner_manager_->local_data_; // 急停轨迹也写入 local_data_，复用发布流程。

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;                      // 三阶 B 样条。
    bspline.start_time = info->start_time_; // 急停轨迹开始时间。
    bspline.traj_id = info->traj_id_;       // 急停轨迹编号。

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint(); // 急停轨迹控制点。
    bspline.pos_pts.reserve(pos_pts.cols());                          // 预分配控制点容量。
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt; // ROS 三维点消息。
      pt.x = pos_pts(0, i);         // 控制点 x。
      pt.y = pos_pts(1, i);         // 控制点 y。
      pt.z = pos_pts(2, i);         // 控制点 z。
      bspline.pos_pts.push_back(pt); // 写入急停轨迹消息。
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot(); // 急停轨迹节点向量。
    bspline.knots.reserve(knots.rows());                    // 预分配节点容量。
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i)); // 复制 knot。
    }

    bspline_pub_->publish(bspline); // 发布急停轨迹给控制端。

    return true;
  }

  void SCANReplanFSM::getLocalTarget()
  {
    const double max_vel = planner_manager_->pp_.max_vel_; // 规划参数中的最大速度。
    const double max_acc = planner_manager_->pp_.max_acc_; // 规划参数中的最大加速度。
    const double duration = planner_manager_->global_data_.global_duration_; // 全局轨迹总时长。
    double t_step = max_vel > 1e-6 ? planning_horizon_ / 20.0 / max_vel : 0.01; // 根据规划视距和速度估计全局轨迹采样步长。
    t_step = std::max(t_step, 0.01); // 采样步长不小于 0.01s，避免循环过密。

    double t_proj = 0.0;               // start_pt_ 投影到全局轨迹上的时间。
    double min_dist_to_start = 9999.0; // 当前找到的最近距离。
    for (double t = 0.0; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t); // 全局轨迹在 t 时刻的位置。
      double dist_to_start = (pos_t - start_pt_).norm();                     // 该点到局部规划起点的距离。
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start; // 更新最近距离。
        t_proj = t;                        // 记录最近点时间，作为局部视距搜索起点。
      }
    }

    double target_t = duration; // 默认局部目标时间为全局终点。
    double total_dist = 0.0;    // 从投影点开始沿全局轨迹累计的弧长。
    bool target_found = false;  // 是否找到了满足 planning_horizon_ 的局部目标。
    Eigen::Vector3d prev_pos = planner_manager_->global_data_.getPosition(t_proj); // 弧长累计的上一采样点。
    local_target_pt_ = end_pt_; // 若全局剩余长度小于视距，则局部目标直接取终点。

    for (double t = t_proj; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t); // 当前采样点。
      total_dist += (pos_t - prev_pos).norm();                               // 累计沿轨迹走过的距离。
      if (total_dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t; // 找到视距边界上的局部目标。
        target_t = t;             // 记录局部目标对应的全局轨迹时间。
        target_found = true;      // 标记已经找到目标点。
        break;
      }
      prev_pos = pos_t; // 推进弧长累计的上一点。
    }
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration; // 记录全局轨迹推进进度。

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) { // 查询候选局部目标是否落在膨胀障碍中。
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0) // 局部目标被占据时，在目标时间附近寻找可通行替代点。
    {
      bool found_free_target = false; // 是否找到了可通行替代目标。
      double adjusted_t = target_t;   // 替代目标对应的全局轨迹时间。

      for (double dt = 0.0; dt <= duration; dt += t_step)
      {
        double t_forward = target_t + dt; // 优先向前搜索更接近任务终点的可行点。
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

        double t_backward = target_t - dt; // 同时向后搜索，避免前方都被占据时无目标可用。
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
        target_t = adjusted_t; // 用替代点时间更新局部目标速度查询位置。
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
    }

    const double braking_distance = max_acc > 1e-6 // 按 v^2/(2a) 估算从最大速度刹停所需距离。
        ? (max_vel * max_vel) / (2.0 * max_acc)
        : std::numeric_limits<double>::infinity();
    if ((end_pt_ - local_target_pt_).norm() < braking_distance) // 局部目标已经接近终点，目标速度设为 0 以便减速停车。
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero(); // 终点附近要求停下。
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t); // 否则沿用全局轨迹在局部目标处的速度。
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel; // 防止全局速度超过局部规划速度上限。
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
