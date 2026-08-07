#include <memory>     // 引入智能指针支持，用于 std::make_shared 创建 ROS2 节点对象
#include <exception>  // 引入标准异常类型，用于捕获初始化或运行过程中的异常

#include <rclcpp/rclcpp.hpp>             // 引入 ROS2 C++ 客户端库的核心接口
#include <plan_manage/scan_replan_fsm.h> // 引入 SCAN-Planner 的重规划有限状态机定义

int main(int argc, char **argv) // 程序入口，argc/argv 用于接收 ROS2 启动参数和命令行参数
{
  rclcpp::init(argc, argv); // 初始化 ROS2 运行时，使当前进程可以创建节点、通信和使用执行器
  auto node = std::make_shared<rclcpp::Node>("scan_planner_node"); // 创建名为 scan_planner_node 的 ROS2 节点

  try // 捕获规划器初始化或执行器运行中可能抛出的异常，避免程序无提示崩溃
  {
    scan_planner::SCANReplanFSM planner; // 创建 SCAN-Planner 重规划有限状态机对象，负责规划流程调度
    planner.init(node.get()); // 使用当前 ROS2 节点初始化规划器，内部通常会读取参数、创建订阅/发布和定时器
    rclcpp::executors::SingleThreadedExecutor executor; // 创建单线程执行器，按顺序处理该节点的回调函数
    executor.add_node(node); // 将 scan_planner_node 加入执行器，使其订阅、定时器等回调可以被调度
    executor.spin(); // 进入 ROS2 回调循环，持续处理消息、定时器和服务回调，直到节点关闭
  }
  catch (const std::exception &error) // 捕获标准异常并输出致命日志，便于定位初始化或运行失败原因
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize SCAN-Planner: %s", error.what()); // 打印致命错误日志和异常信息
    rclcpp::shutdown(); // 异常发生时主动关闭 ROS2 运行时，释放相关资源
    return 1; // 返回非零退出码，表示程序异常结束
  }

  rclcpp::shutdown(); // 正常退出回调循环后关闭 ROS2 运行时，清理节点和通信资源
  return 0; // 返回 0 表示程序正常结束
}
