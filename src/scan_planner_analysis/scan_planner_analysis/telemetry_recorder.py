"""ROS 2 node that records SCAN-Planner velocity telemetry as JSON Lines."""

from datetime import datetime
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .log_format import encode_record


def resolve_output_path(output_file, output_directory, stamp=None):
    """Resolve an explicit file or create a timestamped file in a directory."""
    if output_file:
        return os.path.abspath(os.path.expanduser(output_file))
    if not output_directory:
        output_directory = "~/.ros/scan_planner_analysis"
    if stamp is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = os.path.abspath(os.path.expanduser(output_directory))
    return os.path.join(directory, f"velocity_{stamp}.jsonl")


class TelemetryRecorder(Node):
    """Record commands and measured odometry without affecting control."""

    def __init__(self):
        super().__init__("scan_planner_telemetry_recorder")
        self.declare_parameter("output_file", "")
        self.declare_parameter("output_directory", "")
        self.declare_parameter("safe_cmd_topic", "/cmd_vel_smoothed")
        self.declare_parameter("raw_cmd_topic", "/scan_planner/cmd_vel_raw")
        self.declare_parameter("odom_topic", "/scan_planner/body_pose")
        self.declare_parameter("flush_interval", 1.0)

        output_file = self.get_parameter("output_file").value
        output_directory = self.get_parameter("output_directory").value
        self.output_file = resolve_output_path(output_file, output_directory)
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        self._stream = open(self.output_file, "a", encoding="utf-8")

        safe_topic = self.get_parameter("safe_cmd_topic").value
        raw_topic = self.get_parameter("raw_cmd_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.create_subscription(
            Twist, safe_topic,
            lambda message: self._record_twist("cmd_vel_safe", safe_topic, message),
            100)
        self.create_subscription(
            Twist, raw_topic,
            lambda message: self._record_twist("cmd_vel_raw", raw_topic, message),
            100)
        self.create_subscription(
            Odometry, odom_topic,
            lambda message: self._record_odom(odom_topic, message),
            qos_profile_sensor_data)

        flush_interval = max(
            0.1, float(self.get_parameter("flush_interval").value))
        self.create_timer(flush_interval, self._stream.flush)
        self.get_logger().info(
            f"Recording SCAN-Planner telemetry to {self.output_file}")

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _write(self, record_type, topic, **values):
        self._stream.write(
            encode_record(record_type, self._now_ns(), topic, **values) + "\n")

    def _record_twist(self, record_type, topic, message):
        self._write(
            record_type, topic,
            vx=message.linear.x, vy=message.linear.y, vz=message.linear.z,
            wx=message.angular.x, wy=message.angular.y, wz=message.angular.z)

    def _record_odom(self, topic, message):
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000 +
            int(message.header.stamp.nanosec))
        twist = message.twist.twist
        self._write(
            "odom_velocity", topic, message_timestamp_ns=stamp_ns,
            vx=twist.linear.x, vy=twist.linear.y, vz=twist.linear.z,
            wx=twist.angular.x, wy=twist.angular.y, wz=twist.angular.z)

    def destroy_node(self):
        if hasattr(self, "_stream") and not self._stream.closed:
            self._stream.flush()
            self._stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
