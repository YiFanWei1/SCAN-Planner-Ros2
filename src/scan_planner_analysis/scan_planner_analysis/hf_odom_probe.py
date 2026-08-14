#!/usr/bin/env python3

import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class HighFrequencyOdomProbe(Node):
    """Measure source timestamp cadence independently from DDS callback cadence."""

    def __init__(self):
        super().__init__("hf_odom_probe")
        self.topic = self.declare_parameter("topic", "/lio_odom_hf").value
        self.report_period = float(self.declare_parameter("report_period", 1.0).value)
        self.gap_threshold = float(self.declare_parameter("gap_threshold", 0.10).value)
        self.header_gap_threshold = float(
            self.declare_parameter("header_gap_threshold", 0.03).value
        )
        self.stale_threshold = float(self.declare_parameter("stale_threshold", 0.10).value)
        self.output_file = str(self.declare_parameter("output_file", "").value)
        reliability = str(
            self.declare_parameter("reliability", "best_effort").value
        ).lower()
        depth = int(self.declare_parameter("qos_depth", 1).value)
        if (self.report_period <= 0.0 or self.gap_threshold <= 0.0 or
                self.header_gap_threshold <= 0.0 or depth <= 0):
            raise ValueError(
                "report_period, gap thresholds and qos_depth must be positive"
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if reliability == "reliable"
                else ReliabilityPolicy.BEST_EFFORT
            ),
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Odometry, self.topic, self.odom_callback, qos
        )
        self.timer = self.create_timer(self.report_period, self.report)
        self.stall_watchdog = self.create_timer(
            min(0.02, self.gap_threshold / 5.0), self.check_callback_stall
        )

        self.started_monotonic = time.monotonic()
        self.last_report_monotonic = self.started_monotonic
        self.last_arrival_monotonic = None
        self.last_header_stamp = None
        self.total_messages = 0
        self.interval_messages = 0
        self.total_gaps = 0
        self.interval_gaps = 0
        self.confirmed_callback_stalls = 0
        self.source_time_gaps = 0
        self.stall_alert_active = False
        self.non_monotonic_stamps = 0
        self.max_receive_gap = 0.0
        self.max_header_gap = 0.0
        self.max_message_age = -math.inf
        self.min_message_age = math.inf
        self.output_stream = None
        if self.output_file:
            path = Path(self.output_file).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.output_stream = path.open("a", encoding="utf-8", buffering=1)

        self.get_logger().info(
            "Independent odometry probe: topic=%s reliability=%s depth=%d "
            "gap_threshold=%.3fs header_gap_threshold=%.3fs output=%s"
            % (self.topic, reliability, depth, self.gap_threshold,
               self.header_gap_threshold,
               self.output_file or "console-only")
        )

    @staticmethod
    def stamp_seconds(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def write_event(self, event):
        if self.output_stream is not None:
            self.output_stream.write(json.dumps(event, ensure_ascii=False) + "\n")

    def odom_callback(self, msg):
        arrival_monotonic = time.monotonic()
        arrival_ros = self.get_clock().now().nanoseconds * 1e-9
        header_stamp = self.stamp_seconds(msg)
        message_age = arrival_ros - header_stamp
        receive_gap = None
        header_gap = None
        if self.last_arrival_monotonic is not None:
            receive_gap = arrival_monotonic - self.last_arrival_monotonic
            header_gap = header_stamp - self.last_header_stamp
            self.max_receive_gap = max(self.max_receive_gap, receive_gap)
            self.max_header_gap = max(self.max_header_gap, header_gap)
            if header_gap <= 0.0:
                self.non_monotonic_stamps += 1
            if receive_gap >= self.gap_threshold:
                self.total_gaps += 1
                self.interval_gaps += 1
                event = {
                    "event": "receive_gap",
                    "arrival_ros": arrival_ros,
                    "receive_gap": receive_gap,
                    "header_gap": header_gap,
                    "message_age": message_age,
                    "header_stamp": header_stamp,
                    "source_cadence_ok": (
                        0.0 < header_gap <= self.header_gap_threshold
                    ),
                }
                self.write_event(event)
                if event["source_cadence_ok"]:
                    self.confirmed_callback_stalls += 1
                    self.get_logger().error(
                        "[CALLBACK_STALL_CONFIRMED] callback resumed: "
                        "receive_gap=%.6fs but header_gap=%.6fs remains normal; "
                        "message_age=%.6fs"
                        % (receive_gap, header_gap, message_age)
                    )
                else:
                    self.source_time_gaps += 1
                    self.get_logger().warn(
                        "[SOURCE_TIME_GAP] receive_gap=%.6fs header_gap=%.6fs "
                        "message_age=%.6fs"
                        % (receive_gap, header_gap, message_age)
                    )

        self.total_messages += 1
        self.interval_messages += 1
        self.max_message_age = max(self.max_message_age, message_age)
        self.min_message_age = min(self.min_message_age, message_age)
        self.last_arrival_monotonic = arrival_monotonic
        self.last_header_stamp = header_stamp
        self.stall_alert_active = False

        if receive_gap is not None and (
            receive_gap >= self.gap_threshold or message_age >= self.stale_threshold
        ):
            self.write_event({
                "event": "sample",
                "arrival_ros": arrival_ros,
                "header_stamp": header_stamp,
                "receive_gap": receive_gap,
                "header_gap": header_gap,
                "message_age": message_age,
            })

    def check_callback_stall(self):
        """Raise the alert while the callback is stalled, not only after recovery."""
        if self.last_arrival_monotonic is None or self.stall_alert_active:
            return
        silence = time.monotonic() - self.last_arrival_monotonic
        if silence < self.gap_threshold:
            return
        self.stall_alert_active = True
        event = {
            "event": "callback_stall_begin",
            "arrival_ros": self.get_clock().now().nanoseconds * 1e-9,
            "silence": silence,
            "last_header_stamp": self.last_header_stamp,
        }
        self.write_event(event)
        self.get_logger().error(
            "[CALLBACK_STALL_BEGIN] no %s callback for %.6fs; "
            "last_header_stamp=%.9f"
            % (self.topic, silence, self.last_header_stamp)
        )

    def report(self):
        now_monotonic = time.monotonic()
        interval = max(1e-9, now_monotonic - self.last_report_monotonic)
        elapsed = max(1e-9, now_monotonic - self.started_monotonic)
        interval_hz = self.interval_messages / interval
        total_hz = self.total_messages / elapsed
        age_max = self.max_message_age if self.total_messages else math.nan
        age_min = self.min_message_age if self.total_messages else math.nan
        summary = {
            "event": "summary",
            "arrival_ros": self.get_clock().now().nanoseconds * 1e-9,
            "elapsed": elapsed,
            "messages": self.total_messages,
            "interval_hz": interval_hz,
            "average_hz": total_hz,
            "interval_gaps": self.interval_gaps,
            "total_gaps": self.total_gaps,
            "max_receive_gap": self.max_receive_gap,
            "max_header_gap": self.max_header_gap,
            "min_message_age": age_min,
            "max_message_age": age_max,
            "non_monotonic_stamps": self.non_monotonic_stamps,
            "confirmed_callback_stalls": self.confirmed_callback_stalls,
            "source_time_gaps": self.source_time_gaps,
        }
        self.write_event(summary)
        self.get_logger().info(
            "[HF_ODOM_SUMMARY] interval_hz=%.1f average_hz=%.1f "
            "gaps=%d/%d max_receive_gap=%.6fs max_header_gap=%.6fs "
            "message_age=[%.6f, %.6f]s confirmed_stalls=%d "
            "source_gaps=%d non_monotonic=%d"
            % (interval_hz, total_hz, self.interval_gaps, self.total_gaps,
               self.max_receive_gap, self.max_header_gap, age_min, age_max,
               self.confirmed_callback_stalls, self.source_time_gaps,
               self.non_monotonic_stamps)
        )
        self.interval_messages = 0
        self.interval_gaps = 0
        self.last_report_monotonic = now_monotonic

    def destroy_node(self):
        self.report()
        if self.output_stream is not None:
            self.output_stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HighFrequencyOdomProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
