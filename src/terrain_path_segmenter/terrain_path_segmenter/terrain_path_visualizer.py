#!/usr/bin/env python3
"""ROS node that visualizes terrain-linear sections of an input path."""

import colorsys
import copy
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from terrain_path_segmenter.segmentation import segment_path


def color_for_index(index, alpha=1.0):
    """Generate stable, visually distinct colors."""
    red, green, blue = colorsys.hsv_to_rgb((index * 0.61803398875) % 1.0,
                                           0.78, 1.0)
    return ColorRGBA(r=red, g=green, b=blue, a=alpha)


class TerrainPathVisualizer(Node):
    """Segment incoming global paths and expose read-only visualization."""

    def __init__(self):
        super().__init__("terrain_path_visualizer")
        self.declare_parameter("max_linear_z_error", 0.04)
        self.declare_parameter("slope_merge_threshold", 0.04)
        self.declare_parameter("minimum_segment_length", 0.5)
        self.declare_parameter("segment_reached_tolerance", 0.25)
        self.declare_parameter("visual_z_offset", 0.05)
        self.declare_parameter("line_width", 0.07)

        self.max_error = self.get_parameter("max_linear_z_error").value
        self.slope_threshold = self.get_parameter(
            "slope_merge_threshold").value
        self.minimum_length = self.get_parameter(
            "minimum_segment_length").value
        self.reached_tolerance = self.get_parameter(
            "segment_reached_tolerance").value
        self.z_offset = self.get_parameter("visual_z_offset").value
        self.line_width = self.get_parameter("line_width").value

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        input_qos = QoSProfile(depth=10)
        input_qos.reliability = ReliabilityPolicy.RELIABLE
        input_qos.durability = DurabilityPolicy.VOLATILE
        self.marker_pub = self.create_publisher(
            MarkerArray, "segments", transient_qos)
        self.processed_path_pub = self.create_publisher(
            Path, "processed_path", transient_qos)
        self.current_path_pub = self.create_publisher(
            Path, "current_path", transient_qos)
        self.current_goal_pub = self.create_publisher(
            PoseStamped, "current_goal", transient_qos)
        self.path_sub = self.create_subscription(
            Path, "global_path", self.path_callback, input_qos)
        self.odom_sub = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, 20)

        self.path = None
        self.ranges = []
        self.current_index = 0
        self.last_odom = None
        self.get_logger().info(
            "Waiting for a global path; visualization only, no command output")

    def path_callback(self, message):
        if len(message.poses) < 2:
            self.get_logger().warning("Ignoring path with fewer than two poses")
            return
        points = [(pose.pose.position.x, pose.pose.position.y,
                   pose.pose.position.z) for pose in message.poses]
        try:
            ranges = segment_path(
                points, self.max_error, self.slope_threshold,
                self.minimum_length)
        except ValueError as error:
            self.get_logger().error(f"Cannot segment path: {error}")
            return
        self.path = copy.deepcopy(message)
        self.ranges = ranges
        self.current_index = 0
        self.processed_path_pub.publish(self.path)
        self.publish_visualization()
        self.get_logger().info(
            f"Segmented {len(points)} path points into {len(ranges)} sections")

    def odom_callback(self, message):
        self.last_odom = message
        if self.path is None or not self.ranges:
            return
        while self.current_index + 1 < len(self.ranges):
            goal_index = self.ranges[self.current_index][1]
            goal = self.path.poses[goal_index].pose.position
            position = message.pose.pose.position
            # The global path may represent terrain height while odometry is
            # reported at body height, so section handoff is evaluated in XY.
            distance = math.hypot(position.x - goal.x,
                                  position.y - goal.y)
            if distance > self.reached_tolerance:
                break
            self.current_index += 1
            self.get_logger().info(
                f"Visualization advanced to segment {self.current_index + 1}/"
                f"{len(self.ranges)}")
            self.publish_visualization()

    def make_path(self, start, end):
        result = Path()
        result.header = self.path.header
        result.header.stamp = self.get_clock().now().to_msg()
        result.poses = copy.deepcopy(self.path.poses[start:end + 1])
        for pose in result.poses:
            pose.header = result.header
        return result

    def publish_visualization(self):
        if self.path is None or not self.ranges:
            return
        now = self.get_clock().now().to_msg()
        frame = self.path.header.frame_id or "world"
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for segment_index, (start, end) in enumerate(self.ranges):
            line = Marker()
            line.header.frame_id = frame
            line.header.stamp = now
            line.ns = "segmented_paths"
            line.id = segment_index
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = (self.line_width * 1.8 if
                            segment_index == self.current_index else
                            self.line_width)
            line.color = color_for_index(segment_index)
            for pose in self.path.poses[start:end + 1]:
                point = Point()
                point.x = pose.pose.position.x
                point.y = pose.pose.position.y
                point.z = pose.pose.position.z + self.z_offset
                line.points.append(point)
            markers.markers.append(line)

            goal_pose = self.path.poses[end].pose.position
            target = Marker()
            target.header = line.header
            target.ns = "segment_targets"
            target.id = segment_index
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose.position = copy.deepcopy(goal_pose)
            target.pose.position.z += self.z_offset
            target.pose.orientation.w = 1.0
            target.scale.x = target.scale.y = target.scale.z = 0.22
            target.color = color_for_index(segment_index)
            markers.markers.append(target)

            label = Marker()
            label.header = line.header
            label.ns = "target_labels"
            label.id = segment_index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = copy.deepcopy(target.pose.position)
            label.pose.position.z += 0.28
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f"G{segment_index + 1}"
            markers.markers.append(label)

        current_start, current_end = self.ranges[self.current_index]
        current_goal_position = self.path.poses[current_end].pose.position
        active = Marker()
        active.header.frame_id = frame
        active.header.stamp = now
        active.ns = "current_target"
        active.id = 0
        active.type = Marker.SPHERE
        active.action = Marker.ADD
        active.pose.position = copy.deepcopy(current_goal_position)
        active.pose.position.z += self.z_offset
        active.pose.orientation.w = 1.0
        active.scale.x = active.scale.y = active.scale.z = 0.38
        active.color = ColorRGBA(r=1.0, g=0.95, b=0.0, a=1.0)
        markers.markers.append(active)

        self.marker_pub.publish(markers)
        self.current_path_pub.publish(self.make_path(current_start, current_end))
        goal = copy.deepcopy(self.path.poses[current_end])
        goal.header.frame_id = frame
        goal.header.stamp = now
        self.current_goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainPathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
