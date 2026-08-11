#!/usr/bin/env python3
"""ROS node that visualizes terrain-linear sections of an input path."""

import colorsys
import copy

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from terrain_path_segmenter.segmentation import (
    active_segment_for_progress, cumulative_xy_distance,
    project_onto_path, segment_handoff_reason, segment_path)


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
        self.declare_parameter("accept_first_path_only", False)
        self.declare_parameter("visual_z_offset", 0.05)
        self.declare_parameter("line_width", 0.07)
        self.declare_parameter("progress_projection_z_weight", 2.0)
        self.declare_parameter("progress_backtrack_tolerance", 0.30)
        self.declare_parameter("progress_max_forward_distance", 4.0)
        self.declare_parameter("progress_floor_tolerance", 0.75)
        self.declare_parameter("progress_pass_margin", 0.02)
        self.declare_parameter("progress_pass_max_xy_distance", 1.0)
        self.declare_parameter("initial_projection_horizon", 3.0)

        self.max_error = self.get_parameter("max_linear_z_error").value
        self.slope_threshold = self.get_parameter(
            "slope_merge_threshold").value
        self.minimum_length = self.get_parameter(
            "minimum_segment_length").value
        self.reached_tolerance = self.get_parameter(
            "segment_reached_tolerance").value
        self.accept_first_path_only = self.get_parameter(
            "accept_first_path_only").value
        self.z_offset = self.get_parameter("visual_z_offset").value
        self.line_width = self.get_parameter("line_width").value
        self.projection_z_weight = self.get_parameter(
            "progress_projection_z_weight").value
        self.progress_backtrack_tolerance = self.get_parameter(
            "progress_backtrack_tolerance").value
        self.progress_max_forward_distance = self.get_parameter(
            "progress_max_forward_distance").value
        self.progress_floor_tolerance = self.get_parameter(
            "progress_floor_tolerance").value
        self.progress_pass_margin = self.get_parameter(
            "progress_pass_margin").value
        self.progress_pass_max_xy_distance = self.get_parameter(
            "progress_pass_max_xy_distance").value
        self.initial_projection_horizon = self.get_parameter(
            "initial_projection_horizon").value

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
            Odometry, "body_pose", self.odom_callback,
            qos_profile_sensor_data)

        self.path = None
        self.points = []
        self.distances = []
        self.ranges = []
        self.current_index = 0
        self.path_progress = 0.0
        self.body_to_path_z_offset = None
        self.last_projection = None
        self.path_revision = 0
        self.last_odom = None
        self.ignored_path_updates = 0
        self.get_logger().debug(
            "Waiting for a global path; accept_first_path_only=%s" %
            self.accept_first_path_only)

    @staticmethod
    def odom_position(message):
        position = message.pose.pose.position
        return (position.x, position.y, position.z)

    def terrain_z_hint(self, position):
        if self.body_to_path_z_offset is None:
            return None
        return position[2] - self.body_to_path_z_offset

    def project_current_position(self, position):
        """Project odometry near the previously accepted route progress."""
        minimum = max(0.0, self.path_progress -
                      self.progress_backtrack_tolerance)
        maximum = min(self.distances[-1], self.path_progress +
                      self.progress_max_forward_distance)
        return project_onto_path(
            self.points, position,
            terrain_z_hint=self.terrain_z_hint(position),
            z_weight=self.projection_z_weight,
            min_progress=minimum, max_progress=maximum)

    def initialize_height_offset(self, points, position):
        """Estimate the body/path height offset near a fresh path's start."""
        probe = project_onto_path(
            points, position, z_weight=0.0,
            max_progress=self.initial_projection_horizon)
        self.body_to_path_z_offset = position[2] - probe.point[2]
        return probe.point[2]

    def path_callback(self, message):
        if self.accept_first_path_only and self.path is not None:
            self.ignored_path_updates += 1
            if self.ignored_path_updates == 1 or self.ignored_path_updates % 50 == 0:
                self.get_logger().debug(
                    "Ignoring global path update because first-path-only mode "
                    f"is active (ignored={self.ignored_path_updates})")
            return
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
        previous_index = self.current_index
        terrain_hint = None
        position = None
        if self.last_odom is not None:
            position = self.odom_position(self.last_odom)
            if self.path is not None and self.points and self.distances:
                previous_projection = self.project_current_position(position)
                self.path_progress = max(
                    self.path_progress, previous_projection.progress)
                terrain_hint = previous_projection.point[2]
                if self.body_to_path_z_offset is None:
                    self.body_to_path_z_offset = (
                        position[2] - previous_projection.point[2])
            elif self.body_to_path_z_offset is not None:
                terrain_hint = self.terrain_z_hint(position)
            else:
                terrain_hint = self.initialize_height_offset(points, position)

        distances = cumulative_xy_distance(points)
        projection = None
        inherited_progress = 0.0
        inherited_index = 0
        if position is not None:
            projection = project_onto_path(
                points, position, terrain_z_hint=terrain_hint,
                z_weight=self.projection_z_weight)
            inherited_progress = projection.progress
            inherited_index = active_segment_for_progress(
                ranges, distances, inherited_progress)
            measured_offset = position[2] - projection.point[2]
            if (self.body_to_path_z_offset is None or
                    projection.z_error <= self.progress_floor_tolerance):
                self.body_to_path_z_offset = measured_offset

        self.path = copy.deepcopy(message)
        self.points = points
        self.distances = distances
        self.ranges = ranges
        self.path_progress = inherited_progress
        self.current_index = inherited_index
        self.last_projection = projection
        self.path_revision += 1
        self.processed_path_pub.publish(self.path)
        self.publish_visualization()
        self.get_logger().debug(
            f"Path revision {self.path_revision}: segmented {len(points)} "
            f"points into {len(ranges)} sections; inherited progress="
            f"{self.path_progress:.2f}m, active={self.current_index + 1}/"
            f"{len(self.ranges)}, previous_active={previous_index + 1}")

    def odom_callback(self, message):
        self.last_odom = message
        if self.path is None or not self.ranges:
            return
        position = self.odom_position(message)
        projection = self.project_current_position(position)
        self.last_projection = projection
        self.path_progress = max(self.path_progress, projection.progress)
        terrain_hint = self.terrain_z_hint(position)

        # Slowly follow small body/path height-offset changes (suspension,
        # pitch and odometry noise) without allowing a wrong-floor projection
        # to redefine the offset in one sample.
        if (projection.xy_distance <= 1.0 and
                projection.z_error <= self.progress_floor_tolerance):
            measured_offset = position[2] - projection.point[2]
            self.body_to_path_z_offset = (
                measured_offset if self.body_to_path_z_offset is None else
                0.98 * self.body_to_path_z_offset + 0.02 * measured_offset)

        advanced = False
        while self.current_index + 1 < len(self.ranges):
            goal_index = self.ranges[self.current_index][1]
            goal_message = self.path.poses[goal_index].pose.position
            goal = (goal_message.x, goal_message.y, goal_message.z)
            reason = segment_handoff_reason(
                position, terrain_hint, projection, goal,
                self.distances[goal_index], self.reached_tolerance,
                self.progress_floor_tolerance, self.progress_pass_margin,
                self.progress_pass_max_xy_distance)
            if reason is None:
                break
            self.current_index += 1
            advanced = True
            self.get_logger().debug(
                f"Advanced to segment {self.current_index + 1}/"
                f"{len(self.ranges)} by {reason}; progress="
                f"{self.path_progress:.2f}m")
        if advanced:
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
