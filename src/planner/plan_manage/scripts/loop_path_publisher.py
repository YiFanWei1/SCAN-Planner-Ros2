#!/usr/bin/env python3

import copy
import math

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped


def parse_waypoints(values, close_path=True):
    if len(values) < 6 or len(values) % 3:
        raise ValueError("waypoints must contain at least two x,y,z triples")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("waypoints must contain finite values")
    points = [tuple(values[index:index + 3]) for index in range(0, len(values), 3)]
    if close_path and math.dist(points[0], points[-1]) > 1e-6:
        points.append(points[0])
    return points


def parse_terrain_profiles(values):
    """Parse x_center, half_width, y_start, y_end, z_start, z_end tuples."""
    if len(values) % 6:
        raise ValueError("terrain_profiles must contain groups of six values")
    profiles = [tuple(values[index:index + 6]) for index in range(0, len(values), 6)]
    for profile in profiles:
        x_center, half_width, y_start, y_end, z_start, z_end = profile
        if not all(math.isfinite(value) for value in profile):
            raise ValueError("terrain_profiles must contain finite values")
        if half_width < 0.0 or y_end <= y_start:
            raise ValueError("invalid terrain profile dimensions")
    return profiles


def parse_terrain_platforms(values):
    """Parse x_min, x_max, y_min, y_max, height tuples."""
    if len(values) % 5:
        raise ValueError("terrain_platforms must contain groups of five values")
    platforms = [tuple(values[index:index + 5]) for index in range(0, len(values), 5)]
    for platform in platforms:
        x_min, x_max, y_min, y_max, height = platform
        if not all(math.isfinite(value) for value in platform):
            raise ValueError("terrain_platforms must contain finite values")
        if x_max <= x_min or y_max <= y_min:
            raise ValueError("invalid terrain platform dimensions")
    return platforms


def terrain_height(x, y, profiles, fallback_z=0.0, platforms=None):
    """Return the ground height of the ramp/platform covering an XY point."""
    for x_min, x_max, y_min, y_max, height in platforms or []:
        if x_min <= x <= x_max and y_min <= y <= y_max:
            return height
    for x_center, half_width, y_start, y_end, z_start, z_end in profiles:
        if abs(x - x_center) > half_width:
            continue
        if y <= y_start:
            return z_start
        if y >= y_end:
            return z_end
        ratio = (y - y_start) / (y_end - y_start)
        return z_start + ratio * (z_end - z_start)
    return fallback_z


def densify_waypoints(points, spacing, terrain_profiles=None, terrain_platforms=None):
    """Sample a polyline in XY and optionally project every point onto terrain."""
    terrain_profiles = terrain_profiles or []
    first = points[0]
    first_z = terrain_height(
        first[0], first[1], terrain_profiles, first[2], terrain_platforms)
    dense = [(first[0], first[1], first_z)]
    for start, end in zip(points, points[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, math.ceil(distance / spacing))
        for index in range(1, steps + 1):
            ratio = index / steps
            x = start[0] + ratio * (end[0] - start[0])
            y = start[1] + ratio * (end[1] - start[1])
            interpolated_z = start[2] + ratio * (end[2] - start[2])
            z = terrain_height(
                x, y, terrain_profiles, interpolated_z, terrain_platforms)
            dense.append((x, y, z))
    return dense


class LoopPathPublisher(Node):
    def __init__(self):
        super().__init__("loop_path_publisher")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("waypoints", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("shuttle_waypoints", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("loop_enabled", True)
        self.declare_parameter("loop_position_tolerance", 0.5)
        self.declare_parameter("loop_departure_distance", 1.0)
        self.declare_parameter("loop_speed_tolerance", 0.2)
        self.declare_parameter("loop_cooldown", 3.0)
        self.declare_parameter("shuttle_repeat_enabled", True)
        self.declare_parameter("cycle_via_initial_route", False)
        self.declare_parameter("path_point_spacing", 0.20)
        self.declare_parameter("terrain_profiles", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("terrain_platforms", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("body_height", 0.4)

        self.frame_id = self.get_parameter("frame_id").value
        self.loop_enabled = self.get_parameter("loop_enabled").value
        self.waypoints = parse_waypoints(
            self.get_parameter("waypoints").value, close_path=self.loop_enabled)
        try:
            shuttle_values = self.get_parameter("shuttle_waypoints").value
        except ParameterUninitializedException:
            shuttle_values = []
        self.shuttle_waypoints = (
            parse_waypoints(shuttle_values, close_path=False) if shuttle_values else [])
        self.active_waypoints = self.waypoints
        self.initial_route_complete = False
        self.shuttle_forward = True
        self.position_tolerance = self.get_parameter("loop_position_tolerance").value
        self.departure_distance = self.get_parameter("loop_departure_distance").value
        self.speed_tolerance = self.get_parameter("loop_speed_tolerance").value
        self.cooldown = self.get_parameter("loop_cooldown").value
        self.shuttle_repeat_enabled = self.get_parameter("shuttle_repeat_enabled").value
        self.cycle_via_initial_route = self.get_parameter("cycle_via_initial_route").value
        self.path_point_spacing = self.get_parameter("path_point_spacing").value
        try:
            terrain_profile_values = self.get_parameter("terrain_profiles").value
        except ParameterUninitializedException:
            terrain_profile_values = []
        try:
            terrain_platform_values = self.get_parameter("terrain_platforms").value
        except ParameterUninitializedException:
            terrain_platform_values = []
        self.terrain_profiles = parse_terrain_profiles(terrain_profile_values)
        self.terrain_platforms = parse_terrain_platforms(terrain_platform_values)
        self.body_height = self.get_parameter("body_height").value
        if min(self.position_tolerance, self.departure_distance, self.speed_tolerance) < 0.0:
            raise ValueError("loop tolerances must be non-negative")
        if self.cooldown < 0.0:
            raise ValueError("loop_cooldown must be non-negative")
        if self.path_point_spacing <= 0.0:
            raise ValueError("path_point_spacing must be positive")

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(Path, "initial_path", path_qos)
        self.body_path_publisher = self.create_publisher(
            Path, "initial_body_path", path_qos)
        self.subscription = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, 20)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.last_odom = None
        self.last_publish_time = None
        self.departed = False
        self.lap_count = 0
        dense_count = len(densify_waypoints(
            self.waypoints, self.path_point_spacing, self.terrain_profiles,
            self.terrain_platforms))
        self.get_logger().info(
            f"Waiting to publish {'closed loop' if self.loop_enabled else 'one-way route'} "
            f"with {dense_count} dense points")

    def make_path(self):
        message = Path()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y, z in densify_waypoints(
                self.active_waypoints, self.path_point_spacing, self.terrain_profiles,
                self.terrain_platforms):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def publish_path(self, waypoints=None):
        if waypoints is not None:
            self.active_waypoints = waypoints
        ground_path = self.make_path()
        self.publisher.publish(ground_path)
        body_path = Path()
        body_path.header = ground_path.header
        for ground_pose in ground_path.poses:
            pose = PoseStamped()
            pose.header = ground_pose.header
            pose.pose = copy.deepcopy(ground_pose.pose)
            pose.pose.position.z += self.body_height
            body_path.poses.append(pose)
        self.body_path_publisher.publish(body_path)
        self.last_publish_time = self.get_clock().now()
        self.departed = False
        self.lap_count += 1
        route_kind = ("shuttle route" if self.initial_route_complete else
                      ("closed reference path" if self.loop_enabled else "initial approach route"))
        self.get_logger().info(
            f"Published {route_kind} with {len(ground_path.poses)} dense points, "
            f"run {self.lap_count}")

    def odom_callback(self, message):
        self.last_odom = message

    def timer_callback(self):
        if self.last_odom is None or self.publisher.get_subscription_count() == 0:
            return
        if self.last_publish_time is None:
            self.publish_path()
            return
        if not self.loop_enabled and not self.shuttle_waypoints:
            return

        position = self.last_odom.pose.pose.position
        target = self.active_waypoints[-1]
        distance = math.dist((position.x, position.y, position.z),
                             (target[0], target[1], target[2] + 0.4))
        twist = self.last_odom.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        elapsed = (self.get_clock().now() - self.last_publish_time).nanoseconds / 1e9
        if (distance <= self.position_tolerance and speed <= self.speed_tolerance and
                elapsed >= self.cooldown and self.shuttle_waypoints):
            if not self.initial_route_complete:
                self.initial_route_complete = True
                self.shuttle_forward = True
            elif not self.shuttle_repeat_enabled:
                return
            else:
                self.shuttle_forward = not self.shuttle_forward
            if self.cycle_via_initial_route:
                route = self.shuttle_waypoints if self.shuttle_forward else self.waypoints
            else:
                route = (self.shuttle_waypoints if self.shuttle_forward
                         else list(reversed(self.shuttle_waypoints)))
            self.publish_path(route)
            return
        if self.loop_enabled:
            start = self.active_waypoints[0]
            start_distance = math.hypot(position.x - start[0], position.y - start[1])
            if start_distance >= self.departure_distance:
                self.departed = True
            if (self.departed and start_distance <= self.position_tolerance and
                    speed <= self.speed_tolerance and elapsed >= self.cooldown):
                self.publish_path()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LoopPathPublisher()
        rclpy.spin(node)
    except (ValueError, KeyboardInterrupt, rclpy.exceptions.ROSInterruptException) as error:
        if isinstance(error, ValueError):
            rclpy.logging.get_logger("loop_path_publisher").error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
