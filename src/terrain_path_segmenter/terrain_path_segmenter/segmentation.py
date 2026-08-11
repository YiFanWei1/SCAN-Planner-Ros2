"""Pure path segmentation and progress algorithms, independent of ROS."""

import math
from typing import NamedTuple


class PathProjection(NamedTuple):
    """Closest point on an XYZ path under an XY/terrain-height metric."""

    segment_index: int
    ratio: float
    progress: float
    point: tuple
    xy_distance: float
    z_error: float


def cumulative_xy_distance(points):
    """Return horizontal arc length for a sequence of XYZ points."""
    distances = [0.0]
    for first, second in zip(points, points[1:]):
        distances.append(
            distances[-1] + math.hypot(second[0] - first[0],
                                       second[1] - first[1]))
    return distances


def path_direction_similarity(first_points, second_points):
    """Return the cosine between the start-to-end directions of two paths.

    A value near one means that both paths describe the same navigation
    direction, while a value near minus one identifies a return trip.  ``None``
    is returned for a path whose endpoints are effectively coincident because
    its direction is undefined.
    """
    if len(first_points) < 2 or len(second_points) < 2:
        return None

    directions = []
    for points in (first_points, second_points):
        direction = tuple(points[-1][axis] - points[0][axis]
                          for axis in range(3))
        length = math.sqrt(sum(value * value for value in direction))
        if length <= 1e-6:
            return None
        directions.append(tuple(value / length for value in direction))

    return sum(first * second
               for first, second in zip(directions[0], directions[1]))


def is_reverse_navigation_path(previous_points, candidate_points,
                               maximum_direction_cosine=-0.25):
    """Return whether ``candidate_points`` starts a reverse-direction task.

    ``accept_first_path_only`` is used to reject rolling replans belonging to
    one task.  A real return trip must nevertheless be accepted.  Requiring a
    clearly negative direction cosine distinguishes that return trip from
    ordinary forward extensions and small endpoint motion.
    """
    if (not math.isfinite(maximum_direction_cosine) or
            maximum_direction_cosine < -1.0 or
            maximum_direction_cosine > 1.0):
        raise ValueError("maximum_direction_cosine must be within [-1, 1]")
    similarity = path_direction_similarity(previous_points, candidate_points)
    return (similarity is not None and
            similarity <= maximum_direction_cosine)


def project_onto_path(points, position, terrain_z_hint=None, z_weight=2.0,
                      min_progress=0.0, max_progress=None):
    """Project a position onto a path while disambiguating overlapping floors.

    ``position`` is normally the body odometry position.  Since a global path
    may describe terrain rather than body-center height, callers can provide a
    ``terrain_z_hint`` obtained from the previous path projection.  This makes
    an XY-overlapping stair flight on another floor more expensive without
    assuming a fixed robot body height.

    The returned progress is horizontal arc length.  Restricting its range is
    useful for continuous odometry updates: it prevents a self-intersection
    farther down the route from being selected in a single update.
    """
    if not points:
        raise ValueError("cannot project onto an empty path")
    if len(position) != 3 or not all(math.isfinite(v) for v in position):
        raise ValueError("position must be a finite XYZ triple")
    if z_weight < 0.0 or not math.isfinite(z_weight):
        raise ValueError("z_weight must be finite and non-negative")

    distances = cumulative_xy_distance(points)
    total_length = distances[-1]
    lower = max(0.0, min(float(min_progress), total_length))
    upper = total_length if max_progress is None else max(
        lower, min(float(max_progress), total_length))
    if not math.isfinite(lower) or not math.isfinite(upper):
        raise ValueError("progress limits must be finite")
    if terrain_z_hint is not None and not math.isfinite(terrain_z_hint):
        raise ValueError("terrain_z_hint must be finite")

    best = None
    best_key = None
    for index, (first, second) in enumerate(zip(points, points[1:])):
        segment_start = distances[index]
        segment_end = distances[index + 1]
        segment_length = segment_end - segment_start
        if segment_end < lower - 1e-9 or segment_start > upper + 1e-9:
            continue
        if segment_length <= 1e-9:
            continue

        dx = second[0] - first[0]
        dy = second[1] - first[1]
        ratio = ((position[0] - first[0]) * dx +
                 (position[1] - first[1]) * dy) / (segment_length ** 2)
        minimum_ratio = max(0.0, (lower - segment_start) / segment_length)
        maximum_ratio = min(1.0, (upper - segment_start) / segment_length)
        ratio = max(minimum_ratio, min(maximum_ratio, ratio))
        point = tuple(first[axis] + ratio * (second[axis] - first[axis])
                      for axis in range(3))
        xy_distance = math.hypot(position[0] - point[0],
                                 position[1] - point[1])
        z_error = (0.0 if terrain_z_hint is None else
                   abs(terrain_z_hint - point[2]))
        score = xy_distance ** 2 + (z_weight * z_error) ** 2
        progress = segment_start + ratio * segment_length

        # Earlier progress wins an exact tie.  Fresh navigation paths normally
        # begin at the robot, so this also avoids jumping to a later same-floor
        # self-intersection when no prior progress exists on the new revision.
        key = (score, progress)
        if best_key is None or key < best_key:
            best_key = key
            best = PathProjection(index, ratio, progress, point,
                                  xy_distance, z_error)

    if best is not None:
        return best

    # A path made only of repeated XY points has zero horizontal length.  It is
    # not useful for progress tracking, but returning its first point keeps the
    # caller deterministic and lets normal path validation handle it.
    point = tuple(points[0])
    xy_distance = math.hypot(position[0] - point[0],
                             position[1] - point[1])
    z_error = (0.0 if terrain_z_hint is None else
               abs(terrain_z_hint - point[2]))
    return PathProjection(0, 0.0, 0.0, point, xy_distance, z_error)


def active_segment_for_progress(ranges, distances, progress):
    """Return the section whose endpoint is still ahead of ``progress``."""
    if not ranges:
        raise ValueError("segment ranges cannot be empty")
    for index, (_, end) in enumerate(ranges[:-1]):
        if progress < distances[end] - 1e-6:
            return index
    return len(ranges) - 1


def segment_handoff_reason(position, terrain_z_hint, projection,
                           goal, goal_progress, reached_tolerance,
                           floor_tolerance=0.75,
                           passed_progress_margin=0.02,
                           passed_max_xy_distance=1.0):
    """Return why a segment should advance, or ``None`` if it should not.

    Handoff is accepted either inside the traditional XY goal circle or after
    the robot's path projection has passed the endpoint.  Both checks are
    gated by terrain height when a hint is available, preventing a pose on an
    XY-overlapping floor from completing the wrong stair segment.
    """
    if (reached_tolerance < 0.0 or floor_tolerance < 0.0 or
            passed_progress_margin < 0.0 or passed_max_xy_distance < 0.0):
        raise ValueError("handoff tolerances must be non-negative")
    z_compatible = (terrain_z_hint is None or
                    abs(terrain_z_hint - goal[2]) <= floor_tolerance)
    distance = math.hypot(position[0] - goal[0], position[1] - goal[1])
    if z_compatible and distance <= reached_tolerance:
        return "goal_tolerance"
    projection_matches_floor = (terrain_z_hint is None or
                                projection.z_error <= floor_tolerance)
    if (projection_matches_floor and
            projection.xy_distance <= passed_max_xy_distance and
            projection.progress >= goal_progress + passed_progress_margin):
        return "passed_endpoint"
    return None


def line_error(points, distances, start, end):
    """Return maximum Z error and its index against the endpoint line."""
    span = distances[end] - distances[start]
    if span <= 1e-9:
        return 0.0, start
    maximum = 0.0
    maximum_index = start
    for index in range(start + 1, end):
        ratio = (distances[index] - distances[start]) / span
        expected = points[start][2] + ratio * (
            points[end][2] - points[start][2])
        error = abs(points[index][2] - expected)
        if error > maximum:
            maximum = error
            maximum_index = index
    return maximum, maximum_index


def segment_slope(points, distances, start, end):
    """Return dz/ds_xy for a segment."""
    span = distances[end] - distances[start]
    return 0.0 if span <= 1e-9 else (points[end][2] - points[start][2]) / span


def segment_path(points, max_linear_z_error=0.04,
                 slope_merge_threshold=0.04,
                 minimum_segment_length=0.5):
    """Split XYZ points into overlapping index ranges with near-linear Z.

    The returned ranges are inclusive. Adjacent ranges share their boundary
    point, which guarantees a continuous target height during later handoff.
    """
    if len(points) < 2:
        return [(0, 0)] if points else []
    if max_linear_z_error <= 0.0:
        raise ValueError("max_linear_z_error must be positive")
    if slope_merge_threshold < 0.0 or minimum_segment_length < 0.0:
        raise ValueError("segment thresholds must be non-negative")
    if not all(len(point) == 3 and all(math.isfinite(v) for v in point)
               for point in points):
        raise ValueError("points must be finite XYZ triples")

    distances = cumulative_xy_distance(points)
    corners = {0, len(points) - 1}

    def split(start, end):
        error, index = line_error(points, distances, start, end)
        left_length = distances[index] - distances[start]
        right_length = distances[end] - distances[index]
        if (error > max_linear_z_error and index > start and index < end and
                left_length >= minimum_segment_length and
                right_length >= minimum_segment_length):
            corners.add(index)
            split(start, index)
            split(index, end)

    split(0, len(points) - 1)
    ordered = sorted(corners)
    ranges = [(ordered[i], ordered[i + 1])
              for i in range(len(ordered) - 1)]

    # RDP may retain corners caused only by small noise. Merge adjacent pieces
    # when their slopes agree and the combined interval still fits well.
    merged = []
    for current in ranges:
        if not merged:
            merged.append(current)
            continue
        previous = merged[-1]
        previous_slope = segment_slope(
            points, distances, previous[0], previous[1])
        current_slope = segment_slope(
            points, distances, current[0], current[1])
        combined_error, _ = line_error(
            points, distances, previous[0], current[1])
        if (abs(previous_slope - current_slope) <= slope_merge_threshold and
                combined_error <= max_linear_z_error):
            merged[-1] = (previous[0], current[1])
        else:
            merged.append(current)
    return merged
