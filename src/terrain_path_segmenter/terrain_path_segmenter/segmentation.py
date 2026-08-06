"""Pure path segmentation algorithms, independent of ROS messages."""

import math


def cumulative_xy_distance(points):
    """Return horizontal arc length for a sequence of XYZ points."""
    distances = [0.0]
    for first, second in zip(points, points[1:]):
        distances.append(
            distances[-1] + math.hypot(second[0] - first[0],
                                       second[1] - first[1]))
    return distances


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
