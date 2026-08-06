import math

import pytest

from terrain_path_segmenter.segmentation import segment_path, segment_slope
from terrain_path_segmenter.segmentation import cumulative_xy_distance


def make_piecewise_path(spacing=0.2):
    points = []
    for index in range(61):
        y = index * spacing
        if y <= 2.0:
            z = 0.0
        elif y <= 6.0:
            z = (y - 2.0) * 0.25
        else:
            z = 1.0
        points.append((0.0, y, z))
    return points


def test_piecewise_flat_ramp_flat_is_split_into_three_sections():
    points = make_piecewise_path()
    ranges = segment_path(points, max_linear_z_error=0.02,
                          slope_merge_threshold=0.03,
                          minimum_segment_length=0.4)
    assert len(ranges) == 3
    assert ranges[0][0] == 0
    assert ranges[-1][1] == len(points) - 1
    assert ranges[0][1] == ranges[1][0]
    assert ranges[1][1] == ranges[2][0]
    distances = cumulative_xy_distance(points)
    slopes = [segment_slope(points, distances, start, end)
              for start, end in ranges]
    assert slopes == pytest.approx([0.0, 0.25, 0.0], abs=1e-8)


def test_single_linear_slope_stays_in_one_section():
    points = [(float(i), 0.0, 0.1 * i) for i in range(20)]
    assert segment_path(points) == [(0, len(points) - 1)]


def test_small_z_noise_does_not_create_sections():
    points = [(i * 0.2, 0.0, 0.1 * i * 0.2 + 0.005 * math.sin(i))
              for i in range(40)]
    assert len(segment_path(points, max_linear_z_error=0.02)) == 1


def test_invalid_threshold_is_rejected():
    with pytest.raises(ValueError):
        segment_path([(0, 0, 0), (1, 0, 0)], max_linear_z_error=0.0)
