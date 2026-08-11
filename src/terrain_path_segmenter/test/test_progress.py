import pytest

from terrain_path_segmenter.segmentation import (
    PathProjection, active_segment_for_progress, cumulative_xy_distance,
    is_reverse_navigation_path, path_direction_similarity,
    project_onto_path, segment_handoff_reason)


def test_same_direction_rolling_path_is_not_a_reverse_task():
    previous = [(0.0, 0.0, 0.0), (5.0, 1.0, -2.0)]
    rolling = [(1.0, 0.2, -0.4), (7.0, 1.4, -2.8)]

    assert path_direction_similarity(previous, rolling) > 0.99
    assert not is_reverse_navigation_path(previous, rolling)


def test_upstairs_return_path_is_a_reverse_task():
    downstairs = [(0.0, 0.0, 0.0), (-30.0, 12.0, -13.0)]
    upstairs = [(-30.0, 12.0, -13.0), (0.0, 0.0, 0.0)]

    assert path_direction_similarity(downstairs, upstairs) == pytest.approx(-1.0)
    assert is_reverse_navigation_path(downstairs, upstairs)


def test_sideways_path_does_not_accidentally_replace_first_task():
    previous = [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)]
    sideways = [(5.0, 0.0, 0.0), (5.0, 5.0, 0.0)]

    assert path_direction_similarity(previous, sideways) == pytest.approx(0.0)
    assert not is_reverse_navigation_path(previous, sideways)


def test_path_revision_inherits_segment_from_current_projection():
    points = [(float(index), 0.0, 0.0) for index in range(11)]
    ranges = [(0, 3), (3, 7), (7, 10)]
    distances = cumulative_xy_distance(points)

    projection = project_onto_path(
        points, (5.2, 0.1, 0.35), terrain_z_hint=0.0)

    assert projection.progress == pytest.approx(5.2)
    assert active_segment_for_progress(
        ranges, distances, projection.progress) == 1


def test_height_hint_selects_correct_xy_overlapping_floor():
    # Both horizontal runs occupy the same XY line.  The connector is placed
    # away from the query so only the height hint determines the right floor.
    points = [
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 4.0, -3.0),
        (0.0, 0.0, -3.0),
        (4.0, 0.0, -3.0),
    ]

    upper = project_onto_path(
        points, (2.0, 0.05, 0.3), terrain_z_hint=0.0)
    lower = project_onto_path(
        points, (2.0, 0.05, -2.7), terrain_z_hint=-3.0)

    assert upper.point[2] == pytest.approx(0.0)
    assert upper.progress < 4.0
    assert lower.point[2] == pytest.approx(-3.0)
    assert lower.progress > 4.0


def test_progress_window_prevents_jump_to_later_self_intersection():
    points = [
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 4.0, 0.0),
        (0.0, 4.0, 0.0),
        (0.0, 0.0, 0.0),
    ]

    projection = project_onto_path(
        points, (0.05, 0.0, 0.0), min_progress=3.5,
        max_progress=7.5)

    assert 3.5 <= projection.progress <= 7.5
    assert projection.progress < 8.0


def test_handoff_inside_goal_circle_requires_same_floor():
    projection = PathProjection(
        segment_index=3, ratio=0.0, progress=3.8,
        point=(0.05, 0.0, -3.0), xy_distance=0.05, z_error=0.0)

    reason = segment_handoff_reason(
        position=(0.05, 0.0, -2.7), terrain_z_hint=-3.0,
        projection=projection, goal=(0.0, 0.0, 0.0),
        goal_progress=4.0, reached_tolerance=0.3,
        floor_tolerance=0.75)

    assert reason is None


def test_handoff_after_endpoint_does_not_require_goal_circle():
    projection = PathProjection(
        segment_index=5, ratio=0.3, progress=5.35,
        point=(5.35, 0.35, -0.2), xy_distance=0.02, z_error=0.01)

    reason = segment_handoff_reason(
        position=(5.35, 0.35, 0.1), terrain_z_hint=-0.19,
        projection=projection, goal=(5.0, 0.0, 0.0),
        goal_progress=5.0, reached_tolerance=0.3,
        floor_tolerance=0.75)

    assert reason == "passed_endpoint"


def test_handoff_does_not_pass_endpoint_when_far_from_path():
    projection = PathProjection(
        segment_index=5, ratio=0.3, progress=5.35,
        point=(5.35, 0.0, 0.0), xy_distance=2.0, z_error=0.0)

    reason = segment_handoff_reason(
        position=(5.35, 2.0, 0.3), terrain_z_hint=0.0,
        projection=projection, goal=(5.0, 0.0, 0.0),
        goal_progress=5.0, reached_tolerance=0.3,
        floor_tolerance=0.75, passed_max_xy_distance=1.0)

    assert reason is None


def test_active_segment_advances_at_shared_boundary():
    points = [(float(index), 0.0, 0.0) for index in range(7)]
    distances = cumulative_xy_distance(points)
    ranges = [(0, 3), (3, 6)]

    assert active_segment_for_progress(ranges, distances, 2.99) == 0
    assert active_segment_for_progress(ranges, distances, 3.0) == 1
