import importlib.util
import math
import os

import pytest


SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts", "loop_path_publisher.py")
)
SPEC = importlib.util.spec_from_file_location("loop_path_publisher", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_parse_waypoints_closes_open_path():
    points = MODULE.parse_waypoints([0.0, 0.0, 0.0, 1.0, 2.0, 0.0])
    assert points == [(0.0, 0.0, 0.0), (1.0, 2.0, 0.0), (0.0, 0.0, 0.0)]


def test_parse_waypoints_preserves_closed_path():
    points = MODULE.parse_waypoints(
        [0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0]
    )
    assert len(points) == 3


def test_parse_waypoints_can_preserve_open_route():
    points = MODULE.parse_waypoints(
        [0.0, 0.0, 0.0, 1.0, 2.0, 0.5], close_path=False
    )
    assert points == [(0.0, 0.0, 0.0), (1.0, 2.0, 0.5)]


def test_densify_waypoints_interpolates_xyz_and_keeps_endpoint():
    points = MODULE.densify_waypoints(
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.5)], 0.25)
    assert len(points) == 5
    assert points[0] == (0.0, 0.0, 0.0)
    assert points[-1] == (1.0, 0.0, 0.5)
    assert points[2][2] == pytest.approx(0.25)


@pytest.fixture
def terrain_profiles():
    return MODULE.parse_terrain_profiles([
        -6.0, 1.5, 1.0, 5.0, 0.0, 1.0,
        6.0, 1.5, 3.0, 5.0, 0.0, 1.0,
    ])


@pytest.mark.parametrize("y, expected", [
    (1.0, 0.0), (3.0, 0.5), (5.0, 1.0), (7.0, 1.0),
])
def test_gentle_ramp_height(terrain_profiles, y, expected):
    assert MODULE.terrain_height(-6.0, y, terrain_profiles) == pytest.approx(expected)


@pytest.mark.parametrize("y, expected", [
    (3.0, 0.0), (4.0, 0.5), (5.0, 1.0), (7.0, 1.0),
])
def test_steep_ramp_height(terrain_profiles, y, expected):
    assert MODULE.terrain_height(6.0, y, terrain_profiles) == pytest.approx(expected)


def test_connector_platform_height_overrides_flat_fallback(terrain_profiles):
    platforms = MODULE.parse_terrain_platforms([-4.1, 4.1, 5.5, 8.5, 1.0])
    assert MODULE.terrain_height(
        0.0, 7.0, terrain_profiles, 0.0, platforms) == pytest.approx(1.0)
    assert MODULE.terrain_height(
        0.0, 5.4, terrain_profiles, 0.0, platforms) == pytest.approx(0.0)


def test_connector_route_stays_at_platform_height(terrain_profiles):
    platforms = MODULE.parse_terrain_platforms([-4.1, 4.1, 5.5, 8.5, 1.0])
    route = [(-6.0, 7.0, 1.0), (6.0, 7.0, 1.0)]
    dense = MODULE.densify_waypoints(route, 0.2, terrain_profiles, platforms)
    assert len(dense) == 61
    assert all(point[2] == pytest.approx(1.0) for point in dense)


def test_terrain_path_is_dense_continuous_and_reverse_symmetric(terrain_profiles):
    route = [(-6.0, 0.0, 0.0), (-6.0, 6.0, 1.0)]
    forward = MODULE.densify_waypoints(route, 0.2, terrain_profiles)
    reverse = MODULE.densify_waypoints(list(reversed(route)), 0.2, terrain_profiles)
    for forward_point, reverse_point in zip(forward, reversed(reverse)):
        assert forward_point == pytest.approx(reverse_point, abs=1e-12)
    assert max(math.dist(a, b) for a, b in zip(forward, forward[1:])) < 0.21
    assert all(a[2] <= b[2] for a, b in zip(forward, forward[1:]))


@pytest.mark.parametrize("values", [[], [0.0, 0.0, 0.0], [0.0] * 7,
                                     [0.0, 0.0, 0.0, math.nan, 0.0, 0.0]])
def test_parse_waypoints_rejects_invalid_values(values):
    with pytest.raises(ValueError):
        MODULE.parse_waypoints(values)
