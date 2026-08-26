import pytest

from swarm_drone.coverage_planner import generate_coverage_path
from swarm_drone.region_allocator import Region


def test_path_is_generated():
    region = Region(drone_id=0, min_x=0.0, max_x=10.0, min_y=0.0, max_y=10.0)
    waypoints = generate_coverage_path(region, waypoint_spacing=1.0, boundary_margin=0.5)
    assert len(waypoints) > 0


@pytest.mark.parametrize('region', [
    Region(drone_id=0, min_x=0.0, max_x=10.0, min_y=0.0, max_y=10.0),
    Region(drone_id=1, min_x=-5.0, max_x=5.0, min_y=-5.0, max_y=5.0),
    Region(drone_id=2, min_x=0.0, max_x=3.0, min_y=0.0, max_y=20.0),
])
def test_waypoints_stay_inside_region_with_margin(region):
    margin = 0.5
    waypoints = generate_coverage_path(region, waypoint_spacing=1.0, boundary_margin=margin)
    for x, y, _z in waypoints:
        assert region.min_x + margin - 1e-9 <= x <= region.max_x - margin + 1e-9
        assert region.min_y + margin - 1e-9 <= y <= region.max_y - margin + 1e-9


def test_waypoint_spacing_is_reasonable():
    region = Region(drone_id=0, min_x=0.0, max_x=10.0, min_y=0.0, max_y=10.0)
    spacing = 2.0
    waypoints = generate_coverage_path(region, waypoint_spacing=spacing, boundary_margin=0.0)
    row_ys = sorted({round(y, 6) for _x, y, _z in waypoints})
    for a, b in zip(row_ys, row_ys[1:]):
        assert (b - a) == pytest.approx(spacing, abs=1e-6) or (b - a) <= spacing + 1e-6


@pytest.mark.parametrize('width,height', [(2.0, 2.0), (20.0, 5.0), (1.0, 1.0), (7.5, 13.2)])
def test_different_region_sizes_work(width, height):
    region = Region(drone_id=0, min_x=0.0, max_x=width, min_y=0.0, max_y=height)
    waypoints = generate_coverage_path(region, waypoint_spacing=0.5, boundary_margin=0.1)
    assert len(waypoints) >= 2


def test_zero_spacing_raises():
    region = Region(drone_id=0, min_x=0.0, max_x=10.0, min_y=0.0, max_y=10.0)
    with pytest.raises(ValueError):
        generate_coverage_path(region, waypoint_spacing=0.0, boundary_margin=0.5)


def test_margin_too_large_raises():
    region = Region(drone_id=0, min_x=0.0, max_x=1.0, min_y=0.0, max_y=1.0)
    with pytest.raises(ValueError):
        generate_coverage_path(region, waypoint_spacing=0.5, boundary_margin=1.0)


def test_altitude_is_applied_to_all_waypoints():
    region = Region(drone_id=0, min_x=0.0, max_x=4.0, min_y=0.0, max_y=4.0)
    waypoints = generate_coverage_path(
        region, waypoint_spacing=1.0, boundary_margin=0.0, altitude=5.0)
    assert all(z == 5.0 for _x, _y, z in waypoints)
