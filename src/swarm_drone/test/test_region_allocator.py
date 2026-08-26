import itertools

import pytest

from swarm_drone.region_allocator import allocate_regions


def _overlap(a, b):
    x_overlap = max(0.0, min(a.max_x, b.max_x) - max(a.min_x, b.min_x))
    y_overlap = max(0.0, min(a.max_y, b.max_y) - max(a.min_y, b.min_y))
    return x_overlap * y_overlap


@pytest.mark.parametrize('n', [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
def test_correct_number_of_regions(n):
    regions = allocate_regions(n, -10.0, 10.0, -10.0, 10.0)
    assert len(regions) == n
    assert sorted(r.drone_id for r in regions) == list(range(n))


@pytest.mark.parametrize('n', [1, 4, 5, 6, 7, 9])
def test_no_overlaps(n):
    regions = allocate_regions(n, -10.0, 10.0, -10.0, 10.0)
    for a, b in itertools.combinations(regions, 2):
        assert _overlap(a, b) < 1e-9, f'{a} overlaps {b}'


@pytest.mark.parametrize('n', [1, 4, 6, 8, 9])
def test_approximately_equal_area(n):
    regions = allocate_regions(n, -10.0, 10.0, -10.0, 10.0)
    total_area = 20.0 * 20.0
    expected = total_area / n
    for r in regions:
        assert r.area() == pytest.approx(expected, rel=0.25), (
            f'region {r.drone_id} area {r.area()} far from expected {expected}')


@pytest.mark.parametrize('n', [1, 4, 5, 9])
def test_regions_inside_requested_area(n):
    min_x, max_x, min_y, max_y = -10.0, 10.0, -5.0, 5.0
    regions = allocate_regions(n, min_x, max_x, min_y, max_y)
    for r in regions:
        assert min_x - 1e-9 <= r.min_x <= r.max_x <= max_x + 1e-9
        assert min_y - 1e-9 <= r.min_y <= r.max_y <= max_y + 1e-9


def test_regions_cover_full_area():
    regions = allocate_regions(4, -10.0, 10.0, -10.0, 10.0)
    total = sum(r.area() for r in regions)
    assert total == pytest.approx(400.0, rel=1e-6)


def test_invalid_num_drones_raises():
    with pytest.raises(ValueError):
        allocate_regions(0, -10.0, 10.0, -10.0, 10.0)


def test_invalid_area_raises():
    with pytest.raises(ValueError):
        allocate_regions(4, 10.0, -10.0, -10.0, 10.0)


def test_reconfiguring_num_drones_changes_allocation():
    regions_4 = allocate_regions(4, -10.0, 10.0, -10.0, 10.0)
    regions_6 = allocate_regions(6, -10.0, 10.0, -10.0, 10.0)
    assert len(regions_4) == 4
    assert len(regions_6) == 6
    assert regions_4[0].area() != regions_6[0].area()
