import pytest

from swarm_drone.swarm_config import ConfigError, SwarmConfig

VALID_DICT = {
    'swarm': {'num_drones': 4, 'leader_id': 0},
    'mapping_area': {'min_x': -10.0, 'max_x': 10.0, 'min_y': -10.0, 'max_y': 10.0},
    'takeoff': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'flight_altitude': 5.0},
    'coverage': {'waypoint_spacing': 1.0, 'boundary_margin': 0.5},
    'simulation': {'use_sim_time': True},
    'testing': {'enabled': False, 'test_drone_id': 0},
}


def _cfg(**overrides):
    import copy
    data = copy.deepcopy(VALID_DICT)
    for section, values in overrides.items():
        data[section].update(values)
    return SwarmConfig.from_dict(data)


def test_drone_0_is_leader_others_are_followers():
    cfg = _cfg()
    assert cfg.is_leader(0) is True
    assert cfg.is_leader(1) is False
    assert cfg.is_leader(2) is False
    assert cfg.is_leader(3) is False


def test_changing_num_drones_updates_swarm():
    cfg4 = _cfg(swarm={'num_drones': 4, 'leader_id': 0})
    cfg6 = _cfg(swarm={'num_drones': 6, 'leader_id': 0})
    assert cfg4.num_drones == 4
    assert cfg6.num_drones == 6
    assert cfg6.is_leader(5) is False
    assert cfg6.is_leader(0) is True


def test_missing_section_raises():
    data = {k: v for k, v in VALID_DICT.items() if k != 'mapping_area'}
    with pytest.raises(ConfigError):
        SwarmConfig.from_dict(data)


def test_invalid_num_drones_raises():
    with pytest.raises(ConfigError):
        _cfg(swarm={'num_drones': 0, 'leader_id': 0})


def test_invalid_leader_id_raises():
    with pytest.raises(ConfigError):
        _cfg(swarm={'num_drones': 4, 'leader_id': 4})


def test_zero_area_raises():
    with pytest.raises(ConfigError):
        _cfg(mapping_area={'min_x': 0.0, 'max_x': 0.0, 'min_y': -10.0, 'max_y': 10.0})


def test_negative_area_raises():
    with pytest.raises(ConfigError):
        _cfg(mapping_area={'min_x': 10.0, 'max_x': -10.0, 'min_y': -10.0, 'max_y': 10.0})


def test_invalid_test_drone_id_raises_only_when_testing_enabled():
    # not enabled -> out-of-range test_drone_id is ignored
    _cfg(testing={'enabled': False, 'test_drone_id': 99})
    with pytest.raises(ConfigError):
        _cfg(testing={'enabled': True, 'test_drone_id': 99})
