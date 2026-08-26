"""
Loads and validates config/swarm.yaml.

Single source of truth for all swarm/mission parameters - nodes should
not hard-code these values.
"""

from dataclasses import dataclass

import yaml


class ConfigError(Exception):
    pass


@dataclass
class SwarmConfig:
    num_drones: int
    leader_id: int
    ack_timeout_sec: float
    heartbeat_timeout_sec: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    takeoff_x: float
    takeoff_y: float
    takeoff_z: float
    flight_altitude: float
    spawn_ring_spacing: float
    stagger_delay_sec: float
    waypoint_spacing: float
    boundary_margin: float
    cruise_speed: float
    waypoint_tolerance: float
    use_sim_time: bool
    world_name: str
    testing_enabled: bool
    test_drone_id: int

    @classmethod
    def from_dict(cls, data):
        try:
            swarm = data['swarm']
            area = data['mapping_area']
            takeoff = data['takeoff']
            coverage = data['coverage']
            sim = data.get('simulation', {})
            testing = data.get('testing', {})
        except (KeyError, TypeError) as e:
            raise ConfigError(f'Missing required YAML section: {e}') from e

        try:
            cfg = cls(
                num_drones=int(swarm['num_drones']),
                leader_id=int(swarm['leader_id']),
                ack_timeout_sec=float(swarm.get('ack_timeout_sec', 10.0)),
                heartbeat_timeout_sec=float(swarm.get('heartbeat_timeout_sec', 5.0)),
                min_x=float(area['min_x']),
                max_x=float(area['max_x']),
                min_y=float(area['min_y']),
                max_y=float(area['max_y']),
                takeoff_x=float(takeoff['x']),
                takeoff_y=float(takeoff['y']),
                takeoff_z=float(takeoff['z']),
                flight_altitude=float(takeoff['flight_altitude']),
                spawn_ring_spacing=float(takeoff.get('spawn_ring_spacing', 0.5)),
                stagger_delay_sec=float(takeoff.get('stagger_delay_sec', 2.0)),
                waypoint_spacing=float(coverage['waypoint_spacing']),
                boundary_margin=float(coverage['boundary_margin']),
                cruise_speed=float(coverage.get('cruise_speed', 1.0)),
                waypoint_tolerance=float(coverage.get('waypoint_tolerance', 0.3)),
                use_sim_time=bool(sim.get('use_sim_time', True)),
                world_name=str(sim.get('world_name', 'mapping_world')),
                testing_enabled=bool(testing.get('enabled', False)),
                test_drone_id=int(testing.get('test_drone_id', 0)),
            )
        except (KeyError, TypeError, ValueError) as e:
            raise ConfigError(f'Invalid or missing YAML parameter: {e}') from e

        cfg.validate()
        return cfg

    def validate(self):
        if self.num_drones < 1:
            raise ConfigError(f'swarm.num_drones must be >= 1, got {self.num_drones}')
        if not (0 <= self.leader_id < self.num_drones):
            raise ConfigError(
                f'swarm.leader_id ({self.leader_id}) must be a valid drone id '
                f'in [0, {self.num_drones - 1}]')
        if self.max_x <= self.min_x or self.max_y <= self.min_y:
            raise ConfigError(
                f'mapping_area is degenerate or inverted: x=[{self.min_x},{self.max_x}] '
                f'y=[{self.min_y},{self.max_y}]')
        if self.waypoint_spacing <= 0:
            raise ConfigError(
                f'coverage.waypoint_spacing must be > 0, got {self.waypoint_spacing}')
        if self.boundary_margin < 0:
            raise ConfigError(f'coverage.boundary_margin must be >= 0, got {self.boundary_margin}')
        usable_width = self.area_width() - 2 * self.boundary_margin
        usable_height = self.area_height() - 2 * self.boundary_margin
        if usable_width <= 0 or usable_height <= 0:
            raise ConfigError('coverage.boundary_margin leaves no usable area inside mapping_area')
        if self.testing_enabled and not (0 <= self.test_drone_id < self.num_drones):
            raise ConfigError(
                f'testing.test_drone_id ({self.test_drone_id}) is not a valid drone id '
                f'in [0, {self.num_drones - 1}]')

    def is_leader(self, drone_id):
        if not (0 <= drone_id < self.num_drones):
            raise ConfigError(
                f'drone id {drone_id} is not valid for num_drones={self.num_drones}')
        return drone_id == self.leader_id

    def area_width(self):
        return self.max_x - self.min_x

    def area_height(self):
        return self.max_y - self.min_y

    @classmethod
    def from_yaml_file(cls, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        if not data:
            raise ConfigError(f'YAML config at {path} is empty')
        return cls.from_dict(data)


def default_config_path():
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('swarm_drone') + '/config/swarm.yaml'
