# Swarm Drone — Full Codebase Context

## What This Project Is

A **ROS 2 Humble + Gazebo Ignition Fortress** simulation of a **multi-drone swarm performing autonomous, leader-coordinated area coverage**. By default 4 drones (1 leader + 3 followers) each fly a **boustrophedon (lawnmower) path** over an equal sub-region of a 20×20 m flat world. The control stack is layered so the Gazebo backend can later be replaced with PX4/MAVROS without touching swarm logic.

---

## Repository Layout

```
Final-swarm-drone/               ← colcon workspace root
├── README.md
├── .gitignore
├── build/                       ← colcon build artefacts (not in repo)
├── install/                     ← colcon install tree  (not in repo)
├── log/                         ← colcon log           (not in repo)
└── src/
    └── swarm_drone/             ← the single ament_python ROS 2 package
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── LICENSE              (Apache-2.0)
        ├── config/
        │   └── swarm.yaml       ← single source of truth for all parameters
        ├── launch/
        │   ├── auto.launch.py            ← launches swarm logic nodes (auto start)
        │   ├── simulation.launch.py      ← launches Gazebo & spawns physical drones
        │   ├── swarm_launch.launch.py    ← unified launch (Gazebo + backend nodes + wait for CLI goal)
        │   └── teleop.launch.py          ← teleop launcher wrapper
        ├── swarm_drone/                  ← Python package (the actual nodes/logic)
        │   ├── __init__.py
        │   ├── coverage_planner.py       ← boustrophedon lawnmower path generator
        │   ├── drone_controller.py       ← motion controller abstraction (takeoff/move_to/land)
        │   ├── follower.py               ← per-drone mission state machine & follower node
        │   ├── leader.py                 ← swarm orchestration node & state machine
        │   ├── manual_control.py         ← interactive CLI teleop with leader override/resume logic
        │   ├── marker_manager.py         ← RViz MarkerArray visualizer
        │   ├── mission_cli.py            ← interactive terminal CLI for entering area & goals
        │   ├── region_allocator.py       ← grid-based region area division
        │   ├── swarm_config.py           ← YAML parameter parser & validator
        │   ├── task_manager.py           ← inter-drone communication & bookkeeping
        │   └── task_monitor.py           ← readable mission transcript logger
        ├── urdf/
        │   ├── robot.urdf.xacro ← top-level includes the three below
        │   ├── bot.xacro        ← physical body geometry
        │   ├── camera.xacro     ← RGB camera sensor
        │   └── control.xacro   ← Gazebo VelocityControl + OdometryPublisher
        ├── worlds/
        │   └── mapping_world.sdf
        ├── rviz/
        │   └── swarm.rviz
        ├── resource/
        │   └── swarm_drone
        ├── test/
        │   ├── test_coverage_planner.py  ← unit tests for coverage path generation
        │   ├── test_region_allocator.py  ← unit tests for equal-area region division
        │   ├── test_swarm_config.py      ← unit tests for YAML config loading/validation
        │   ├── test_copyright.py         ← ROS 2 ament_copyright linter test
        │   ├── test_flake8.py            ← ROS 2 ament_flake8 code style linter test
        │   └── test_pep257.py            ← ROS 2 ament_pep257 docstring linter test
```

---

## Architecture Stack (top → bottom)

```
Leader / Follower nodes         leader.py / follower.py
        ↓
Task allocation & comms         task_manager.py
        ↓
Region geometry                 region_allocator.py
        ↓
Coverage path geometry          coverage_planner.py
        ↓
Per-drone mission state machine DroneMission (in follower.py)
        ↓
Low-level control API           drone_controller.py
        ↓
Gazebo VelocityControl plugin + OdometryPublisher  (control.xacro)
        ↓
ros_gz_bridge                   cmd_vel ↔ Gazebo, odom ↔ ROS 2
```

`drone_controller.py` is the **only** file that knows how a drone is actuated. Swapping to MAVROS/PX4 means only changing `DroneController` internals and `control.xacro`.

---

## File-by-File Reference

---

### `config/swarm.yaml`

**Single source of truth for every parameter.** No node hard-codes these values.

| Section | Key | Default | Meaning |
|---|---|---|---|
| `swarm` | `num_drones` | 4 | Total drones in the swarm |
| `swarm` | `leader_id` | 0 | Which drone index is the leader |
| `swarm` | `ack_timeout_sec` | 10.0 | Seconds before leader logs an unresponsive follower |
| `swarm` | `heartbeat_timeout_sec` | 5.0 | Seconds of silence before a drone is considered lost |
| `mapping_area` | `min_x/max_x/min_y/max_y` | ±10.0 m | Rectangular coverage area |
| `takeoff` | `x/y/z` | 0/0/0 | Logical takeoff origin |
| `takeoff` | `flight_altitude` | 5.0 m | Cruise altitude |
| `takeoff` | `spawn_ring_spacing` | 2.0 m | Radius of stagger ring to avoid Gazebo spawn collisions |
| `takeoff` | `stagger_delay_sec` | 9.0 s | Per-drone-id delay before takeoff (avoids mid-air crossing) |
| `coverage` | `waypoint_spacing` | 1.0 m | Distance between lawnmower sweep rows |
| `coverage` | `boundary_margin` | 0.5 m | Inset from region edge |
| `coverage` | `cruise_speed` | 1.0 m/s | Linear speed between waypoints |
| `coverage` | `waypoint_tolerance` | 0.3 m | Arrival radius for a waypoint |
| `simulation` | `use_sim_time` | true | All nodes use `/clock` |
| `simulation` | `world_name` | `mapping_world` | SDF world name |
| `testing` | `enabled` | false | Testing mode flag |
| `testing` | `test_drone_id` | 0 | Default drone for teleop |

---

### `swarm_drone/swarm_config.py`

Loads and validates `swarm.yaml`. Exposes the `SwarmConfig` dataclass.

- `SwarmConfig.from_yaml_file(path)` — parses YAML, calls `validate()`, returns the dataclass.
- `SwarmConfig.validate()` — raises `ConfigError` for bad values (e.g. inverted area, zero spacing, out-of-range leader_id).
- `SwarmConfig.is_leader(drone_id)` — returns `True` if `drone_id == leader_id`.
- `SwarmConfig.area_width()` / `area_height()` — convenience.
- `default_config_path()` — resolves `<pkg_share>/config/swarm.yaml` via `ament_index`.

---

### `swarm_drone/region_allocator.py`

Pure geometry (no ROS/Gazebo dependency). Can be unit-tested directly.

**`Region` dataclass** (frozen):
- `drone_id`, `min_x`, `max_x`, `min_y`, `max_y`
- `.area()`, `.center()`, `.contains(x, y, margin=0.0)`

**`allocate_regions(num_drones, min_x, max_x, min_y, max_y)`**:
- Computes a grid shape as close to square as possible (`_grid_shape`).
- Grid formula: `rows = floor(sqrt(n))`, `cols = ceil(n/rows)`.
- Assigns regions row-major (drone 0 = top-left). If the last row is short, remaining cells are widened to fill full width.
- Examples: 4→2×2, 6→2×3, 9→3×3.
- Returns a list of `Region` objects.

---

### `swarm_drone/coverage_planner.py`

Pure geometry. Generates the lawnmower path for a single `Region`.

**`generate_coverage_path(region, waypoint_spacing, boundary_margin, altitude=0.0)`**:
- Insets by `boundary_margin` on all four sides.
- Sweeps along Y rows spaced `waypoint_spacing` apart.
- Alternates direction each row (left→right, right→left, …).
- Returns a list of `(x, y, z)` tuples. `z` is always `altitude`.

---

### `swarm_drone/drone_controller.py`

**Low-level per-drone control.** Only this file knows Gazebo's actuation interface.

**Topics (within the drone's namespace):**
- Publishes: `cmd_vel` (`geometry_msgs/Twist`) — velocity commands to Gazebo VelocityControl.
- Subscribes: `odom` (`nav_msgs/Odometry`) — pose feedback from Gazebo OdometryPublisher.

**Modes:** `idle | seek | hover | landed`

**API used by all upper layers:**
| Method | What it does |
|---|---|
| `takeoff(altitude, on_complete)` | Rise vertically to `altitude` from current (x,y) |
| `move_to(x, y, z, on_reached)` | Fly straight-line P-controller to (x,y,z) |
| `stop()` | Cancel target, hover in place (publish zero Twist) |
| `land(on_complete)` | Descend to z=0.15, then set mode idle |
| `current_position()` | Returns `(x,y,z)` or `None` before first odom |
| `has_odometry()` | `True` once first odom message received |
| `distance_to(x,y,z)` | Euclidean distance to a point |

**Control loop** (10 Hz timer): in `seek` mode, computes `dx/dy/dz`, publishes velocity scaled to `cruise_speed`. Declares arrival when `distance < waypoint_tolerance`, switches to `hover`, fires callback.

---

### `swarm_drone/follower.py`

Contains two things: the **`DroneMission` class** (used by both leader and follower) and the **`FollowerNode`** ROS 2 node.

#### `DroneState` enum
```
INITIALIZING → WAITING_FOR_TASK → TASK_ASSIGNED → TAKING_OFF
→ GOING_TO_REGION → COVERING → COMPLETED → LANDING → IDLE
```

#### `DroneMission`
Owns one drone's `DroneController` and runs its state machine.

- Constructs `DroneController`, creates `state` publisher (`/state` topic, relative to namespace).
- **`assign_region(region)`**: generates waypoints via `coverage_planner`, sets state to `TASK_ASSIGNED`. Ignores duplicate calls.
- **`_tick()`** (5 Hz): handles `TASK_ASSIGNED → TAKING_OFF` (with stagger delay = `drone_id × stagger_delay_sec`) and `COMPLETED → LANDING`.
- **`_publish_state()`** (2 Hz): publishes JSON to `state` topic with `drone_id`, `role`, `state`, `region_id`, `position`, `progress`.
- **`coverage_progress()`**: fraction of waypoints reached.

#### `FollowerNode`
- Parameters: `drone_id` (int, default 1), `config_path`.
- Subscribes to `/swarm/region_assignment` (global).
- On assignment message: parses JSON, finds the entry matching `self.mission.drone_id`, calls `mission.assign_region()`.

**Entry point:** `follower` console script → `main()`.

---

### `swarm_drone/leader.py`

Runs the **swarm orchestration state machine** on top of a `DroneMission` (leader is also an active drone) and `TaskManager`.

#### `LeaderState` enum
```
INITIALIZING → WAITING_FOR_SWARM → ALLOCATING_TASKS → WAITING_FOR_ACK
→ MISSION_RUNNING → MONITORING → MISSION_COMPLETE
```

#### `LeaderNode`
- Parameters: `config_path`.
- Creates: `DroneMission(self, config.leader_id, 'leader', config)` and `TaskManager(self, config)`.
- **`_tick()`** (0.5 Hz timer): drives the state machine + calls `_check_heartbeats()` + publishes mission state via `task_manager.publish_mission_state()`.

**State transitions:**

| State | Condition to advance |
|---|---|
| `WAITING_FOR_SWARM` | Leader's own mission is `WAITING_FOR_TASK` AND all follower states are known (not `None`) |
| `ALLOCATING_TASKS` | Calls `allocate_regions()`, stores in `TaskManager`, assigns own region to `DroneMission`, advances immediately to `WAITING_FOR_ACK` |
| `WAITING_FOR_ACK` | Republishes assignments every 1 s; advances when all drones leave `STILL_WAITING` states |
| `MISSION_RUNNING` | Advances immediately to `MONITORING` |
| `MONITORING` | Tracks all drones reaching `COMPLETED/LANDING/IDLE`; advances to `MISSION_COMPLETE` when all done |

- **`_check_heartbeats()`**: logs error if any follower's last `/state` message is older than `heartbeat_timeout_sec`. Logs recovery when it comes back.

**Entry point:** `leader` console script → `main()`.

---

### `swarm_drone/task_manager.py`

Leader-side pub/sub bookkeeping. All swarm-wide communication goes through here.

**Publishers:**
- `/swarm/region_assignment` (`std_msgs/String` JSON) — all assignments + flight altitude.
- `/swarm/mission_state` (`std_msgs/String`) — leader's `LeaderState` value.

**Subscribers:**
- `/drone_<i>/state` for every `i` in `0..num_drones-1` — parses JSON status, updates `_latest_status` and `_last_seen`.

**Key methods:**
- `state_of(drone_id)` — returns the `'state'` string from latest status, or `None`.
- `seconds_since_seen(drone_id)` — age of last status message.
- `set_assignments(regions)` — stores `{drone_id: Region}` dict.
- `publish_assignments(flight_altitude)` — serializes assignments to JSON and publishes.
- `publish_mission_state(state_name)` — publishes leader state string.

---

### `swarm_drone/task_monitor.py`

**Read-only observer node.** Never publishes anything. Provides a human-readable mission log.

- Subscribes: `/swarm/mission_state` and `/drone_<i>/state` for all drones.
- Logs (deduplicated): swarm initialization, drone→region assignments, per-drone state transitions, `SWARM MISSION COMPLETE`.

**Entry point:** `task_monitor` console script → `main()`.

---

### `swarm_drone/marker_manager.py`

Publishes `visualization_msgs/MarkerArray` on `/swarm/markers` at 2 Hz for RViz.

**Subscribes to:**
- `/swarm/region_assignment` — builds `Region` objects, pre-computes coverage waypoints locally (same `generate_coverage_path` call DroneMission uses).
- `/drone_<i>/state` for all drones — tracks positions, states, roles, progress.

**Marker types published (per drone):**
| Namespace | Marker type | What it shows |
|---|---|---|
| `region_boundary` | LINE_STRIP | Colored rectangle outline of assigned region |
| `region_label` | TEXT_VIEW_FACING | "Region N" at region center |
| `waypoints` | POINTS | All planned lawnmower waypoints as dots |
| `drone_position` | SPHERE | Live drone position (larger = leader) |
| `drone_label` | TEXT_VIEW_FACING | "D\<id\> [ROLE] \<STATE\> \<progress\>%" |

Color palette: 6-color list, assigned by `drone_id % 6`. `frame_id = 'map'`.

**Entry point:** `marker_manager` console script → `main()`.

---

## ROS 2 Topics Summary

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/swarm/region_assignment` | `std_msgs/String` (JSON) | leader → all | All region assignments + flight altitude |
| `/swarm/mission_state` | `std_msgs/String` | leader → all | Leader's `LeaderState` enum value |
| `/swarm/markers` | `visualization_msgs/MarkerArray` | marker_manager → RViz | Full swarm visualization |
| `/drone_<i>/state` | `std_msgs/String` (JSON) | each drone → leader/monitor/marker | id, role, state, region_id, position, progress |
| `/drone_<i>/cmd_vel` | `geometry_msgs/Twist` | DroneController → Gazebo | Linear velocity command |
| `/drone_<i>/odom` | `nav_msgs/Odometry` | Gazebo → DroneController | Pose + twist feedback |
| `/drone_<i>/camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS 2 | RGB camera frames (15 Hz) |
| `/drone_<i>/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → ROS 2 | Camera intrinsics |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS 2 | Sim time (via clock_bridge) |

JSON schema for `/drone_<i>/state`:
```json
{
  "drone_id": 0,
  "role": "leader",
  "state": "COVERING",
  "region_id": 0,
  "position": [x, y, z],
  "progress": 0.42
}
```

JSON schema for `/swarm/region_assignment`:
```json
{
  "flight_altitude": 5.0,
  "assignments": [
    {"drone_id": 0, "min_x": -10.0, "max_x": 0.0, "min_y": -10.0, "max_y": 0.0},
    ...
  ]
}
```

---

## Hardware / Simulation Model (URDF/XACRO)

### `urdf/robot.urdf.xacro`
Top-level entry point. Accepts `prefix` arg (e.g. `drone_0_`). Includes the three xacro files below.

### `urdf/bot.xacro` — Physical Body
Quadrotor in **Quad-X layout** (arms at 45°/135°/225°/315°). All geometry is primitives (box/cylinder), no meshes.

| Part | Geometry | Size | Mass |
|---|---|---|---|
| Body (`base_link`) | Box | 0.30 × 0.30 × 0.10 m | 1.2 kg |
| Each arm (×4) | Box | 0.22 × 0.03 × 0.015 m | 0.05 kg |
| Each motor housing (×4) | Cylinder | r=0.02 m, h=0.04 m | 0.03 kg |
| Each propeller disc (×4) | Cylinder | r=0.09 m, h=0.01 m | 0.01 kg |

Inertia tensors computed analytically (box: `m(y²+z²)/12`, cylinder: `mr²/2`). All arm/motor/prop joints are `fixed` relative to `base_link`.

Colors: body = dark grey `(0.15,0.15,0.18)`, arms = mid grey, motors = black, props = semi-transparent silver.

### `urdf/camera.xacro` — RGB Camera
- Link: `<prefix>camera_link` — 0.02 × 0.028 × 0.02 m box, mass 0.01 kg.
- Mounted at `(body_length/2 + 0.01, 0, -body_height/4)` — front-center, slightly below midplane.
- Optical frame: `<prefix>camera_optical_link` — rotated `(-π/2, 0, -π/2)` for standard camera convention (x-right, y-down, z-forward).
- Gazebo sensor: type `camera`, 640×480 R8G8B8, H-FOV 60° (1.047 rad), near=0.05 m, far=100 m, 15 Hz.
- Topic in Gazebo: `<prefix>camera/image_raw`.

### `urdf/control.xacro` — Gazebo Control Interface
Two Ignition/gz-sim plugins attached to each drone model:

1. **`ignition-gazebo-velocity-control-system`** (`gz::sim::systems::VelocityControl`): accepts `cmd_vel` (`geometry_msgs/Twist`) and moves the body kinematically — **no rotor thrust physics**. This is why DroneController's P-controller works.
2. **`ignition-gazebo-odometry-publisher-system`** (`gz::sim::systems::OdometryPublisher`): publishes pose + twist to `<prefix>odom` frame at 30 Hz. `odom_frame = <prefix>odom`, `robot_base_frame = <prefix>base_link`, `dimensions = 3` (3D odometry).

---

## Gazebo World — `worlds/mapping_world.sdf`

World name: `mapping_world`. SDF version 1.6.

**Physics:** `max_step_size = 0.004 s` (250 Hz), `real_time_factor = 1.0`.

**Plugins loaded:**
- `ignition-gazebo-physics-system` — rigid body physics.
- `ignition-gazebo-user-commands-system` — allows `ros2 run ros_gz_sim create` to spawn models at runtime.
- `ignition-gazebo-scene-broadcaster-system` — scene updates to Gazebo GUI.
- `ignition-gazebo-sensors-system` with `ogre2` render engine — needed for camera sensors.

**Static models:**
- `ground_plane`: 40×40 m flat plane (green-grey), with collision.
- `boundary_north/south/east/west`: four thin red box walls at ±10 m edges, **visual only (no collision)**. They mark the default `mapping_area` boundaries but don't block drones if the area is changed in YAML.

**Lighting:** single directional sun at `(0,0,10)`, direction `(-0.5, 0.1, -0.9)`.

---

## Launch Files

### `launch/simulation.launch.py` — Terminal 1
Starts Gazebo and spawns all drones. Must run first.

1. Launches `ign gazebo -r mapping_world.sdf` with `IGN_IP=127.0.0.1` and `LIBGL_ALWAYS_SOFTWARE=1`.
2. Starts a `clock_bridge` node: bridges `/world/mapping_world/clock` → `/clock`.
3. For each drone `i` in `0..num_drones-1`:
   - **Spawn position**: on a ring of radius `spawn_ring_spacing` at angle `2π*i/num_drones` around the takeoff origin (single drone spawns exactly at origin). Z offset = `base_z + 0.3`.
   - **`robot_state_publisher`** in namespace `drone_<i>` with the xacro-processed URDF (prefix = `drone_<i>_`).
   - **`ros_gz_bridge`** per drone: bridges `cmd_vel`, `odom`, `camera/image_raw`, `camera/camera_info`.
   - **`static_transform_publisher`**: publishes identity TF from `map` → `<prefix>odom` (since OdometryPublisher reports world-frame poses, these frames coincide).
   - All spawns staggered by `i × 3.0 s` (starting at t=2 s) to give Gazebo physics time to settle each body before the next.

### `launch/auto.launch.py` — Terminal 2
Starts the autonomous mission logic. Run after simulation is up.

- One **`leader`** node in namespace `drone_<leader_id>`.
- One **`follower`** node per non-leader drone, each in namespace `drone_<i>` with `drone_id=i`.
- One **`marker_manager`** node (no namespace).
- One **`task_monitor`** node (no namespace).
- All nodes get `config_path` and `use_sim_time` parameters.

### `launch/teleop.launch.py` — Optional
Cannot actually run `teleop_twist_keyboard` through `ros2 launch` (no real TTY on stdin). Instead, **prints** the correct `ros2 run` command for the selected drone and exits.

- `drone_id` launch argument (default: `testing.test_drone_id` from YAML).
- Printed command: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/drone_<id>/cmd_vel`

---

## RViz Configuration — `rviz/swarm.rviz`

Fixed frame: `map`. Window: 1400×900.

| Display | Topic / Source | Notes |
|---|---|---|
| Grid | — | 30×30 XY grid, 1 m cells |
| TF | — | All frames, arrows + axes + names |
| SwarmMarkers (MarkerArray) | `/swarm/markers` | All swarm visualization |
| Drone0Model (RobotModel) | `/drone_0/robot_description` | URDF model of drone 0 |
| Drone0Camera (Image) | `/drone_0/camera/image_raw` | Disabled by default |

View: Orbit, distance=30, focal=(0,0,0), pitch=0.6, yaw=0.78 (isometric-ish top-down).

---

## Package Metadata

- **Name:** `swarm_drone`
- **Build type:** `ament_python`
- **Maintainer:** Yashika Aggarwal (yashikaagg3@gmail.com)
- **License:** Apache-2.0
- **ROS version:** Humble
- **Gazebo version:** Ignition Fortress (gz-sim 6)

**ROS dependencies** (from `package.xml`):
`rclpy`, `std_msgs`, `geometry_msgs`, `visualization_msgs`, `xacro`, `robot_state_publisher`, `joint_state_publisher`, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`, `launch`, `launch_ros`, `tf2_ros`, `teleop_twist_keyboard`, `python3-yaml`

**Console scripts** (from `setup.py`):
| Script | Entry point |
|---|---|
| `leader` | `swarm_drone.leader:main` |
| `follower` | `swarm_drone.follower:main` |
| `marker_manager` | `swarm_drone.marker_manager:main` |
| `task_monitor` | `swarm_drone.task_monitor:main` |

---

## State Machine Reference

### Per-drone (`DroneState` in `follower.py`)
```
INITIALIZING
  → WAITING_FOR_TASK      (immediately, nothing to calibrate)
  → TASK_ASSIGNED         (on assign_region())
  → TAKING_OFF            (after stagger delay = drone_id × stagger_delay_sec)
  → GOING_TO_REGION       (first waypoint navigation)
  → COVERING              (waypoint 2+)
  → COMPLETED             (all waypoints reached)
  → LANDING               (descend to z=0.15)
  → IDLE                  (landed)
```

### Leader swarm orchestration (`LeaderState` in `leader.py`)
```
INITIALIZING
  → WAITING_FOR_SWARM     (waiting for own WAITING_FOR_TASK + all followers visible)
  → ALLOCATING_TASKS      (calls region_allocator, assigns own region)
  → WAITING_FOR_ACK       (republishes assignments every 1s until all ack)
  → MISSION_RUNNING       (all acknowledged; transitions next tick)
  → MONITORING            (watches for all COMPLETED/LANDING/IDLE)
  → MISSION_COMPLETE
```

---

## How to Run

```bash
# Build (once)
cd ~/swarm_drone/Final-swarm-drone
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Terminal 1 — Gazebo + drones
ros2 launch swarm_drone simulation.launch.py

# Terminal 2 — Autonomous mission
ros2 launch swarm_drone auto.launch.py

# Terminal 3 — RViz (optional)
rviz2 -d $(ros2 pkg prefix swarm_drone)/share/swarm_drone/rviz/swarm.rviz

# Manual teleop (run the printed command in a 4th terminal)
ros2 launch swarm_drone teleop.launch.py drone_id:=1
```

## Key Design Decisions

1. **No PX4/MAVROS yet** — `DroneController` uses Gazebo's `VelocityControl` (kinematic, no rotor physics). Swap only `DroneController` + `control.xacro` for MAVROS later.
2. **No custom `.msg` types** — all inter-node data is `std_msgs/String` with JSON payloads. Keeps the package pure `ament_python` (no mixed `ament_cmake` interface package).
3. **Leader is also an active drone** — `LeaderNode` contains a `DroneMission` instance and runs its own coverage. Logic is never duplicated.
4. **Staggered spawn + staggered takeoff** — spawn ring avoids Gazebo physics collisions at spawn time; `stagger_delay_sec` avoids mid-air crossing during transit to regions.
5. **ACK handshake** — leader republishes assignments until every follower acknowledges (moves out of `WAITING_FOR_TASK`). Handles message loss.
6. **Pure-geometry modules** — `region_allocator.py` and `coverage_planner.py` have no ROS/Gazebo imports and can be unit-tested without a running ROS environment.
