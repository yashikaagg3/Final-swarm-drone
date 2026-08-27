# swarm_drone

A ROS 2 Jazzy + Gazebo Harmonic simulation of a drone swarm
performing autonomous, leader-coordinated area coverage. One drone is
the leader (also an active drone), the rest are followers; the leader
divides the mapping area into roughly equal regions, assigns one to
each drone over real ROS 2 topics, and every drone flies a lawnmower
coverage pattern over its own region.

This is a first version deliberately built **without** PX4/MAVROS - see
"Architecture" below for how the control stack is layered so that can
be swapped in later without touching swarm logic.

## Project purpose

- 4 drones by default (1 leader + 3 followers), configurable via YAML.
- Leader-based task allocation: equal-area region division, assignment,
  and an acknowledgement handshake (the leader never assumes a follower
  received its assignment).
- Autonomous lawnmower coverage of each drone's assigned region.
- `visualization_msgs/MarkerArray` showing drone ids, roles, region
  boundaries, region ids, positions and coverage waypoints.
- Individual drone testing by id.

## Architecture

```
Leader / Followers          (leader.py, follower.py)
        |
        v
Task Allocation              (task_manager.py - pub/sub + ack bookkeeping)
        |
        v
Region Allocation            (region_allocator.py - pure geometry)
        |
        v
Coverage Planner             (coverage_planner.py - pure geometry)
        |
        v
Waypoint / mission logic     (follower.py: DroneMission state machine)
        |
        v
Drone Controller              (drone_controller.py - takeoff/move_to/stop/land)
        |
        v
Gazebo (VelocityControl + OdometryPublisher plugins, via ros_gz_bridge)
```

`drone_controller.py` is the **only** file that knows how a drone is
actually actuated. Everything above it talks to `takeoff()`,
`move_to(x, y, z)`, `stop()` and `land()` only. To add PX4/MAVROS
later, replace what's inside `DroneController` (and the plugins in
`urdf/control.xacro`) with a MAVROS-based implementation - the leader,
follower, task allocation, region allocation, coverage planner and
MarkerArray/monitor code do not need to change.

Region assignment and drone state are communicated with plain
`std_msgs/String` JSON payloads rather than custom `.msg` types, so the
whole package stays a single `ament_python` package (no mixed
`ament_cmake` interface package needed) - see `task_manager.py` and
`follower.py` for the schema.

### Topics

- `/swarm/region_assignment` (leader -> all): all region assignments + flight altitude, republished until every drone acknowledges.
- `/swarm/mission_state` (leader -> all): the leader's own state machine state.
- `/swarm/markers` (marker_manager -> RViz): `visualization_msgs/MarkerArray`.
- `/drone_<i>/state` (each drone -> leader, marker_manager, task_monitor): id, role, state, assigned region id, position, coverage progress.
- `/drone_<i>/cmd_vel`, `/drone_<i>/odom`: DroneController's actuation/feedback (bridged to Gazebo).
- `/drone_<i>/camera/image_raw`, `/drone_<i>/camera/camera_info`: the drone's RGB camera.

### State machines

Drone (leader and follower both run this, from `follower.py`):
`INITIALIZING -> WAITING_FOR_TASK -> TASK_ASSIGNED -> TAKING_OFF ->
GOING_TO_REGION -> COVERING -> COMPLETED -> LANDING -> IDLE`

Leader (`leader.py`), on top of its own drone state machine:
`INITIALIZING -> WAITING_FOR_SWARM -> ALLOCATING_TASKS ->
WAITING_FOR_ACK -> MISSION_RUNNING -> MONITORING -> MISSION_COMPLETE`

## Environment this was built/verified against

- Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12.
- Gazebo: **Gazebo Harmonic**, via `gz sim`. This is the branch (`ros2-jazzy`)
  of the project ported from the original Humble/Gazebo Fortress build
  (see `main`) - the plugin names, environment variables, and
  `ros_gz_bridge` message-type strings below are all Harmonic's, not
  Fortress's, and are **not** interchangeable between the two branches.
- Control: `gz-sim-velocity-control-system` (kinematic body velocity
  control, no rotor thrust physics) + `gz-sim-odometry-publisher-system`.
  World plugins (`worlds/mapping_world.sdf`) use the same `gz-sim-*`
  naming: `gz-sim-physics-system`, `gz-sim-user-commands-system`,
  `gz-sim-scene-broadcaster-system`, `gz-sim-sensors-system`.
- `ros_gz_bridge` arguments reference `gz.msgs.*` types (e.g.
  `gz.msgs.Twist`), not Fortress's `ignition.msgs.*` - and `GZ_IP` is
  used instead of `IGN_IP`, matching Harmonic's env var naming.

## Dependencies

```
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard python3-yaml python3-pytest
```

## Installation & building

This repo itself is the colcon workspace root (the package lives at
`src/swarm_drone` inside it) - no extra wrapper folder needed. Clone it
wherever you like; the examples below assume `~/swarm_drone`, adjust the
path if you put it somewhere else.

```bash
git clone <this-repo-url> ~/swarm_drone
cd ~/swarm_drone
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Tip: the `source` lines above are only needed once per terminal because
sourcing sets environment variables (`PATH`, `PYTHONPATH`,
`AMENT_PREFIX_PATH`, ...) that don't persist across shells - every new
terminal starts without them. To skip typing them in every new terminal,
add both to `~/.bashrc` (adjust the path if you cloned somewhere other
than `~/swarm_drone`):

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo '[ -f ~/swarm_drone/install/setup.bash ] && source ~/swarm_drone/install/setup.bash' >> ~/.bashrc
```

This is a local shell setting, not part of the repo, so it only affects
your own machine - it won't change what anyone else needs to run after
cloning this repo fresh.

## Running

**Terminal 1 - Gazebo + drones:**

```bash
source ~/swarm_drone/install/setup.bash
ros2 launch swarm_drone simulation.launch.py
```

This starts Gazebo, loads `worlds/mapping_world.sdf`, and spawns
`swarm.num_drones` drones (config/swarm.yaml) into `/drone_0`..`/drone_<n-1>`.

If Gazebo's window never renders / topics never publish and you see
`libEGL: failed to create dri2 screen` in the log, your GL/EGL stack
can't create a hardware context (common in some sandboxes/VMs/CI). Fix:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch swarm_drone simulation.launch.py
```

**Terminal 2 - autonomous swarm mission:**

```bash
source ~/swarm_drone/install/setup.bash
ros2 launch swarm_drone auto.launch.py
```

Starts the leader, one follower per non-leader drone, `marker_manager`,
and `task_monitor`. Watch terminal 2 (or just `task_monitor`'s output)
for the mission transcript: initialization, ready handshake, region
assignment + acks, takeoff, coverage, and `SWARM MISSION COMPLETE`.

**Terminal 3 - RViz (optional):**

```bash
source ~/swarm_drone/install/setup.bash
rviz2 -d $(ros2 pkg prefix swarm_drone)/share/swarm_drone/rviz/swarm.rviz
```

Start this only after both Terminal 1 (Gazebo) and Terminal 2
(`auto.launch.py`) are already running, since RViz just visualizes their
topics. Verified working: you should see, per drone -

- A colored sphere at its live position (bigger sphere = the leader),
  driven directly by `/drone_<i>/state`'s reported position - so it
  tracks the drone's real position even if you fly it manually with
  `teleop.launch.py` and take it outside its own region.
- A text label above it: `D<id> [ROLE] <STATE> <progress>%`.
- Its region boundary as a colored rectangle outline, with a `Region
  <id>` label at its center - 4 quadrants for the default 4-drone/2x2
  grid.
- Its planned lawnmower waypoints as small dots filling that rectangle.
- `drone_0`'s robot model + TF frame (RViz is configured to show
  `/drone_0/robot_description`).

All of the above comes from `/swarm/markers` (published by
`marker_manager`) except the robot model/TF, which comes straight from
`robot_state_publisher`/Gazebo.

## Testing an individual drone

Two ways:

1. **Unit-test the algorithms** (no Gazebo needed):
   ```bash
   cd ~/swarm_drone
   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select swarm_drone
   colcon test-result --verbose
   ```
   (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` works around a `pytest`/`anyio`
   plugin conflict some environments have installed; harmless to
   always include.)

2. **Run one drone's mission node standalone** against a running
   `simulation.launch.py`, to inspect its id/role/region/position/state
   directly:
   ```bash
   ros2 run swarm_drone follower --ros-args -r __ns:=/drone_2 -p drone_id:=2
   # drone_0 (the default leader_id) instead:
   ros2 run swarm_drone leader --ros-args -r __ns:=/drone_0
   ros2 topic echo /drone_2/state
   ```
   Or fly a drone manually with the keyboard. `ros2 launch` can't host
   an interactive keyboard tool (it doesn't give child processes a real
   terminal on stdin, so `teleop_twist_keyboard` fails with
   `termios.error: Inappropriate ioctl for device` if launched that
   way) - `ros2 launch swarm_drone teleop.launch.py drone_id:=2` prints
   the exact command to run instead:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/drone_2/cmd_vel
   # i/j/k/l/,/u/o to move, q/z to change speed, k to stop
   ```

## Changing the number of drones

Edit `config/swarm.yaml`:

```yaml
swarm:
  num_drones: 6
  leader_id: 0
```

`region_allocator.py` picks a grid as close to square as possible for
any `num_drones` (4 -> 2x2, 6 -> 2x3, 8 -> 2x4, 9 -> 3x3, ...) - nothing
else needs to change. Re-run `simulation.launch.py` / `auto.launch.py`.

## Changing the mapping area / coverage spacing

Also in `config/swarm.yaml`:

```yaml
mapping_area:
  min_x: -10.0
  max_x: 10.0
  min_y: -10.0
  max_y: 10.0

coverage:
  waypoint_spacing: 1.0   # distance between lawnmower sweep rows
  boundary_margin: 0.5    # how far inside its region a drone stays
```

Note: `worlds/mapping_world.sdf`'s four red boundary-marker walls are a
**visual-only** reference sized for the default ±10 area; they have no
collision, so enlarging `mapping_area` is safe and just won't visually
line up with those markers anymore.

## Troubleshooting

- **RViz opens but shows nothing at all (empty Displays, no drones)**:
  this isn't an RViz problem - RViz never caches anything, it only shows
  what's live right now. It means `simulation.launch.py` and/or
  `auto.launch.py` aren't actually running in another terminal (not
  started yet, closed, or crashed). Confirm with `ros2 topic list` -
  if you don't see `/swarm/markers` and `/drone_0/state` in the output,
  start (or restart) those two launches first, then open RViz.
- **Nothing moves / all topics silent, no errors**: check for
  `libEGL`/DRI2 failures in the Gazebo log - see `LIBGL_ALWAYS_SOFTWARE=1`
  above. This failure mode is silent: Gazebo's whole step loop stalls,
  not just camera rendering.
- **Two Gazebo processes running / duplicate `/drone_0` topics with
  weird jumpy positions**: you have two `simulation.launch.py` instances
  up at once. `pkill -f "ign gazebo"` and relaunch just one.
- **`Drone <n> has not acknowledged its region assignment`**: that
  follower node isn't running, crashed, or its `drone_id` parameter is
  wrong - check `ros2 node list` and that `follower --ros-args -p
  drone_id:=<n>` matches its namespace.
- **`Drone <n> unresponsive: no state update for ...`**: that drone's
  `/drone_<n>/state` topic has gone quiet (node crashed, or Gazebo
  bridge died) - check that node's terminal output.
- **A drone sits on the ground at z≈0 forever, state stuck at
  `TAKING_OFF`/`GOING_TO_REGION`, and even a raw `cmd_vel` published
  directly to it does nothing**: seen once on this machine when several
  drones were spawned in very quick succession under software
  rendering - the `VelocityControl` plugin apparently didn't finish
  attaching before flight commands started. `simulation.launch.py`'s
  `spawn_stagger_sec` (currently 3.0s) exists specifically to give each
  spawn time to settle before the next one starts; if you still see
  this, increase it further and/or increase `takeoff.spawn_ring_spacing`
  in `config/swarm.yaml`.
- **colcon test flake8/pep257 failures from an unrelated plugin**: if
  you see `ModuleNotFoundError: No module named '_pytest.scope'`, run
  with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` (a user-installed `anyio`
  pytest plugin expects a newer pytest than ROS's system one).
