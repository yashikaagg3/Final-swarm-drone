# Simulation to hardware (PX4 + MAVROS)

This is the report of the PX4/MAVROS path: what changed, what did **not**
change, how to run it, and how a real Pixhawk later replaces SITL.

## What this pass did

The swarm still has **one** launch file and still waits for **you** to
dispatch a mission:

```bash
ros2 launch swarm_drone swarm_launch.launch.py
ros2 run swarm_drone mission_cli
```

`simulation.backend` in `src/swarm_drone/config/swarm.yaml` selects the
vehicle:

| Value | Vehicle | Default? |
|---|---|---|
| `mavros` | PX4 SITL (x500) + MAVROS position setpoints | yes |
| `gazebo` | Ignition Fortress + VelocityControl `cmd_vel` (original demo) | no |

Default is `mavros`. PX4 SITL and `ros-humble-mavros` must be installed
(see below). Use `backend: gazebo` only for the fast kinematic demo.

## What did not change

- `mission_cli` prompts and the `/swarm/goal` JSON rectangle
  (`min_x/max_x/min_y/max_y`, altitude, spacing, speed)
- Leader `auto_start:=false` (drones idle until that goal)
- `region_allocator.py` grid split
- `coverage_planner.py` lawnmower
- `marker_manager`, `task_monitor`, RViz launch
- Kinematic URDF + `mapping_world.sdf` when backend is `gazebo`

Unknown keys on `/swarm/goal` are already ignored (`dict.get`). A future
web UI can add `polygon: [[x, y], ...]` without MAVROS knowing about it.

## Architecture

```
mission_cli  (or future web UI)
    → /swarm/goal
leader / followers / coverage planner     unchanged
    → takeoff() / move_to() / stop() / land()
create_drone_controller()
    → gazebo: Twist on cmd_vel
    → mavros: PoseStamped on /drone_i/mavros/setpoint_position/local
         → MAVROS → MAVLink → PX4 (SITL UDP or Pixhawk UART)
```

You do not send `cmd_vel` to a Pixhawk. `move_to(x, y, z)` **is** the
command. The MAVROS backend streams that pose at ≥20 Hz. PX4 runs the
position controller, EKF (IMU, baro, mag, GPS), arming, and OFFBOARD.

Arming = motors allowed to spin. OFFBOARD = PX4 will track companion
setpoints. Both are handled inside `MavrosDroneController.takeoff()`,
not in the leader.

## Topic map (mavros backend)

Per drop `i`, namespace `/drone_<i>/mavros/`:

| Direction | Topic / service | Role |
|---|---|---|
| pub | `setpoint_position/local` (`PoseStamped`, ENU) | fly / hover |
| sub | `local_position/pose` | EKF pose (replaces gazebo `odom`) |
| sub | `state` | connected, armed, mode |
| srv | `set_mode` (`OFFBOARD`) | allow companion setpoints |
| srv | `cmd/arming` | arm / disarm |
| srv | `cmd/land` | land |

MAVLink sysid is `i+1`. SITL FCU URL is `udp://:1454{i}@127.0.0.1:1458{i}`
via `simulation.mavros.fcu_url_pattern`.

Swarm topics are unchanged: `/swarm/goal`, `/swarm/region_assignment`,
`/swarm/markers`, `/drone_<i>/state`.

## Running PX4 SITL

1. Install MAVROS and GeographicLib datasets:

   ```bash
   sudo apt install ros-humble-mavros ros-humble-mavros-extras
   sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
   ```

2. Clone and build PX4 SITL (once):

   ```bash
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
   cd ~/PX4-Autopilot
   make px4_sitl
   ```

   Or set `simulation.mavros.px4_dir` / `PX4_AUTOPILOT_DIR` to that tree.

3. In `config/swarm.yaml`:

   ```yaml
   simulation:
     backend: mavros
     use_sim_time: true
     wind:
       speed_m_s: 1.5
       direction_deg: 0.0
       gusts: false
   ```

4. Same two commands as the gazebo demo: `swarm_launch.launch.py`, then
   `mission_cli`.

If PX4 is missing, the mavros branch raises a clear error pointing here.
The gazebo branch never looks for PX4.

## Switching to a real Pixhawk

On each companion computer (one drop):

```yaml
simulation:
  backend: mavros
  use_sim_time: false
  mavros:
    start_sitl: false
    fcu_url_pattern: "/dev/ttyUSB0:921600"
```

UART device and baud must match the Pixhawk TELEM2 / USB link. Sysid
still comes from drone id (`drone_0` → sysid 1). Flash the same PX4
firmware family you used in SITL. Do not publish `cmd_vel` at the
Pixhawk.

That is the sim-to-hardware switch: connection URL + `start_sitl` +
`use_sim_time`. Mission CLI, regions, and coverage stay the same.

## Wind

Wind is applied **only** on the PX4 world (`mapping_world_px4.sdf`),
from `simulation.wind`. The kinematic gazebo world is unchanged.

PX4’s position controller leans into wind and holds the setpoint, in
SITL and on hardware, until wind exceeds thrust / max tilt. SITL wind is
smoother than a field; GPS and vibration will still differ outdoors.
Software (setpoints, arming, OFFBOARD timeout) transfers; you may still
retune `MPC_*` limits on the real airframe.

`speed_m_s: 0` disables wind.

## Geofence and future web UI (not built this pass)

Today the mission area is an axis-aligned rectangle from `mission_cli`
→ `/swarm/goal`. Coverage waypoints already stay inside
`coverage.boundary_margin`.

Later:

1. A web page draws a polygon (or rectangle).
2. It publishes `/swarm/goal` (same topic as the CLI), including
   `polygon: [[x, y], ...]`.
3. The leader allocates sub-regions inside that shape.
4. That same polygon is uploaded to PX4 as a geofence (`GF_*`) so the
   Pixhawk rejects leaving the drawn area even if OFFBOARD setpoints are
   wrong.

The flight stack must not be driven by the UI directly. CLI remains
valid.

## Files touched

| File | Change |
|---|---|
| `config/swarm.yaml` | `backend`, `wind`, `mavros` block; default is mavros |
| `swarm_config.py` | parse/validate those fields; `mavros_fcu_url()` |
| `drone_controller.py` | `GazeboDroneController` + `create_drone_controller()` |
| `mavros_controller.py` | new: 20 Hz pose setpoints, OFFBOARD, arm, land |
| `follower.py` | factory; stay `INITIALIZING` until pose exists |
| `manual_control.py` | `cmd_vel` vs MAVROS velocity by backend |
| `launch/swarm_launch.launch.py` | branch on backend; same swarm nodes |
| `worlds/mapping_world_px4.sdf` | PX4 world + bounds + wind placeholders |
| `package.xml` | exec depend on mavros (runtime, mavros branch) |

`mission_cli.py`, `region_allocator.py`, and `coverage_planner.py` were
not modified.
