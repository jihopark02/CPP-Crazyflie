# cf231 Coverage Guidance

Coverage-path guidance for a Bitcraze Crazyflie (cf231), tracked with an OptiTrack/NatNet
motion capture system, validated both on real hardware and in Gazebo simulation.

## Structure

- **`real_flight/`** — Standalone Python scripts that fly the real Crazyflie directly via
  [cflib](https://github.com/bitcraze/crazyflie-lib-python) and a vendored NatNet client
  (no ROS2 in the loop). Position/orientation comes from Motive over NatNet and is fed into
  the onboard EKF via `extpos`.
  - `direct_cflib_fly.py` — basic takeoff/hover sanity check
  - `direct_cflib_waypoints.py` — PID waypoint-to-waypoint navigation
  - `direct_cflib_square.py`, `direct_cflib_l1_square.py` — rectangle path via classic L1 guidance
  - `direct_cflib_coverage.py` — L1 guidance over a full boustrophedon coverage path
  - `direct_cflib_coverage_refined.py` / `direct_cflib_coverage_refined2.py` — improved guidance:
    projects onto path *segments* (not just nearest point) and adds an explicit cross-track
    P-correction term, converted from world-frame to the drone's body-frame velocity setpoint
  - `plot_flight_error.py` — reads a flight log CSV and plots cross-track / altitude error vs time
  - `paths/` — coverage paths (CSV + original Excel source), including obstacle-avoidance variants

- **`simulation/`** — Gazebo (ros_gz) validation of the same guidance approach.
  - `coverage_guidance_sim_live.py` — original classic-L1 sim harness
  - `coverage_guidance_sim_refined.py` — same segment-projection + cross-track-correction
    guidance as the real-flight refined2 script, ported to ROS2 `/cmd_vel` (Twist), **without**
    the corner-deceleration feature
  - `plot_sim_error.py` — error analysis for sim flight logs
  - `worlds/crazyflie_world.sdf` — Gazebo world with 3 box obstacles for obstacle-avoidance testing

- **`results/`** — Per-scenario outputs from the simulation runs (`obs0`: no obstacles,
  `obs1`: 1 obstacle, `obs3`: 3 obstacles), each containing the flight log (CSV + Excel), the
  flown trajectory re-exported in the same format as the planned path, a trajectory plot
  (planned path vs obstacles vs flown trail), and a cross-track/altitude error plot.

## Simulation setup

The Gazebo side depends on a third-party ROS2/Gazebo bridge package (not part of this repo):

```bash
cd ~/ros2_ws/src
git clone https://github.com/knmcguire/ros_gz_crazyflie
cd ~/ros2_ws && colcon build --symlink-install
```

Copy `simulation/worlds/crazyflie_world.sdf` from this repo over the package's own
`ros_gz_crazyflie_gazebo/share/ros_gz_crazyflie_gazebo/worlds/crazyflie_world.sdf` (installed
copy) to get the resized room and the 3 obstacle boxes used in the `obs1`/`obs3` runs.

The package's launch file loads `model://crazyflie` without setting the Gazebo model search
path, which fails out of the box (`Unable to find uri[model://crazyflie]`). Work around it by
exporting the resource path before launching:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="$HOME/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models:$GZ_SIM_RESOURCE_PATH"
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py
```

Then, in another terminal:

```bash
python3 simulation/coverage_guidance_sim_refined.py <path_csv> <log_csv> [plot_png] [ox,oy,size ...]
```

## Guidance approach

Both the real-flight and simulation guidance controllers:
1. Project the vehicle's current position onto the nearest *segment* of the planned path
   (not just the nearest sampled point), tracking a monotonically increasing arc-length
   progress `s` so the tracker never jumps backward at sharp turns.
2. Command a velocity that combines a constant along-path speed with a proportional
   cross-track correction term (capped to a max lateral speed), converted from world frame
   to the vehicle's body frame.
3. (Real-flight refined2 only) Automatically slow down when the upcoming path direction
   changes sharply (corners); the simulation port omits this for a simpler baseline.

## Validation results (simulation)

| Scenario | Obstacles | Result | Mean cross-track error |
|---|---|---|---|
| obs0 | none | completed | ~0.8 cm |
| obs1 | 1 | completed, min clearance ~22 cm | ~1.0 cm |
| obs3 | 3 | completed, min clearance ~22 cm | ~1.1 cm |

On real hardware, the same refined2 guidance completed a 3m x 2m rectangle and multiple
2.5cm-resolution coverage paths cleanly, with cross-track error on the order of a few
centimeters and no safety cutoffs triggered by tilt or position error (battery voltage was
the only real-world failure mode encountered).
