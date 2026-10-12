# kinova_dual_arm_dmp_insertion

Two Kinova arms insert ONE tetromino into a narrowed shelf slot (the narrow75
worlds: the most intruding neighbour on each flank 15 mm inside the piece's
silhouette) along the DMP route trained for that slot in PyBullet. The squeeze
is the SOCP's (friction cones as hard constraints, TP-GMM human prior in the
cost), closed on the wrist F/T. **Only the insertion** - no feeder, no vision,
no stand-up, no Dellacherie: the piece is presented by hand (or a fixture) above
its slot and the shelf is built by hand from the exported board.

A self-contained ROS 2 (ament_python) package: copy this folder into
`~/ros2_ws/src/`. It needs numpy, scipy, cvxpy, pyyaml, rclpy, tf2_ros - no
PyBullet, no sympy, no Yang Lin's compiled worlds. The routes are data files.

## What the robot is sent, and what is read

| out, every 10 ms | in, every 10 ms |
|---|---|
| a target pose for each flange (`geometry_msgs/PoseStamped`, arm base frame) | each wrist's F/T (`geometry_msgs/WrenchStamped`) |
| (optional) the tilt stiffness in use (`Float64MultiArray`) | each end-effector pose (TF) |

Inside, exactly as the trainer drives it (`38_learn_pybullet.py drive_hands`):
1. **one object frame, two hands**: the route gives the PIECE pose; both flange
   targets follow from it rigidly;
2. **one scalar on top**, `ds`, the extra close along the grasp axis, integrated
   on the MEASURED normal force towards the SOCP's target (re-solved every
   100 ms, updated every 20 ms);
3. the **jam sentry** (unexpected side force over 3 x the spare friction), the
   **slow straight seat** (30 mm/s), the **stop on a high landing**, the **seat by
   feel** (the shelf carries 60% of the weight with the piece believed within
   40 mm of its seat), the **release**.

The **impedance is the arm controller's job**: the simulated wrist was a spring,
1500 N/m, 40 Nm/rad about the push axis, 3 (approach) / 8 (hold) Nm/rad tilt, and
the routes were trained against it. Use a Cartesian impedance / compliance
controller that takes a target pose (e.g. FZI `cartesian_compliance_controller`
on `ros2_kortex`, or a joint-torque impedance on the Kortex low-level API).
Commanding the full wrist wrench instead was measured worse (CLAUDE.md §9).

## Install (on the ROS PC: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12)

```
source /opt/ros/jazzy/setup.bash
sudo apt install python3-numpy python3-scipy python3-yaml ros-jazzy-tf2-ros
# Ubuntu 24.04 blocks system pip (PEP 668). numpy MUST stay < 2: rclpy is built
# against Jazzy's numpy 1.26, and cvxpy would otherwise pull numpy 2.
pip install --user --break-system-packages "numpy<2" "cvxpy<1.7" clarabel
mkdir -p ~/ros2_ws/src && cp -r <this folder> ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select kinova_dual_arm_dmp_insertion && source install/setup.bash
```

Verified on the simulation PC with Jazzy's versions (numpy 1.26.4, scipy 1.11.4,
cvxpy 1.6.7, Python 3.12): `check` gives the same result as with numpy 2.

Discovery: with `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` this node, the two arm
drivers and their controllers must share the subnet and the `ROS_DOMAIN_ID`.
Check that the topics in `config/cell.yaml` are visible before `insert`:
`ros2 topic list`, `ros2 topic hz /left/ft_sensor/wrench`, `ros2 run tf2_ros
tf2_echo left_base_link left_end_effector_link`.

Timing: one SOCP solve takes ~25 ms and runs every 100 ms inside the 100 Hz
loop, so that tick is late; the loop does not try to catch up (measured in the
dry run: 138 solves in 3.8 s).

## Run

```
ros2 run kinova_dual_arm_dmp_insertion list                       # the routes, with their PyBullet result
ros2 run kinova_dual_arm_dmp_insertion check  L_s7040m6           # NO robot: dry run + reach + path vs PyBullet
ros2 run kinova_dual_arm_dmp_insertion insert L_s7040m6 --config ~/cell.yaml
```

`insert` asks twice on the keyboard: when to present the piece between the
pads, and when to let it go. Ctrl-C or a safety stop (`f_abort`, `track_abort`)
stops streaming; the arms hold their last target. Every tick is logged to
`logs/<world>_<arm>_<time>.csv`. **The placement is measured by hand** (worst
cell against its target, as the trainer scores it): nothing here sees the piece.

`--arm scalar` (default): the squeeze alone, the thesis design, 24/29 worlds in
PyBullet. `--arm hwd`: plus the internal hand wrench (Wong et al. eq. 49-50 on
the null space of G), 25/29; it chases F/T MOMENTS, so it is locked behind
`allow_hand_wrench` until they are verified. A route runs only with the control
it was trained with (scalar routes under the wrench: 24 -> 11 of 29).

## Before the first insertion

1. **The cell.** The shelf frame is the simulator's world (floor z = 0, shelf in
   the x-z plane, left arm at -y, right at +y, no front or back panel: one tool
   reaches each face). Write each arm's `shelf_in_base` in `config/cell.yaml`.
   `check` prints how far the tools go from each shoulder: with the bases at
   shelf-floor height (the simulated cell) the top poses are **~990 mm** from the
   shoulder against a Gen3's 902 mm and span **~0.9 m in height** - raise the
   bases (or lower the shelf) so the 180-1070 mm band sits around the shoulder.
2. **The board.** Each route file lists the neighbours (`neighbours`: piece,
   position, quaternion, shelf frame) and the shelf (`shelf`: 6x6 cells of
   120 mm, seam 3 mm, side walls). Build that board; the routes were trained on
   it and on nothing else.
3. **The pieces.** 120 mm cells, 0.250 kg for four cells (36 kg/m3: foam).
4. **The tool - THIS IS THE BIG UNKNOWN.** The real tool is two printed plates,
   a central spring and a sponge; the routes were trained with three spheres on
   zero-length springs (`spider_ee/floating_plate.py`). Measure and write in
   `tool:` the face geometry, then compare with what the routes assume:
   - pad-piece friction: the narrow75 routes were trained at **mu = 1.0**
     (`constants.mu_eff` in each route file); the SOCP's cones and the jam
     sentry use that value;
   - the whole coupling, hand to piece: the simulated one measured
     **2804 N/m across, 40.7 Nm/rad about the pad axis** (`constants.k_t`, `k_r`).
     A much stiffer or softer sponge is a different apparatus, and a route
     trained for one is not a route for the other;
   - the SOCP reasons on three points on a 30 mm circle (CONSTRAINT 1); on a
     sponge they are geometry, not measured contacts - a declared approximation.
5. **The F/T.** Set `tool_mass`, `tool_com_in_ft`, `ft_sign`. The sensors are
   zeroed (`tare`) twice per run with the tools in free air. The piece weighs
   2.45 N: the sensor must resolve well under 1 N.

## Files

| file | what |
|---|---|
| `controller.py` | the insertion: object frame, squeeze loop, SOCP, sentry, seat, release |
| `ros_backend.py` | the only file that knows ROS and the robot bases |
| `dryrun.py` | a backend with no robot, for `check` (not a physics simulation) |
| `geometry.py` | tool geometry, frames, route pacing |
| `route.py` | reads one exported route |
| `socp.py`, `prior.py`, `prior_torsional_C.npz` | copied unchanged from `10_tripad_passivo/lib/` (session-C prior, Kp_rot 40) |
| `routes/<arm>/<world>.json` | the route (object poses + the model's slot reaction per waypoint), grasp, board, constants, the PyBullet result |
| `routes/<arm>/<world>_trace.npz` | the flange poses PyBullet commanded on that run, 100 Hz - the reference `check` compares against |
| `config/cell.yaml` | topics, frames, calibration, safety |

The routes are exported on the simulation PC by
`dmp_realistic_6x6/tight/kinova_export/export_routes.py` (the trainer replays
each best route with `--export-route`); the PyBullet placement in each file is
that replay, and matches `BEST_ROUTES.md` / `BEST_ROUTES_HWD.md`.

## What is NOT known yet - say it before quoting a result

- Every route was trained and measured on ONE board at the NOMINAL grasp in
  PyBullet. Robustness to the real grasp error is not tested.
- The real tool's compliance and friction are not the simulated ones (above).
- The arm's impedance controller is assumed to reproduce the simulated wrist;
  its real stiffness must be checked (push the flange, read the F/T).
- Verified offline only: on `check`, the flange path commanded in the route
  phase agrees with PyBullet's to 0.2 mm median, 0.6 mm max (J s7109). The ROS
  backend has not run against a robot.
