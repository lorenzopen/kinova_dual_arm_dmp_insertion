# HANDOFF - state of the Kinova port (2026-10-12)

Read this first in a new session. The package is NOT yet on the robot PC and
has NEVER run on a robot.

## The goal (and only this)

Lorenzo's thesis (dual-arm manipulation, SOCP with TP-GMM prior, Prof. Campolo,
NTU). This package does ONE thing: two Kinova Gen3 arms insert ONE tetromino
into a narrowed shelf slot ("narrow75") along a DMP route trained in PyBullet.
No feeder, no vision, no stand-up, no Tetris game. The piece is presented by
hand above its slot; the board is built by hand from the route file.

- **narrow75** = on each flank the most intruding neighbour enters the piece's
  silhouette by 12.5% of a cell per side: opening = 75% of the piece
  (120 mm trained: 15 mm/side, 90 mm opening; 100 mm real: 12.5 mm/side, 75 mm).
  NOT to be confused with the grid seam between columns (3 mm trained, 2.5 mm scaled).
- **Real hardware**: pieces built from 100 mm cubes (trained: 120 mm). Tri-pad =
  two 3D-printed plastic plates, a central spring, a sponge face, triangle side
  70 mm (trained: three spheres, 52 mm side, circumradius 30 mm).

## Where things are

| what | where |
|---|---|
| this repo (deploy copy, the only thing for the robot PC) | `C:\Users\Lorenzo\Desktop\kinova_dual_arm_dmp_insertion` |
| canonical copy inside the thesis (all thesis code lives there, CONSTRAINT 6) | `TESI_antiG\lorenzo_data_analysis\kinova_dual_arm_dmp_insertion\` - keep the two in sync |
| route exporter + scale test (simulation PC only) | `TESI_antiG\lorenzo_data_analysis\dmp_realistic_6x6\tight\kinova_export\` |
| trainer flags added for this | `10_tripad_passivo/tetris/38_learn_pybullet.py --export-route / --route-json / --scale / --tripad-side` (declared arms, defaults unchanged) |
| the lab's Kinova stack (NOT ours, never modify) | robot PC `~/ros2_ws/src/kinova_controller_pkgs` (copy in `Downloads\kinova_controller_pkgs-...`) |
| the thesis notes | `TESI_antiG\CLAUDE.md` §8a, last three rows |

## What the package does

`controller.py` reproduces the trainer's `drive_hands` step for step: one object
frame carried by two flange targets, ONE scalar squeeze `ds` closed on the
measured normal force towards the SOCP's target (TP-GMM prior, every 100 ms),
jam sentry, slow straight seat, stop on a high landing, seat by feel, release.
The wrist impedance is the arm controller's job (sim: 1500 N/m, 40 Nm/rad about
the push axis, 8 tilt).

`ros_backend.py` talks to the LAB stack as is:
`ros2 launch kinova_control dual_cartesian.launch.py` (Pinocchio Cartesian
impedance in torque at 1 kHz, gravity compensated from the URDF; namespaces
/left = 192.180.0.108, /right = 192.180.0.107). It publishes
`/<ns>/target_pose` (PoseStamped, base_link), reads `/<ns>/joint_states`
(flange pose by Pinocchio FK on the same URDF - that stack publishes no TF) and
`/<ns>/estimated_wrench` (from joint torque sensors, ee frame), tares with
`/<ns>/zero_wrench`.

Routes: `routes/scalar/` (29, the default, 24/29 in PyBullet) and `routes/hwd/`
(29, + internal hand wrench, 25/29). `config/cell.yaml: scale 0.8333` scales
every position to the 100 mm pieces on load.

## Measured so far (all offline / in simulation)

- Export reproduces the PyBullet ledgers (L s7040 5.11 mm, J s7109 2.6, I s7196 11.0).
- `check` (dry run, no physics): commanded flange path in the route phase = PyBullet's to 0.2 mm median (J s7109, scale 1).
- Same results with Jazzy's numpy 1.26.4 / scipy 1.11.4 / cvxpy 1.6.7.
- **Scaling, PyBullet, 3 worlds** (placed / slip mm; "in" = <= 10 / 16.7 mm at 100 mm):
  I s7196 120mm 11.0/8.4 -> scaled real tri-pad 4.5/8.0; L s7040 5.1/2.4 -> 8.7/9.0;
  T s7039 2.1/1.1 -> 2.6/3.0. All in, 3+3 pads. A signal on 3 worlds, not a result on 29.
- Reach: at scale 0.833 the farthest pose is 826 mm from the shoulder (Gen3 902).
- A 70 mm tri-pad with 10 mm spheres needs a 100.8 mm free disc > a 100 mm face; the test used 8.3 mm spheres. Where the real sponge ends matters.

## Open before the first `insert`

1. **Wrench noise.** The joint-torque estimate may be too noisy for the seat by feel
   (threshold 60% of the weight ~ 0.85 N for a 0.145 kg piece). Measure: arm still,
   `zero_wrench`, then `ros2 topic hz` / `echo /left/estimated_wrench` for 10 s.
   If the noise is over ~0.3 N: the lab's ATI Mini40 (`wrench_source: ft`) or heavier pieces.
2. **URDF mass.** Their `gen3_p2_F_only.urdf` carries the 2F-85 gripper (0.925 kg) at
   end_effector_link. With a tri-pad mounted, make a COPY with the tri-pad's mass and
   pass it as `urdf_path` (launch AND `cell.yaml`). The 2F-85 and the LinkerHand
   must be unmounted and remounted - agree it with the lab.
3. **Calibrate `config/cell.yaml`**: `shelf_in_base` per arm, `tool_in_ee`, `tool:`
   (pad_x, pad_r, r_tri), `piece_mass` (weigh it), `urdf_path`.
4. **Stiffness**: launch with `ee_frame:=end_effector_link` and the `cart_k` printed by
   `check`. The lab only ran up to 350 N/m (1000 commented): go up in steps. Never lower
   `max_torque` (it also clips gravity compensation).
5. **Seam of the real board**: 2.5 mm (as tested) or 3 mm? Ask Lorenzo; at 3 mm re-run
   the 3-world scale test.
6. Not done: IK / joint-limit / arm-arm collision check over the trajectory (Pinocchio
   on their URDF can do it once the base poses are known); scale test on the other 26 worlds.

## Rules that matter here

- The robot PC is SHARED: never build in or edit `~/ros2_ws` (the colleague's), never
  change its network connections, never `pip install` into the system or `~/.local`
  (their controller needs numpy < 2). Use `~/lorenzo_ws` and `~/lorenzo_venv`.
- The route files are DATA exported by the trainer; a route runs only with the control
  it was trained with (scalar vs hwd).
- Report placements measured by hand (worst cell vs target); a run that fails is data.
