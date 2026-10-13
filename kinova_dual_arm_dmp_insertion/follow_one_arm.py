"""ONE Kinova follows the flange path of a learnt DMP route, very slowly. No piece.

The first test on the real robot: is the path where we think it is, and can the
arm follow it? No squeeze, no SOCP, no force loop, no second arm, no piece -
only the flange targets this arm would get during `insert`, streamed to the
lab's controller (`/<ns>/target_pose`) at a fraction of the trained speed.

What it sends, in order (all slow, straight line + slerp between poses):
    1. from where the flange is NOW to the grasp pose of the route
       (at --speed / --turn, 10 mm/s and 3 deg/s)
    2. the grasp pose -> the mouth of the slot -> the learnt route -> the slot,
       stretched to last --duration seconds (default 40; the trained pacing
       between waypoints is kept, only slowed down)
    3. nothing more: the lab controller keeps the last target, the arm stays there

WHERE: by default, with no shelf and no calibration, the imaginary shelf
stands at the arm's LEFT: the path is turned about z so the tool pushes along
the base's +y (the flange looking left), and it runs in an x-z plane in front
of the base - mean x = --forward (default 0.40 m), y = --side (default
0.20 m), heights as in the shelf frame (+ --lift). `--cell` uses the
calibrated `shelf_in_base` of config/cell.yaml instead (the two-arm cell).

The hand pose along the route is computed exactly as `controller.Insertion`
does it: the two tools are frozen to the piece at the grasp pose, then follow
the piece frame (`freeze` / `drive`). The pads are placed touching the piece's
faces (half the opening + PAD_X + PAD_R from the grasp centre).

Lab stack running first (another terminal), e.g. only the left arm:

    ros2 run kinova_control kinova_controller.py --ip 192.180.0.108 --ros-args \
        -r __ns:=/left -p initial_mode:=cartesian -p home_on_start:=true \
        -p ee_frame:=end_effector_link -p urdf_path:=<URDF> \
        -p cart_k:='[400.0, 400.0, 400.0, 15.0, 15.0, 15.0]' \
        -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'

then, venv active, from the package folder:

    python -m kinova_dual_arm_dmp_insertion.follow_one_arm F75_Tt1_s9007 --routes c100_five_cut800 --arm left --dry
    python -m kinova_dual_arm_dmp_insertion.follow_one_arm F75_Tt1_s9007 --routes c100_five_cut800 --arm left

`--dry` needs no robot and no ROS: it prints the plan and writes the targets
to a CSV. Ctrl-C at any time: the arm is told to stay where it is.
"""
import argparse
import csv
import os
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from . import geometry as GE
from .main import load_config
from .route import Route

SPEED = 0.010        # [m/s]   10 mm/s, to the start: the trained route moves at 85 mm/s
TURN = 0.05          # [rad/s] 3 deg/s, to the start: the trained route turns at 0.25 rad/s
DURATION = 40.0      # [s]     the route itself
RATE = 100.0         # [Hz]    targets sent per second (the controller runs at 1 kHz)
STOP_ERROR = 0.040   # [m]     stop if the flange is this far from its target


def hand_path(route, side):
    """(positions, rotations) of THIS arm's tool, shelf frame: grasp pose, mouth,
    route, slot - as controller.Insertion.freeze / drive compute it."""
    a = route.axis / np.linalg.norm(route.axis)          # LEFT pushes along +a
    sign = -1.0 if side == "left" else +1.0
    reach = 0.5 * route.opening + GE.PAD_X + GE.PAD_R    # pads touching the faces
    hand0 = route.centre + sign * reach * a               # tool position at the grasp
    q0 = GE.quat_from_x(-sign * a)                        # tool x points AT the piece
    # the piece frame: grasp pose, then the route (its first point is the mouth), then the slot
    obj_c = [route.grasp_c] + list(route.centres) + [route.target]
    obj_R = [route.rots[0]] + list(route.rots) + [route.R_target]
    R0, c0 = route.rots[0], route.grasp_c
    offset = R0.inv().apply(hand0 - c0)                   # the tool, seen from the piece
    pos, rot = [], []
    for c, R in zip(obj_c, obj_R):
        dR = R * R0.inv()
        pos.append(c + R.apply(offset))
        rot.append(dR * q0)
    return np.array(pos), rot


def to_base(cfg_arm, p, q):
    """Tool pose in the shelf frame -> ee_frame target in this arm's base_link."""
    shelf_in_base = GE.Frame(cfg_arm["shelf_in_base"]["xyz"], cfg_arm["shelf_in_base"]["quat"])
    ee_in_tool = GE.Frame(cfg_arm["tool_in_ee"]["xyz"], cfg_arm["tool_in_ee"]["quat"]).inverse()
    pb, qb = shelf_in_base.apply(np.asarray(p, float), q)
    return pb + qb.apply(ee_in_tool.t), qb * ee_in_tool.R


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", help="route name, e.g. F75_Tt1_s9007")
    ap.add_argument("--routes", default="c100_five_cut800", help="folder under routes/")
    ap.add_argument("--arm", choices=("left", "right"), default="left")
    ap.add_argument("--speed", type=float, default=SPEED, help="[m/s] to the start (default 0.010)")
    ap.add_argument("--turn", type=float, default=TURN, help="[rad/s] to the start (default 0.05)")
    ap.add_argument("--duration", type=float, default=DURATION, help="[s] the route (default 40)")
    ap.add_argument("--forward", type=float, default=0.40,
                    help="[m] base x of the path (its mean, default 0.40)")
    ap.add_argument("--side", type=float, default=0.20,
                    help="[m] base y of the path's plane, the shelf on the left (default 0.20)")
    ap.add_argument("--cell", action="store_true",
                    help="use the calibrated shelf_in_base of config/cell.yaml instead")
    ap.add_argument("--lift", type=float, default=0.0,
                    help="[m] run the whole path this much HIGHER (first tests in free air)")
    ap.add_argument("--stop-error", type=float, default=STOP_ERROR,
                    help="[m] stop if the flange is this far from its target (default 0.040)")
    ap.add_argument("--dry", action="store_true", help="no robot: print the plan, write the CSV")
    ap.add_argument("--config", default=None)
    a = ap.parse_args()

    cfg, cfg_path = load_config(a.config)
    acfg = cfg[a.arm]
    r = Route(a.world, a.routes, cfg.get("scale", 1.0), cfg.get("piece_mass"))
    print("  " + r.summary())
    p_shelf, q_shelf = hand_path(r, a.arm)
    p_shelf = p_shelf + np.array([0.0, 0.0, a.lift])
    if not a.cell:
        # turn the path about z so the tool pushes along base +y, then place it
        d = q_shelf[0].apply([1.0, 0.0, 0.0])
        turn = Rot.from_rotvec([0.0, 0.0, 0.5 * np.pi - np.arctan2(d[1], d[0])])
        moved = turn.apply(p_shelf)
        shift = np.array([a.forward - moved[:, 0].mean(), a.side - moved[:, 1].mean(), 0.0])
        acfg = dict(acfg, shelf_in_base={"xyz": shift.tolist(), "quat": turn.as_quat().tolist()})
        print(f"  shelf at the arm's left (not calibrated): shelf_in_base xyz "
              f"{np.round(shift, 4).tolist()}, turned {np.degrees(turn.magnitude()):.0f} deg about z")
    targets = [to_base(acfg, p, q) for p, q in zip(p_shelf, q_shelf)]
    pts = np.array([t[0] for t in targets])
    rots = [t[1] for t in targets]
    times = GE.segment_times(pts, rots, a.speed, a.turn)
    times = times * (a.duration / times[-1])              # the trained pacing, stretched
    shoulder = np.asarray(acfg.get("shoulder_in_base", [0.0, 0.0, 0.2848]))
    far = 1000 * np.linalg.norm(pts - shoulder, axis=1).max()
    print(f"  config {os.path.abspath(cfg_path)}")
    print(f"  {a.arm} arm: {len(pts)} poses, {1000 * np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)):.0f} mm "
          f"of path in {times[-1]:.0f} s; farthest {far:.0f} mm from the shoulder "
          f"(reach {1000 * cfg['max_reach']:.0f}); lift {1000 * a.lift:.0f} mm")
    print(f"  first pose in base_link: {np.round(1000 * pts[0], 1).tolist()} mm")
    if far > 1000 * cfg["max_reach"]:
        raise SystemExit("  out of reach: check shelf_in_base in config/cell.yaml")

    if a.dry:
        os.makedirs(cfg["log_dir"], exist_ok=True)
        out = os.path.join(cfg["log_dir"], f"follow_{a.arm}_{a.world}_dry.csv")
        with open(out, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
            for t in np.arange(0.0, times[-1], 1.0 / RATE):
                c, R, *_ = GE.interpolate(times, pts, rots, t)
                w.writerow([f"{t:.2f}", *np.round(c, 5), *np.round(R.as_quat(), 6)])
        print(f"  dry run: targets -> {out}")
        return

    # ---- the robot: only from here on ROS is needed
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from .ros_backend import _FK

    rclpy.init()
    node = rclpy.create_node("kinova_follow_one_arm")
    pub = node.create_publisher(PoseStamped, acfg["target_topic"], 10)
    last = {}
    node.create_subscription(JointState, acfg["joint_states_topic"], lambda m: last.update(js=m), 10)
    fk = _FK(acfg["urdf_path"], acfg["ee_frame"])

    def flange_now():
        """The ee_frame pose in base_link now, by forward kinematics."""
        rclpy.spin_once(node, timeout_sec=0.0)
        return fk.pose(last["js"])

    def send(p, q):
        msg = PoseStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = acfg["base_frame"]
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
        x, y, z, w = q.as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
        pub.publish(msg)

    t0 = time.monotonic()
    while "js" not in last:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - t0 > 10.0:
            raise SystemExit(f"  nothing on {acfg['joint_states_topic']}: is the lab controller running?")

    # step 1: from where the arm is now to the first pose of the path, same slow pace
    p_now, q_now = flange_now()
    start = GE.segment_times([p_now, pts[0]], [q_now, rots[0]], a.speed, a.turn)
    print(f"  the flange is {1000 * np.linalg.norm(pts[0] - p_now):.0f} mm from the first pose: "
          f"{start[-1]:.0f} s to get there, then {times[-1]:.0f} s of path")
    input("  >>> clear around the arm, hand on the E-stop. [Enter] to start, Ctrl-C to quit ")

    plans = [(start, [p_now, pts[0]], [q_now, rots[0]], "to the start"),
             (times, pts, rots, "the route")]
    try:
        for tt, P, Q, name in plans:
            print(f"  {name} ...")
            t0 = time.monotonic()
            while True:
                t = time.monotonic() - t0
                c, R, *_ = GE.interpolate(tt, P, Q, t)
                send(c, R)
                p_meas, _ = flange_now()
                err = float(np.linalg.norm(p_meas - c))
                if err > a.stop_error:
                    print(f"  STOP: the flange is {1000 * err:.0f} mm from its target")
                    send(*flange_now())             # stay where it is
                    return
                if t >= tt[-1]:
                    break
                time.sleep(1.0 / RATE)
        print("  done: the arm holds the last pose (the slot)")
    except KeyboardInterrupt:
        send(*flange_now())                           # stay where it is
        print("\n  stopped by hand: the arm holds where it is")
    finally:
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
