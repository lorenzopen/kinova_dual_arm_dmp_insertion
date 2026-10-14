"""BOTH Kinova follow the flange paths of a learnt DMP route together, slowly. No piece.

The second test on the real robots, after `follow_one_arm`: can the two arms
carry an (imaginary) piece together - the same piece pose, at the same instant,
for both? Empty hands, no squeeze, no SOCP, no force loop: only the two flange
targets `insert` would send, streamed to the lab's controllers (`/left`,
`/right` `target_pose`) at a fraction of the trained speed, by ONE loop, with
ONE clock, so the two can only drift apart as much as their impedances lag.

What it sends, in order (all slow, straight line + slerp between poses):
    1. from where each flange is NOW to its grasp pose; both arrive together
       (the slower of the two sets the time, at --speed / --turn)
    2. the grasp pose -> the mouth of the slot -> the learnt route -> the slot,
       stretched to last --duration seconds; the two arms share the waypoint
       times (waypoint k = the same piece pose for both)
    3. nothing more: the lab controllers keep the last targets

WHERE (pair.py): the two bases are parallel, the left one at y = +--sep (0.60 m)
of the right one. The right tool looks along base +y (to its left), the left
tool along base -y (to its right); the piece path runs half way between them,
mean x = --forward. `--cell` uses the calibrated `shelf_in_base` of
config/cell.yaml instead.

THE GAP between the two tool tips (the sponge faces that look at each other) is
--gap: 0.20 m by default, for the first runs; the real grasp is the piece's
width (`--gap 0.10`, printed as "the grasp"). Each tool stays (gap - width) / 2
further out than at the grasp, all along the route (the piece frame carries it).

It stops BOTH arms (each holds where it is) if either flange is more than
--stop-error from its target, and on Ctrl-C. It logs commanded and measured
flanges of both arms, the distance between them (what would squeeze the piece)
and the estimated wrenches (their noise, arm still and moving) to logs/.

Lab stack first: one `kinova_controller.py` per arm (SETUP_PC_ROBOT.md, part E),
/left = 192.180.0.200, /right = 192.180.0.107. Then, venv active:

    python -m kinova_dual_arm_dmp_insertion.follow_two_arms F75_Tt1_s9007 --dry
    python -m kinova_dual_arm_dmp_insertion.follow_two_arms F75_Tt1_s9007 --gap 0.20
"""
import argparse
import csv
import os
import threading
import time

import numpy as np

from . import geometry as GE
from . import pair
from .follow_one_arm import DURATION, RATE, SPEED, STOP_ERROR, TURN, hand_path, to_base
from .main import load_config
from .route import Route

SIDES = ("left", "right")
GAP = 0.20           # [m] tool tip to tool tip, first runs (the grasp: the piece's width)


def plan(cfg, route, spread, cell_lift):
    """Per arm: ee_frame targets in its own base_link, (positions [N,3], rotations)."""
    out = {}
    for side in SIDES:
        p, q = hand_path(route, side, spread)
        p = p + np.array([0.0, 0.0, cell_lift])
        tg = [to_base(cfg[side], pp, qq) for pp, qq in zip(p, q)]
        out[side] = (np.array([t[0] for t in tg]), [t[1] for t in tg])
    return out


def shared_times(paths, speed, turn, duration):
    """One waypoint clock for both arms: each segment as slow as the slower arm
    needs, then the whole stretched to `duration` (the trained pacing kept)."""
    steps = [np.diff(GE.segment_times(*paths[s], speed, turn), prepend=0.0) for s in SIDES]
    times = np.cumsum(np.maximum(*steps))
    return times * (duration / times[-1])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", help="route name, e.g. F75_Tt1_s9007")
    ap.add_argument("--routes", default="c100_five_cut800", help="folder under routes/")
    ap.add_argument("--speed", type=float, default=SPEED, help="[m/s] to the start (default 0.010)")
    ap.add_argument("--turn", type=float, default=TURN, help="[rad/s] to the start (default 0.05)")
    ap.add_argument("--duration", type=float, default=DURATION, help="[s] the route (default 40)")
    ap.add_argument("--sep", type=float, default=pair.SEP,
                    help=f"[m] left base at +y of the right one (default {pair.SEP})")
    ap.add_argument("--forward", type=float, default=pair.FORWARD,
                    help=f"[m] base x of the piece path (its mean, default {pair.FORWARD})")
    ap.add_argument("--lift", type=float, default=0.0,
                    help="[m] run the whole path this much HIGHER (first tests in free air)")
    ap.add_argument("--shelf-x", type=float, default=None,
                    help="[m] the shelf origin at this base x, the same for every world "
                         "(instead of centring each route on --forward)")
    ap.add_argument("--gap", type=float, default=GAP,
                    help=f"[m] between the two tool tips, facing each other (default {GAP}; "
                         f"the grasp = the piece's width)")
    ap.add_argument("--cell", action="store_true",
                    help="use the calibrated shelf_in_base of config/cell.yaml instead of --sep/--forward")
    ap.add_argument("--stop-error", type=float, default=STOP_ERROR,
                    help="[m] stop both if a flange is this far from its target (default 0.040)")
    ap.add_argument("--dry", action="store_true", help="no robot: print the plan, write the CSV")
    ap.add_argument("--config", default=None)
    a = ap.parse_args()

    cfg, cfg_path = load_config(a.config)
    r = Route(a.world, a.routes, cfg.get("scale", 1.0), cfg.get("piece_mass"))
    print("  " + r.summary())
    print(f"  config {os.path.abspath(cfg_path)}")
    if a.cell:
        cell_lift = a.lift
    else:
        cfg = pair.place(cfg, r, a.sep, a.forward, a.lift, a.shelf_x)
        cell_lift = 0.0                                   # already in shelf_in_base
        for line in pair.describe(cfg, r):
            print("  " + line)
    # THE GAP: each tool moves out by half of what --gap adds to the piece's width
    if a.gap < r.opening:
        raise SystemExit(f"  --gap {a.gap} is less than the piece ({r.opening:.3f} m): the tips would cross")
    spread = 0.5 * (a.gap - r.opening)
    print(f"  tool tips {1000 * a.gap:.0f} mm apart (the grasp: --gap {r.opening:.3f}), "
          f"each tool {1000 * spread:.0f} mm out")
    paths = plan(cfg, r, spread, cell_lift)
    times = shared_times(paths, a.speed, a.turn, a.duration)

    # the distance between the two flanges, in the RIGHT base (left base at +sep y)
    off = np.array([0.0, a.sep, 0.0]) if not a.cell else None
    for side in SIDES:
        pts = paths[side][0]
        shoulder = np.asarray(cfg[side].get("shoulder_in_base", [0.0, 0.0, 0.2848]))
        far = 1000 * np.linalg.norm(pts - shoulder, axis=1).max()
        look = paths[side][1][0].apply([0.0, 0.0, 1.0])  # ee z = the tool's push direction
        print(f"  {side}: {len(pts)} poses, {1000 * np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)):.0f} mm "
              f"in {times[-1]:.0f} s; farthest {far:.0f} mm from the shoulder (reach "
              f"{1000 * cfg['max_reach']:.0f}); first pose {np.round(1000 * pts[0], 1).tolist()} mm, "
              f"flange looking {np.round(look, 2).tolist()} (base axes)")
        print(f"        x {1000 * pts[:, 0].min():.0f}..{1000 * pts[:, 0].max():.0f}, "
              f"y {1000 * pts[:, 1].min():.0f}..{1000 * pts[:, 1].max():.0f}, "
              f"z {1000 * pts[:, 2].min():.0f}..{1000 * pts[:, 2].max():.0f} mm")
        if far > 1000 * cfg["max_reach"]:
            raise SystemExit(f"  {side} out of reach: --forward / --lift, or shelf_in_base")
    if off is not None:
        gap = np.linalg.norm(paths["left"][0] + off - paths["right"][0], axis=1)
        print(f"  flange to flange: {1000 * gap.min():.0f}..{1000 * gap.max():.0f} mm "
              f"(tool tips {1000 * (gap.min() - 2 * (GE.PAD_X + GE.PAD_R)):.0f} mm apart at the closest)")

    os.makedirs(cfg["log_dir"], exist_ok=True)
    if a.dry:
        out = os.path.join(cfg["log_dir"], f"follow2_{a.world}_dry.csv")
        with open(out, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t"] + [f"{c}{s[0].upper()}" for s in SIDES for c in ("x", "y", "z", "qx", "qy", "qz", "qw")])
            for t in np.arange(0.0, times[-1], 1.0 / RATE):
                row = [f"{t:.2f}"]
                for side in SIDES:
                    c, R, *_ = GE.interpolate(times, *paths[side], t)
                    row += [*np.round(c, 5), *np.round(R.as_quat(), 6)]
                w.writerow(row)
        print(f"  dry run: targets -> {out}")
        return

    # ---- the robots: only from here on ROS is needed
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from geometry_msgs.msg import PoseStamped, WrenchStamped
    from sensor_msgs.msg import JointState
    from .ros_backend import _FK

    rclpy.init()
    node = rclpy.create_node("kinova_follow_two_arms")
    last = {s: {} for s in SIDES}
    pubs, fks = {}, {}
    for side in SIDES:
        acfg = cfg[side]
        pubs[side] = node.create_publisher(PoseStamped, acfg["target_topic"], 10)
        node.create_subscription(JointState, acfg["joint_states_topic"],
                                 lambda m, s=side: last[s].update(js=m), 10)
        node.create_subscription(WrenchStamped, acfg["wrench_topic"],
                                 lambda m, s=side: last[s].update(w=m), 10)
        fks[side] = _FK(acfg["urdf_path"], acfg["ee_frame"])
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    threading.Thread(target=exe.spin, daemon=True).start()

    def flange_now(side):
        """The ee_frame pose in this arm's base_link now, by forward kinematics."""
        return fks[side].pose(last[side]["js"])

    def force_now(side):
        w = last[side].get("w")
        return [w.wrench.force.x, w.wrench.force.y, w.wrench.force.z] if w is not None else [np.nan] * 3

    def send(side, p, q):
        msg = PoseStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = cfg[side]["base_frame"]
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
        x, y, z, w = q.as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
        pubs[side].publish(msg)

    def hold_both():
        for side in SIDES:
            send(side, *flange_now(side))

    t0 = time.monotonic()
    while any("js" not in last[s] for s in SIDES):
        time.sleep(0.05)
        if time.monotonic() - t0 > 10.0:
            missing = [cfg[s]["joint_states_topic"] for s in SIDES if "js" not in last[s]]
            raise SystemExit(f"  nothing on {', '.join(missing)}: are both lab controllers running?")
    if any("w" not in last[s] for s in SIDES):
        print("  (no estimated_wrench yet: the log will have no forces)")

    # step 1: each arm from where it is to its first pose, both in the same time
    now = {s: flange_now(s) for s in SIDES}
    start = max(GE.segment_times([now[s][0], paths[s][0][0]], [now[s][1], paths[s][1][0]],
                                 a.speed, a.turn)[-1] for s in SIDES)
    start = max(start, 2.0)
    for s in SIDES:
        print(f"  {s}: the flange is {1000 * np.linalg.norm(paths[s][0][0] - now[s][0]):.0f} mm "
              f"from its first pose")
    print(f"  {start:.0f} s to get there (both), then {times[-1]:.0f} s of path")
    input("  >>> clear around BOTH arms, hand on the E-stops. [Enter] to start, Ctrl-C to quit ")

    now = {s: flange_now(s) for s in SIDES}             # again: the arms may have drifted
    plans = [("to_start", np.array([0.0, start]),
              {s: (np.array([now[s][0], paths[s][0][0]]), [now[s][1], paths[s][1][0]]) for s in SIDES}),
             ("route", times, paths)]
    log_path = os.path.join(cfg["log_dir"], time.strftime(f"follow2_{a.world}_%Y%m%d_%H%M%S.csv"))
    fh = open(log_path, "w", newline="")
    log = csv.writer(fh)
    log.writerow(["t", "phase"] + [f"{c}{s[0].upper()}" for s in SIDES for c in
                                   ("xc", "yc", "zc", "xm", "ym", "zm", "err_mm", "rot_deg", "Fx", "Fy", "Fz")]
                 + ["gap_cmd_mm", "gap_meas_mm"])
    err_max = {s: 0.0 for s in SIDES}
    gap_err_max, forces = 0.0, {s: [] for s in SIDES}
    try:
        for name, tt, P in plans:
            print(f"  {name} ...")
            t0 = time.monotonic()
            while True:
                t = time.monotonic() - t0
                row, cmd, meas = [f"{t:.3f}", name], {}, {}
                for side in SIDES:
                    c, R, *_ = GE.interpolate(tt, *P[side], t)
                    send(side, c, R)
                    cmd[side] = (c, R)
                for side in SIDES:
                    p_m, q_m = flange_now(side)
                    meas[side] = p_m
                    c, R = cmd[side]
                    err = float(np.linalg.norm(p_m - c))
                    rot = float(np.degrees((R.inv() * q_m).magnitude()))
                    err_max[side] = max(err_max[side], err)
                    f = force_now(side)
                    if name == "route":
                        forces[side].append(f)
                    row += [*np.round(1000 * c, 1), *np.round(1000 * p_m, 1), f"{1000 * err:.1f}",
                            f"{rot:.2f}", *np.round(f, 2)]
                    if err > a.stop_error:
                        hold_both()
                        raise RuntimeError(f"{side} flange {1000 * err:.0f} mm from its target")
                if off is not None:
                    g_c = np.linalg.norm(cmd["left"][0] + off - cmd["right"][0])
                    g_m = np.linalg.norm(meas["left"] + off - meas["right"])
                    gap_err_max = max(gap_err_max, abs(g_m - g_c))
                    row += [f"{1000 * g_c:.1f}", f"{1000 * g_m:.1f}"]
                log.writerow(row)
                if t >= tt[-1]:
                    break
                time.sleep(1.0 / RATE)
        print("  done: both arms hold the last pose (the slot)")
    except RuntimeError as e:
        print(f"  STOP: {e} - both arms hold where they are")
    except KeyboardInterrupt:
        hold_both()
        print("\n  stopped by hand: both arms hold where they are")
    finally:
        fh.close()
        for side in SIDES:
            f = np.array(forces[side], float)
            noise = (f"force std over the route {np.round(np.nanstd(f, axis=0), 2).tolist()} N"
                     if len(f) and not np.all(np.isnan(f)) else "no force readings")
            print(f"  {side}: worst tracking {1000 * err_max[side]:.1f} mm; {noise}")
        if off is not None:
            print(f"  flange-to-flange distance vs commanded: worst {1000 * gap_err_max:.1f} mm "
                  f"(what would squeeze or drop the piece)")
        print(f"  log: {log_path}")
        time.sleep(0.2)
        exe.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
