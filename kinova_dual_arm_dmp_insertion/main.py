"""Command line.

    ros2 run kinova_dual_arm_dmp_insertion list
    ros2 run kinova_dual_arm_dmp_insertion check  L_s7040m6 [--arm scalar]   # no robot: dry run + reach
    ros2 run kinova_dual_arm_dmp_insertion insert L_s7040m6 [--arm scalar] [--config cell.yaml]

The real pair without a shelf calibration (pair.py: left base at +0.60 m y of the
right one, tools facing each other), the route 3 times slower than trained:

    python -m kinova_dual_arm_dmp_insertion.main check  F75_Tt1_s9007 --arm c100_five_cut800 --sep 0.60
    python -m kinova_dual_arm_dmp_insertion.main insert F75_Tt1_s9007 --arm c100_five_cut800 --sep 0.60 --slow 3

`check` and `list` need no ROS at all (python -m kinova_dual_arm_dmp_insertion.main check ...).
"""
import argparse
import csv
import os
import sys
import time

import numpy as np

from . import geometry as GE
from . import pair
from .route import Route, available

HERE = os.path.dirname(os.path.abspath(__file__))


def load_config(path):
    import yaml
    if path is None:
        for p in (os.path.join(HERE, "..", "config", "cell.yaml"),):
            if os.path.isfile(p):
                path = p
        if path is None:   # installed by colcon: share/<pkg>/config
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("kinova_dual_arm_dmp_insertion"),
                                "config", "cell.yaml")
    with open(path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    t = cfg.get("tool", {})
    GE.set_tool(t.get("pad_x", 0.020), t.get("r_tri", 0.030), t.get("pad_r", 0.010))
    return cfg, path


class Log:
    """One CSV row per control tick: what was commanded, what was measured."""
    COLS = ("t phase xL yL zL qxL qyL qzL qwL xR yR zR qxR qyR qzR qwR "
            "mxL myL mzL mxR myR mzR FxL FyL FzL FxR FyR FzR ds_mm target_N "
            "fn_N ext_x ext_y ext_z jammed").split()

    def __init__(self, path):
        self.fh = open(path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(self.COLS)

    def row(self, ins, tL, qL, tR, qR):
        m = ins.meas
        e = ins.external_force()
        self.w.writerow([f"{ins.b.now():.3f}", ins.phase, *np.round(tL, 5), *np.round(qL.as_quat(), 5),
                         *np.round(tR, 5), *np.round(qR.as_quat(), 5), *np.round(m["pL"], 5),
                         *np.round(m["pR"], 5), *np.round(m["FL"], 2), *np.round(m["FR"], 2),
                         f"{1000 * ins.ds:.2f}", f"{ins.target or 0:.2f}", f"{ins.normal_force():.2f}",
                         *np.round(e, 2), int(ins.jammed)])

    def close(self):
        self.fh.close()


def reach_report(route, cfg, rows):
    """How far each commanded tool pose is from its arm's base (shoulder) - a
    first reachability check; the arm's own IK has the last word."""
    from .geometry import Frame
    out = []
    for side, cols in (("left", slice(1, 4)), ("right", slice(8, 11))):
        f = Frame(**{"t": cfg[side]["shelf_in_base"]["xyz"], "q": cfg[side]["shelf_in_base"]["quat"]})
        p = f.t + f.R.apply(np.asarray(rows)[:, cols].astype(float))
        d = np.linalg.norm(p - np.asarray(cfg[side].get("shoulder_in_base", [0, 0, 0.28])), axis=1)
        out.append(f"{side}: farthest {1000 * d.max():.0f} mm from the shoulder "
                   f"(limit {1000 * cfg['max_reach']:.0f}); tool height in the base frame "
                   f"{1000 * p[:, 2].min():.0f} to {1000 * p[:, 2].max():.0f} mm, "
                   f"x {1000 * p[:, 0].min():.0f} to {1000 * p[:, 0].max():.0f}, "
                   f"y {1000 * p[:, 1].min():.0f} to {1000 * p[:, 1].max():.0f}")
    return out


def placed(a, cfg, r):
    """With --sep: the nominal pair (pair.py) instead of the calibrated shelf_in_base."""
    if a.sep is None:
        return cfg
    cfg = pair.place(cfg, r, a.sep, a.forward, a.lift, a.shelf_x)
    for line in pair.describe(cfg, r):
        print("  " + line)
    return cfg


def slowed(ins, slow):
    """The route, the seat and the approach `slow` times slower than trained
    (quasi-static: the squeeze and the SOCP run on their own clocks)."""
    ins.motion_speed /= slow
    ins.turn_rate /= slow
    ins.seat_speed /= slow
    return ins


def cmd_check(a, cfg):
    """Dry run: the whole sequence with no robot, then the commanded route
    against the flange poses the simulated apparatus was commanded."""
    from .controller import Insertion
    from .dryrun import DryRun
    r = Route(a.world, a.arm, cfg.get('scale', 1.0), cfg.get('piece_mass'))
    cfg = placed(a, cfg, r)
    b = DryRun(r, period=1.0 / cfg["rate_hz"])
    rows = []

    class Keep:
        def row(self, ins, tL, qL, tR, qR):
            rows.append([ins.b.now(), *tL, *qL.as_quat(), *tR, *qR.as_quat(), ins.phase])
    ins = slowed(Insertion(r, b, log=Keep(), confirm=lambda msg: print(f"  (dry run) {msg}")), a.slow)
    t0 = time.perf_counter()
    ins.run()
    print(f"  dry run: {b.now():.1f} s of motion, solved in {time.perf_counter() - t0:.1f} s; "
          f"SOCP ok {ins.stats['socp_ok']}/{ins.stats['socp_ticks']}")
    # the route phase against the simulator's commanded flanges (same piece frame, same pacing)
    trace = r.path[:-5] + "_trace.npz"
    if r.eff_scale != 1.0:
        print(f"  (the PyBullet trace is at another size: path compared only when the route is not rescaled)")
    elif os.path.isfile(trace):
        tr = np.load(trace)
        sim = tr["rows"][tr["phase"] == "route"]
        mine = np.array([x[:15] for x in rows if x[15] == "route"], float)
        if len(sim) and len(mine):
            # each of mine against the nearest simulated left flange position
            dist = [float(np.min(np.linalg.norm(sim[:, 1:4] - m[1:4], axis=1))) for m in mine]
            print(f"  route phase: {len(mine)} ticks here, {len(sim)} in PyBullet "
                  f"({mine[-1, 0] - mine[0, 0]:.1f} s vs {sim[-1, 0] - sim[0, 0]:.1f} s); "
                  f"left flange path vs PyBullet's: median {1000 * np.median(dist):.1f} mm, "
                  f"max {1000 * np.max(dist):.1f} mm (the squeeze differs: no real contact here)")
    for line in reach_report(r, cfg, [x[:15] for x in rows]):
        print("  " + line)
    for line in stiffness_report(r, cfg):
        print("  " + line)


def stiffness_report(route, cfg):
    """The lab controller's cart_k is diagonal in the BASE axes and fixed at
    launch: the torsional 40 Nm/rad goes on the base axis the push axis lies along."""
    from .geometry import Frame
    k = cfg["stiffness"]
    out = []
    a = route.axis / np.linalg.norm(route.axis)
    for side, push in (("left", a), ("right", -a)):
        f = Frame(cfg[side]["shelf_in_base"]["xyz"], cfg[side]["shelf_in_base"]["quat"])
        d = f.R.apply(push)
        i = int(np.argmax(np.abs(d)))
        rot = [k["tilt"]] * 3
        rot[i] = k["torsion"]
        kk = [k["trans"]] * 3 + rot
        line = (f"{side}: push axis in base_link {np.round(d, 3).tolist()} -> "
                f"cart_k:='[{', '.join(f'{v:.1f}' for v in kk)}]'")
        if abs(d[i]) < 0.95:
            line += "  WARNING: not along a base axis, the diagonal stiffness cannot match the sim"
        out.append(line)
    return out


def cmd_insert(a, cfg):
    from .controller import Abort, Insertion
    from .ros_backend import RosBackend
    r = Route(a.world, a.arm, cfg.get('scale', 1.0), cfg.get('piece_mass'))
    if r.hand_wrench != "off" and not cfg.get("allow_hand_wrench", False):
        sys.exit("this route was trained with the hand wrench (hwd): set allow_hand_wrench: true "
                 "in the config once the F/T moments are verified")
    os.makedirs(cfg["log_dir"], exist_ok=True)
    path = os.path.join(cfg["log_dir"], time.strftime(f"{r.world}_{a.arm}_%Y%m%d_%H%M%S.csv"))
    log = Log(path)
    b = RosBackend(cfg)
    ins = slowed(Insertion(r, b, log=log, f_abort=cfg["f_abort"], track_abort=cfg["track_abort"],
                                jam_scale=cfg.get("jam_scale", 1.0)), a.slow)
    try:
        ins.run()
    except (Abort, KeyboardInterrupt) as e:
        print(f"\n  STOPPED: {e or 'Ctrl-C'} - the arms hold their last target")
    finally:
        log.close()
        b.close()
        print(f"  log: {path}")


ARMS = ("scalar", "hwd", "c100_narrow75", "c100_narrow75hs", "c100_five", "c100_five625", "c100_five_cut800", "c100_five_cut700")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=("list", "check", "insert"))
    ap.add_argument("world", nargs="?")
    ap.add_argument("--arm", default="scalar", choices=ARMS,
                    help="scalar: the squeeze alone (thesis design, 24/29 in PyBullet); "
                         "hwd: + the internal hand wrench (25/29). A route runs only with the control it was trained with. "
                         "c100_*: routes TRAINED at the real size (100 mm cubes, 70 mm tri-pad, scalar squeeze; "
                         "cubes100/README.md): c100_narrow75 the 29 narrow75 worlds, c100_five / c100_five625 the "
                         "20-cube kit worlds at 75% / 62.5%")
    ap.add_argument("--config", default=None)
    ap.add_argument("--slow", type=float, default=1.0,
                    help="route, seat and turns this many times slower than trained (default 1)")
    pair.add_args(ap)
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])
    if a.command == "list":
        from .route import Route as R
        for arm in ARMS:
            for w in available(arm):
                print("  " + R(w, arm).summary())
        return
    if not a.world:
        ap.error("which world? (list)")
    cfg, path = load_config(a.config)
    print(f"  config: {os.path.abspath(path)}")
    (cmd_check if a.command == "check" else cmd_insert)(a, cfg)


def list_entry():
    main(["list"])


def check_entry():
    main(["check"] + sys.argv[1:])


def insert_entry():
    main(["insert"] + sys.argv[1:])


if __name__ == "__main__":
    main()
