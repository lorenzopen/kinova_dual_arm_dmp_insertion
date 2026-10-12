"""Measure where the shelf is from one arm: `shelf_in_base` for config/cell.yaml.

The arm is the measuring tool. A pointer of known length sticks out of the
flange along z of `end_effector_link`; its tip is put by hand on points of the
shelf whose coordinates in the SHELF frame are known (origin on the shelf floor
at the left edge of column 0, x along the columns, z up). For each point the tip
is read by the SAME forward kinematics `insert` uses (Pinocchio on the
controller's URDF), averaged over a short window. Three points not on one line
fix the frame; five or six give a least-squares fit and its residual.

This node only LISTENS to /<ns>/joint_states: it publishes nothing and moves
nothing. The lab stack must be running (low stiffness, so the arm can be pushed
by hand), e.g.

    ros2 launch kinova_control dual_cartesian.launch.py ee_frame:=end_effector_link \
        cart_k:='[100.0, 100.0, 100.0, 10.0, 10.0, 10.0]'

then, venv active, from the package folder:

    python -m kinova_dual_arm_dmp_insertion.calibrate_cell left  --tip 120
    python -m kinova_dual_arm_dmp_insertion.calibrate_cell right --tip 120
    python -m kinova_dual_arm_dmp_insertion.calibrate_cell left  --tip 120 --verify
    python -m kinova_dual_arm_dmp_insertion.calibrate_cell left  --refit logs/calib_left_<time>.json

`--verify` prints where the tip is in the shelf frame with the `shelf_in_base`
now in the config: touch the same point with both arms, they must agree to a
few mm. `--refit` redoes the fit from a saved run, no robot.
"""
import argparse
import collections
import json
import os
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from .main import load_config


def fit(shelf_pts, base_pts):
    """Kabsch: the rigid (R, t) with base = R shelf + t, i.e. the shelf frame in the base."""
    P, Q = np.asarray(shelf_pts, float), np.asarray(base_pts, float)
    pc, qc = P.mean(axis=0), Q.mean(axis=0)
    s = np.linalg.svd(P - pc, compute_uv=False)
    if len(P) < 3 or s[1] < 0.1 * s[0]:
        raise ValueError("the points are (nearly) on one line: add one off that line")
    U, _, Vt = np.linalg.svd((P - pc).T @ (Q - qc))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = qc - R @ pc
    res = np.linalg.norm((P @ R.T + t) - Q, axis=1)
    return Rot.from_matrix(R), t, res


def report(arm, names, shelf_pts, base_pts):
    R, t, res = fit(shelf_pts, base_pts)
    print(f"\n  {arm}: {len(res)} points")
    for n, p, r in zip(names, shelf_pts, res):
        print(f"    {n:>10s}  shelf {np.round(1000 * np.asarray(p), 1).tolist()} mm  residual {1000 * r:.1f} mm")
    print(f"  residual: rms {1000 * np.sqrt(np.mean(res ** 2)):.1f} mm, max {1000 * res.max():.1f} mm"
          + ("  (3 points: no redundancy, the residual says nothing - add more)" if len(res) == 3 else ""))
    tilt = np.degrees(np.arccos(np.clip(R.apply([0, 0, 1])[2], -1, 1)))
    print(f"  rotation from the identity {np.degrees(R.magnitude()):.1f} deg; "
          f"shelf z vs base z {tilt:.1f} deg (a level base on a level shelf: ~0)")
    y = R.apply([0, 1, 0])
    i = int(np.argmax(np.abs(y)))
    print(f"  shelf y (the push axis) in base_link: {np.round(y, 3).tolist()}"
          + ("" if abs(y[i]) > 0.95 else "  WARNING: not along a base axis, cart_k cannot match the sim"))
    x, yy, z, w = R.as_quat()
    print(f"\n  config/cell.yaml, under `{arm}:`\n"
          f"  shelf_in_base: {{xyz: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}], "
          f"quat: [{x:.7f}, {yy:.7f}, {z:.7f}, {w:.7f}]}}\n")


class _Listener:
    """The pointer tip in base_link, from the last `window` seconds of joint_states."""

    def __init__(self, acfg, tip, window):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from .ros_backend import _FK
        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("kinova_calibrate_cell")
        self.fk = _FK(acfg["urdf_path"], acfg["ee_frame"])
        self.tip, self.window = np.array([0.0, 0.0, tip]), window
        self.buf, self.lock = collections.deque(), threading.Lock()
        self.node.create_subscription(JointState, acfg["joint_states_topic"], self._cb, 50)
        self.exe = SingleThreadedExecutor()
        self.exe.add_node(self.node)
        threading.Thread(target=self.exe.spin, daemon=True).start()
        t0 = time.monotonic()
        while not self.buf:
            if time.monotonic() - t0 > 10.0:
                raise RuntimeError(f"nothing on {acfg['joint_states_topic']} - is the lab stack running?")
            time.sleep(0.05)

    def _cb(self, msg):
        now = time.monotonic()
        with self.lock:
            self.buf.append((now, msg))
            while self.buf and now - self.buf[0][0] > self.window:
                self.buf.popleft()

    def tip_in_base(self):
        """(mean tip position, its spread) over the window."""
        with self.lock:
            msgs = [m for _, m in self.buf]
        pts = []
        for m in msgs:
            pe, qe = self.fk.pose(m)
            pts.append(pe + qe.apply(self.tip))
        pts = np.array(pts)
        return pts.mean(axis=0), float(np.max(np.linalg.norm(pts - pts.mean(axis=0), axis=1)))

    def close(self):
        self.exe.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown()


def read_point(listener):
    p, spread = listener.tip_in_base()
    if spread > 0.001:
        print(f"    the tip moved {1000 * spread:.1f} mm during the reading: hold it still and repeat")
        return None
    return p


def calibrate(a, acfg, path):
    lis = _Listener(acfg, a.tip / 1000.0, a.window)
    names, shelf_pts, base_pts = [], [], []
    print(f"  {a.arm}: tip {a.tip:.1f} mm along z of {acfg['ee_frame']}.\n"
          "  Put the tip on a shelf point, hold it still, type its shelf coordinates in mm\n"
          "  (x y z [name]).  u = undo the last point, f = fit now, q = fit and quit.")
    try:
        while True:
            s = input(f"  point {len(names) + 1}> ").strip()
            if s in ("q", "f"):
                if len(names) >= 3:
                    report(a.arm, names, shelf_pts, base_pts)
                else:
                    print("    at least 3 points")
                if s == "q":
                    break
                continue
            if s == "u":
                if names:
                    print(f"    dropped {names.pop()}")
                    shelf_pts.pop(), base_pts.pop()
                continue
            parts = s.split()
            try:
                xyz = [float(v) / 1000.0 for v in parts[:3]]
                assert len(xyz) == 3
            except (ValueError, AssertionError):
                print("    x y z in mm, e.g. `0 0 0 origin`")
                continue
            p = read_point(lis)
            if p is None:
                continue
            names.append(parts[3] if len(parts) > 3 else f"p{len(names) + 1}")
            shelf_pts.append(xyz)
            base_pts.append(p.tolist())
            print(f"    {names[-1]}: tip in base_link {np.round(1000 * p, 1).tolist()} mm")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        lis.close()
    if names:
        os.makedirs(a.log_dir, exist_ok=True)
        out = os.path.join(a.log_dir, time.strftime(f"calib_{a.arm}_%Y%m%d_%H%M%S.json"))
        with open(out, "w") as fh:
            json.dump({"arm": a.arm, "tip_mm": a.tip, "urdf": acfg["urdf_path"], "config": path,
                       "names": names, "shelf_m": shelf_pts, "base_m": base_pts}, fh, indent=1)
        print(f"  points saved: {out}  (redo the fit with --refit)")


def verify(a, acfg):
    from .geometry import Frame
    base_in_shelf = Frame(acfg["shelf_in_base"]["xyz"], acfg["shelf_in_base"]["quat"]).inverse()
    lis = _Listener(acfg, a.tip / 1000.0, a.window)
    print(f"  {a.arm}: the tip in the SHELF frame with the config's shelf_in_base. Enter = read, q = quit.")
    try:
        while input("  > ").strip() != "q":
            p = read_point(lis)
            if p is not None:
                ps, _ = base_in_shelf.apply(p, Rot.identity())
                print(f"    tip in shelf frame {np.round(1000 * ps, 1).tolist()} mm")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        lis.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("arm", choices=("left", "right"))
    ap.add_argument("--tip", type=float, default=0.0,
                    help="[mm] pointer tip along z of the ee_frame (0 = the flange centre)")
    ap.add_argument("--window", type=float, default=0.5, help="[s] readings averaged per point")
    ap.add_argument("--verify", action="store_true", help="print the tip in the shelf frame (config's shelf_in_base)")
    ap.add_argument("--refit", metavar="JSON", help="fit again from a saved run, no robot")
    ap.add_argument("--config", default=None)
    a = ap.parse_args(argv)
    if a.refit:
        with open(a.refit) as fh:
            d = json.load(fh)
        report(d["arm"], d["names"], d["shelf_m"], d["base_m"])
        return
    cfg, path = load_config(a.config)
    a.log_dir = cfg["log_dir"]
    print(f"  config: {os.path.abspath(path)}")
    (verify if a.verify else calibrate)(a, cfg[a.arm], path)


if __name__ == "__main__":
    main()
