"""Record what the two arms' wrench estimate says, with nothing commanded. READ ONLY.

    python -m kinova_dual_arm_dmp_insertion.wrench_log [--tare] [--config cell.yaml]

Subscribes to joint_states and estimated_wrench of both arms (topics and URDF from
cell.yaml) and writes one CSV row per wrench message: joint angles and efforts, the
raw wrench (ee_frame axes, as published) and its force in the base axes, the flange
position by forward kinematics. It publishes no target: move the arms with whatever
else is running (the controller's own hold, a follow script, by hand in free air).

Type a word + Enter while it runs to put a mark in the `mark` column (e.g. `low`,
`high`, `touch`); Ctrl-C stops. --tare calls zero_wrench on both first (arms still,
nothing touching). The summary at the end gives, per mark, the mean base force of
each arm: with nothing touching it should stay near 0 N wherever the arm is.
"""
import argparse
import csv
import os
import sys
import threading
import time

import numpy as np

from .main import load_config

COLS_ARM = ([f"q{i}" for i in range(1, 8)] + [f"tau{i}" for i in range(1, 8)]
            + "fx fy fz tx ty tz Fbx Fby Fbz px py pz".split())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--tare", action="store_true", help="zero_wrench on both arms before logging")
    ap.add_argument("--out", default=None, help="CSV path (default logs/wrench_<time>.csv)")
    a = ap.parse_args(argv)
    cfg, path = load_config(a.config)

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import WrenchStamped
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from .ros_backend import _FK

    rclpy.init()
    node = Node("kinova_wrench_log")
    sides = {"L": cfg["left"], "R": cfg["right"]}
    joints = {s: None for s in sides}
    fk = {s: _FK(c["urdf_path"], c["ee_frame"]) for s, c in sides.items()}
    mark = ["start"]
    lock = threading.Lock()

    os.makedirs(cfg["log_dir"], exist_ok=True)
    out = a.out or os.path.join(cfg["log_dir"], time.strftime("wrench_%Y%m%d_%H%M%S.csv"))
    fh = open(out, "w", newline="")
    w = csv.writer(fh)
    w.writerow(["t", "arm", "mark"] + COLS_ARM)
    rows = {s: [] for s in sides}            # (mark, Fb) for the summary

    def on_joints(s):
        def cb(msg):
            joints[s] = msg
        return cb

    def on_wrench(s):
        def cb(msg):
            js = joints[s]
            if js is None:
                return
            pe, qe = fk[s].pose(js)
            f = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
            t = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
            Fb = qe.apply(f)
            q = list(js.position[:7]) + [np.nan] * (7 - len(js.position[:7]))
            tau = list(js.effort[:7]) + [np.nan] * (7 - len(js.effort[:7]))
            with lock:
                w.writerow([f"{time.monotonic():.4f}", s, mark[0], *np.round(q, 5), *np.round(tau, 3),
                            *np.round(f, 3), *np.round(t, 4), *np.round(Fb, 3), *np.round(pe, 5)])
                rows[s].append((mark[0], Fb, pe))
        return cb

    for s, c in sides.items():
        node.create_subscription(JointState, c["joint_states_topic"], on_joints(s), 50)
        node.create_subscription(WrenchStamped, c["wrench_topic"], on_wrench(s), 50)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    if a.tare:
        for s, c in sides.items():
            cli = node.create_client(Trigger, c["zero_wrench_service"])
            if not cli.wait_for_service(timeout_sec=2.0):
                print(f"  {s}: no {c['zero_wrench_service']}, not tared")
                continue
            fut = cli.call_async(Trigger.Request())
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3.0:
                time.sleep(0.02)
            print(f"  {s}: tared")
        time.sleep(0.5)
        with lock:
            mark[0] = "tared"

    print(f"  logging to {out}\n  type a mark + Enter (e.g. low / high / touch), Ctrl-C to stop")
    try:
        for line in sys.stdin:
            with lock:
                mark[0] = line.strip() or mark[0]
            print(f"  mark: {mark[0]}")
    except KeyboardInterrupt:
        pass
    finally:
        with lock:
            fh.close()
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    print("\n  mean base force with each mark [N] (nothing touching: should stay near 0)")
    for s in sides:
        seen = []
        for m, _, _ in rows[s]:
            if m not in seen:
                seen.append(m)
        for m in seen:
            Fb = np.array([r[1] for r in rows[s] if r[0] == m])
            pz = np.mean([r[2][2] for r in rows[s] if r[0] == m])
            print(f"  {s} {m:>10s}: F = {np.round(Fb.mean(0), 1).tolist()}  |F| {np.linalg.norm(Fb.mean(0)):.1f}"
                  f"  (sd {np.round(Fb.std(0), 1).tolist()}, z flange {pz:.3f} m, {len(Fb)} samples)")
    print(f"  log: {out}")


if __name__ == "__main__":
    main()
