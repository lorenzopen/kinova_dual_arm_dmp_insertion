"""The two Kinova arms through ROS 2: flange targets out, wrench and flange poses in.

Written for the lab's own stack, `kinova_controller_pkgs` (`kinova_control`,
ROS 2 Jazzy), used AS IS - nothing of it is modified:

    ros2 launch kinova_control dual_cartesian.launch.py ee_frame:=end_effector_link cart_k:=...

which runs one `kinova_controller.py` per arm (namespaces /left .108, /right
.107): a Pinocchio Cartesian impedance in joint torque at 1 kHz on the Kortex
low-level API, gravity compensated from the URDF. From it this node uses:

    /<ns>/target_pose       PoseStamped   the ee_frame target, in the arm's base_link
    /<ns>/joint_states      JointState    250 Hz -> the flange pose, by the SAME
                                          Pinocchio forward kinematics on the SAME URDF
    /<ns>/estimated_wrench  WrenchStamped 250 Hz, from the joint torque sensors:
                                          the external wrench ON the arm, ee_frame
                                          axes, about the ee_frame origin
    /<ns>/zero_wrench       Trigger       tare (arm still, nothing touching)

That stack publishes no TF, hence the forward kinematics here.

The estimate is off by up to 8-9 N per arm away from the pose of the tare, the same
on every pass along the same path (2026-10-16: two empty runs agree within ~1 N).
`wrench_baseline:` in cell.yaml names an empty-handed `wrench_log` CSV recorded along
the route: each hand's force is then corrected by the baseline at the nearest flange
position, minus the baseline where the tare was made. Forces only, not moments. A real wrist F/T
sensor (`wrench_source: ft`, frame read from TF) is kept as the other option.

Everything inside the controller is in the SHELF frame; this file is the only
place that knows where the two robot bases are.
"""
import csv
import os
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .geometry import Frame

G_WORLD = np.array([0.0, 0.0, -9.81])        # shelf z is up


def _frame(cfg):
    return Frame(cfg["xyz"], cfg["quat"])


class _FK:
    """Flange pose from joint angles: Pinocchio on the controller's own URDF."""

    def __init__(self, urdf, ee_frame):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.fid = self.model.getFrameId(ee_frame)
        if self.fid == self.model.nframes:
            raise ValueError(f"frame {ee_frame} not in {urdf}")

    def pose(self, msg):
        q = self.pin.neutral(self.model)
        for name, angle in zip(msg.name, msg.position):
            if not self.model.existJointName(name):
                continue                     # gripper / finger joints
            j = self.model.joints[self.model.getJointId(name)]
            if j.nq == 2:                    # continuous joint: (cos, sin)
                q[j.idx_q], q[j.idx_q + 1] = np.cos(angle), np.sin(angle)
            else:
                q[j.idx_q] = angle
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        M = self.data.oMf[self.fid]
        return np.array(M.translation), Rot.from_matrix(np.array(M.rotation))


class _Baseline:
    """Empty-handed wrench vs flange position, one arm, from a `wrench_log` CSV:
    the base-axes force of the K samples nearest to a position, if within RADIUS."""
    K, RADIUS = 25, 0.03

    def __init__(self, path, side):
        from scipy.spatial import cKDTree
        with open(path, newline="") as fh:
            rows = [r for r in csv.DictReader(fh) if r["arm"] == side]
        if any(r["mark"] != "start" for r in rows):
            rows = [r for r in rows if r["mark"] != "start"]     # only after the tare
        if len(rows) < self.K:
            raise ValueError(f"{path}: only {len(rows)} samples for arm {side}")
        self.P = np.array([[float(r[k]) for k in ("px", "py", "pz")] for r in rows])
        self.F = np.array([[float(r[k]) for k in ("Fbx", "Fby", "Fbz")] for r in rows])
        self.tree = cKDTree(self.P)

    def force(self, p):
        """Base-axes force at flange position `p` (base frame), or None if too far."""
        d, i = self.tree.query(np.asarray(p, float), k=self.K)
        if d[0] > self.RADIUS:
            return None
        return self.F[i].mean(0)


def _resolve(path):
    if os.path.isabs(path) or os.path.isfile(path):
        return path
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), path)


class _Arm:
    def __init__(self, node, name, cfg):
        self.name, self.cfg = name, cfg
        self.base = cfg["base_frame"]
        self.shelf_in_base = _frame(cfg["shelf_in_base"])      # pose of the shelf frame in the base
        self.base_in_shelf = self.shelf_in_base.inverse()
        self.tool_in_ee = _frame(cfg["tool_in_ee"])             # our tool frame (x = push) in the ee frame
        self.ee_in_tool = self.tool_in_ee.inverse()
        self.pub = node.create_publisher(PoseStamped, cfg["target_topic"], 10)
        self.joints = self.wrench = None
        node.create_subscription(JointState, cfg["joint_states_topic"], self._joints, 10)
        node.create_subscription(WrenchStamped, cfg["wrench_topic"], self._wrench, 10)
        self.fk = _FK(cfg["urdf_path"], cfg["ee_frame"])
        self.zero = (node.create_client(Trigger, cfg["zero_wrench_service"])
                     if cfg.get("zero_wrench_service") else None)
        self.source = cfg.get("wrench_source", "estimated")
        self.bias = (np.zeros(3), np.zeros(3))
        self.m_tool = float(cfg.get("tool_mass", 0.0))
        self.c_tool = np.asarray(cfg.get("tool_com_in_ft", [0.0, 0.0, 0.0]), float)
        self.sign = float(cfg.get("ft_sign", 1.0))
        self.baseline, self.b0 = None, np.zeros(3)
        self.ticks = self.misses = 0

    def _joints(self, msg):
        self.joints = msg

    def _wrench(self, msg):
        self.wrench = msg


class RosBackend(Node):
    def __init__(self, cfg):
        rclpy.init()
        super().__init__("kinova_dmp_insertion")
        self.cfg = cfg
        self.period = 1.0 / float(cfg["rate_hz"])
        self.arms = {"L": _Arm(self, "left", cfg["left"]), "R": _Arm(self, "right", cfg["right"])}
        if cfg.get("wrench_baseline"):
            path = _resolve(cfg["wrench_baseline"])
            for side, arm in self.arms.items():
                arm.baseline = _Baseline(path, side)
                z = arm.baseline.P[:, 2]
                print(f"  wrench baseline {arm.name}: {len(z)} samples, flange z {z.min():.2f}..{z.max():.2f} m"
                      f" (base), from {path}")
        if any(a.source == "ft" for a in self.arms.values()):
            from tf2_ros import Buffer, TransformListener
            self.tf = Buffer()
            self._tfl = TransformListener(self.tf, self)
        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self)
        threading.Thread(target=self._exec.spin, daemon=True).start()
        deadline = time.monotonic() + 10.0
        while any(a.joints is None or a.wrench is None for a in self.arms.values()):
            if time.monotonic() > deadline:
                missing = [t for a in self.arms.values()
                           for t, m in ((a.cfg["joint_states_topic"], a.joints),
                                        (a.cfg["wrench_topic"], a.wrench)) if m is None]
                raise RuntimeError("nothing received on " + ", ".join(missing)
                                   + " - is dual_cartesian.launch.py running?")
            time.sleep(0.05)
        self._next = time.monotonic()

    # ---- clock
    def now(self):
        return time.monotonic()

    def wait(self):
        self._next += self.period
        dt = self._next - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        else:
            self._next = time.monotonic()   # late: do not try to catch up

    # ---- out
    def command(self, tL, qL, tR, qR, k_tilt):
        """`k_tilt` is not sent: the lab controller takes its stiffness at launch only."""
        for arm, p, q in ((self.arms["L"], tL, qL), (self.arms["R"], tR, qR)):
            pb, qb = arm.shelf_in_base.apply(np.asarray(p, float), q)       # tool pose in the base
            pe = pb + qb.apply(arm.ee_in_tool.t)                          # ee pose in the base
            qe = qb * arm.ee_in_tool.R
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = arm.base
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pe)
            x, y, z, w = qe.as_quat()          # unit quaternion: the controller does not normalise
            msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
            msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
            arm.pub.publish(msg)

    # ---- in
    def _ee_pose(self, arm):
        """(ee position, ee rotation) in the base, by forward kinematics."""
        return arm.fk.pose(arm.joints)

    def _tool_pose(self, arm):
        pe, qe = self._ee_pose(arm)
        pt, qt = pe + qe.apply(arm.tool_in_ee.t), qe * arm.tool_in_ee.R
        return arm.base_in_shelf.apply(pt, qt)                  # tool in shelf

    def tare(self):
        """Zero both wrench readings with the tools in free air."""
        time.sleep(0.5)
        for arm in self.arms.values():
            if arm.source == "estimated":
                if arm.zero is not None and arm.zero.wait_for_service(timeout_sec=2.0):
                    fut = arm.zero.call_async(Trigger.Request())
                    t0 = time.monotonic()
                    while not fut.done() and time.monotonic() - t0 < 3.0:
                        time.sleep(0.02)
                    time.sleep(0.4)          # it averages 50 samples (~200 ms)
                else:
                    self.get_logger().warn(f"{arm.name}: no zero_wrench service, not tared")
                if arm.baseline is not None:
                    b0 = arm.baseline.force(self._ee_pose(arm)[0])
                    if b0 is None:
                        self.get_logger().warn(f"{arm.name}: tare pose off the wrench baseline, "
                                               f"correction relative to its own tare")
                    arm.b0 = np.zeros(3) if b0 is None else b0
            else:
                f, t, _ps, qs = self._ft_raw(arm)
                g = arm.m_tool * G_WORLD
                arm.bias = (qs.inv().apply(f - g), qs.inv().apply(t - np.cross(qs.apply(arm.c_tool), g)))

    def _ft_raw(self, arm):
        """F/T sensor option: (force, moment at the sensor origin, sensor position, rotation), shelf frame."""
        w = arm.wrench
        t = self.tf.lookup_transform(arm.base, w.header.frame_id, Time())
        tr, ro = t.transform.translation, t.transform.rotation
        ps, qs = arm.base_in_shelf.apply(np.array([tr.x, tr.y, tr.z]),
                                         Rot.from_quat([ro.x, ro.y, ro.z, ro.w]))
        f = arm.sign * np.array([w.wrench.force.x, w.wrench.force.y, w.wrench.force.z])
        m = arm.sign * np.array([w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z])
        return f, m, ps, qs

    def read(self):
        out = {}
        for s, arm in self.arms.items():
            pt, qt = self._tool_pose(arm)
            w = arm.wrench
            if arm.source == "estimated":
                # the external wrench ON the arm, ee axes, about the ee origin; the tool's
                # weight is the URDF's business (and what is left of it, the tare's)
                pb, qb = self._ee_pose(arm)
                pe, qe = arm.base_in_shelf.apply(pb, qb)
                f_on_tool = qe.apply([w.wrench.force.x, w.wrench.force.y, w.wrench.force.z])
                if arm.baseline is not None:
                    arm.ticks += 1
                    b = arm.baseline.force(pb)
                    if b is None:
                        arm.misses += 1
                        if arm.misses == 1:
                            self.get_logger().warn(f"{arm.name}: flange off the wrench baseline "
                                                   f"(> {1000 * _Baseline.RADIUS:.0f} mm): not corrected there")
                    else:
                        f_on_tool = f_on_tool - arm.base_in_shelf.R.apply(b - arm.b0)
                m_on_tool = qe.apply([w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z])
                origin = pe
            else:
                f, m, origin, qs = self._ft_raw(arm)
                g = arm.m_tool * G_WORLD
                f_on_tool = f - qs.apply(arm.bias[0]) - g
                m_on_tool = m - qs.apply(arm.bias[1]) - np.cross(qs.apply(arm.c_tool), g)
            # the hand on the piece is the opposite; moment moved to the tool origin
            F = -np.asarray(f_on_tool)
            T = -np.asarray(m_on_tool) + np.cross(origin - pt, F)
            out.update({f"p{s}": pt, f"q{s}": qt.as_quat(), f"F{s}": F, f"T{s}": T})
        return out

    def close(self):
        for arm in self.arms.values():
            if arm.baseline is not None and arm.ticks:
                print(f"  wrench baseline {arm.name}: corrected {100 * (1 - arm.misses / arm.ticks):.0f}% "
                      f"of {arm.ticks} readings")
        self._exec.shutdown()
        self.destroy_node()
        rclpy.shutdown()
