"""The two Kinova arms through ROS 2: flange targets out, F/T and flange poses in.

What the arm side must provide (config/cell.yaml names every topic and frame):
  * a CARTESIAN IMPEDANCE controller per arm that takes a target pose
    (geometry_msgs/PoseStamped, in the arm's base frame) - e.g. the FZI
    cartesian_compliance_controller, or a joint-torque impedance on the Kortex
    low-level API. Stiffness 1500 N/m, 40 Nm/rad about the push axis, 3-8 Nm/rad
    tilt: the simulated wrist (lib/wrist.py). The tilt value in use is also
    published on `stiffness_topic`, for controllers that can take it live;
  * a wrist F/T sensor per arm (geometry_msgs/WrenchStamped, frame in TF);
  * TF from each base frame to its end-effector frame and to its F/T frame.

Everything inside is in the SHELF frame; this file is the only place that
knows where the two robot bases are.
"""
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

from .geometry import Frame

G_WORLD = np.array([0.0, 0.0, -9.81])        # shelf z is up


def _frame(cfg):
    return Frame(cfg["xyz"], cfg["quat"])


class _Arm:
    def __init__(self, node, name, cfg):
        self.name, self.cfg = name, cfg
        self.base, self.ee = cfg["base_frame"], cfg["ee_frame"]
        self.shelf_in_base = _frame(cfg["shelf_in_base"])      # pose of the shelf frame in the base
        self.base_in_shelf = self.shelf_in_base.inverse()
        self.tool_in_ee = _frame(cfg["tool_in_ee"])             # our tool frame (x = push) in the ee frame
        self.ee_in_tool = self.tool_in_ee.inverse()
        self.pub = node.create_publisher(PoseStamped, cfg["target_topic"], 10)
        self.pub_k = (node.create_publisher(Float64MultiArray, cfg["stiffness_topic"], 10)
                      if cfg.get("stiffness_topic") else None)
        node.create_subscription(WrenchStamped, cfg["wrench_topic"], self._wrench, 10)
        self.wrench = None                   # latest WrenchStamped
        self.bias = (np.zeros(3), np.zeros(3))
        self.m_tool = float(cfg["tool_mass"])
        self.c_tool = np.asarray(cfg["tool_com_in_ft"], float)
        self.sign = float(cfg.get("ft_sign", 1.0))

    def _wrench(self, msg):
        self.wrench = msg


class RosBackend(Node):
    def __init__(self, cfg):
        rclpy.init()
        super().__init__("kinova_dmp_insertion")
        self.cfg = cfg
        self.period = 1.0 / float(cfg["rate_hz"])
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        self.arms = {"L": _Arm(self, "left", cfg["left"]), "R": _Arm(self, "right", cfg["right"])}
        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self)
        threading.Thread(target=self._exec.spin, daemon=True).start()
        self._next = time.monotonic()
        self.k_stiff = cfg["stiffness"]
        deadline = time.monotonic() + 10.0
        while any(a.wrench is None for a in self.arms.values()):
            if time.monotonic() > deadline:
                raise RuntimeError("no F/T message on " + ", ".join(
                    a.cfg["wrench_topic"] for a in self.arms.values() if a.wrench is None))
            time.sleep(0.05)

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
        for arm, p, q in ((self.arms["L"], tL, qL), (self.arms["R"], tR, qR)):
            pb, qb = arm.shelf_in_base.apply(np.asarray(p, float), q)       # tool pose in the base
            pe = pb + qb.apply(arm.ee_in_tool.t)                          # ee pose in the base
            qe = qb * arm.ee_in_tool.R
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = arm.base
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pe)
            x, y, z, w = qe.as_quat()
            msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
            msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
            arm.pub.publish(msg)
            if arm.pub_k is not None:
                k = self.k_stiff
                arm.pub_k.publish(Float64MultiArray(data=[k["trans"]] * 3 + [k["torsion"], k_tilt, k_tilt]))

    # ---- in
    def _lookup(self, parent, child):
        t = self.tf.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.05))
        tr, ro = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), Rot.from_quat([ro.x, ro.y, ro.z, ro.w])

    def _tool_pose(self, arm):
        pe, qe = self._lookup(arm.base, arm.ee)                  # ee in base
        pt, qt = pe + qe.apply(arm.tool_in_ee.t), qe * arm.tool_in_ee.R
        return arm.base_in_shelf.apply(pt, qt)                  # tool in shelf

    def _ft_raw(self, arm):
        """(force, moment at the sensor origin), shelf frame, and the sensor pose."""
        w = arm.wrench
        ps, qs = self._lookup(arm.base, w.header.frame_id)
        ps, qs = arm.base_in_shelf.apply(ps, qs)
        f = arm.sign * np.array([w.wrench.force.x, w.wrench.force.y, w.wrench.force.z])
        t = arm.sign * np.array([w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z])
        return f, t, ps, qs

    def tare(self):
        """Zero both sensors with the tools in free air: the offset left after
        removing the tool's own weight at THIS attitude, kept in the sensor frame."""
        time.sleep(0.5)
        for arm in self.arms.values():
            f, t, _ps, qs = self._ft_raw(arm)
            g = arm.m_tool * G_WORLD
            arm.bias = (qs.inv().apply(f - g), qs.inv().apply(t - np.cross(qs.apply(arm.c_tool), g)))

    def read(self):
        out = {}
        for s, arm in self.arms.items():
            pt, qt = self._tool_pose(arm)
            f, t, ps, qs = self._ft_raw(arm)
            g = arm.m_tool * G_WORLD
            # the load on the tool beyond the sensor, minus its own weight = what the piece does to it
            f_piece = f - qs.apply(arm.bias[0]) - g
            t_piece = t - qs.apply(arm.bias[1]) - np.cross(qs.apply(arm.c_tool), g)
            # the hand on the piece is the opposite; moment moved to the tool origin
            F = -f_piece
            T = -t_piece + np.cross(ps - pt, -f_piece)
            out.update({f"p{s}": pt, f"q{s}": qt.as_quat(), f"F{s}": F, f"T{s}": T})
        return out

    def hold(self):
        """Stop streaming: each impedance controller keeps its last target."""
        pass

    def close(self):
        self._exec.shutdown()
        self.destroy_node()
        rclpy.shutdown()
