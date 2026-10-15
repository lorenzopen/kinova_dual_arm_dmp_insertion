"""A backend with no robot: to check the sequence, the timing and the commanded
flange paths before touching the Kinova. NOT a simulation of the insertion.

The tools go exactly where they are commanded; the piece is a rigid box held
at the grasp pose until the pads squeeze it, then it moves with the hands; its
faces push back on each hand KP x penetration; the shelf floor carries it when
it reaches its target height. There are no neighbours, so no jam is possible.
Simulated time: one `wait()` = one control period, faster than real time.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from . import geometry as GE
from .controller import PICKUP_N

G = 9.81


class DryRun:
    def __init__(self, route, period=0.01, k_face=1500.0, k_floor=20000.0):
        self.r, self.period = route, float(period)
        self.k_face, self.k_floor = float(k_face), float(k_floor)
        self.t = 0.0
        a = route.axis / np.linalg.norm(route.axis)
        self.cmd = (route.centre - 0.3 * a, GE.quat_from_x(a), route.centre + 0.3 * a, GE.quat_from_x(-a))
        self.held = False                    # the pads carry the piece
        self.c, self.R = route.grasp_c.copy(), route.grasp_R
        self.local = None                    # piece pose in the left tool's frame, once held

    def now(self):
        return self.t

    def wait(self):
        self.t += self.period

    def tare(self):
        pass

    def command(self, tL, qL, tR, qR, k_tilt):
        self.cmd = (np.asarray(tL, float), qL, np.asarray(tR, float), qR)

    def read(self):
        pL, qL, pR, qR = self.cmd
        mid = 0.5 * (pL + pR)
        if self.held:                        # the piece rides between the two tools
            dc, dR = self.local
            self.c, self.R = mid + qL.apply(dc), qL * dR
        # each face: the tool's contact face against the piece's face plane
        n = {}
        a_body = self.r.grasp_R.inv().apply(self.r.axis)          # axis in the piece
        centre = self.c + self.R.apply(self.r.grasp_R.inv().apply(self.r.centre - self.r.grasp_c))
        a = self.R.apply(a_body) / np.linalg.norm(a_body)
        for side, p, q, sgn in (("L", pL, qL, -1.0), ("R", pR, qR, +1.0)):
            x = q.apply([1.0, 0.0, 0.0])
            face = p + (GE.PAD_X + GE.PAD_R) * x
            plane = centre + sgn * 0.5 * self.r.opening * a
            n[side] = self.k_face * max(0.0, float((face - plane) @ x))
        if not self.held and min(n.values()) > 0.5 * min(self.r.derived_N, PICKUP_N):
            self.held = True
            self.local = (qL.inv().apply(self.c - mid), qL.inv() * self.R)
        support = 0.0
        if self.held:
            depth = float(self.r.target[2] - self.c[2])
            if depth > 0:
                support = self.k_floor * depth
        carried = self.r.mass * G - support if self.held else 0.0
        xL, xR = qL.apply([1.0, 0.0, 0.0]), qR.apply([1.0, 0.0, 0.0])
        up = np.array([0.0, 0.0, 1.0])
        return dict(pL=pL, qL=qL.as_quat(), pR=pR, qR=qR.as_quat(),
                    FL=n["L"] * xL + 0.5 * carried * up, TL=np.zeros(3),
                    FR=n["R"] * xR + 0.5 * carried * up, TR=np.zeros(3))
