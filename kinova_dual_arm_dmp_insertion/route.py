"""One exported narrow75 route (routes/<arm>/<world>.json), read into numpy.

The file is written on the simulation PC by
`dmp_realistic_6x6/tight/kinova_export/export_routes.py`, which replays the
trained route in PyBullet with `38_learn_pybullet.py --export-route`. All poses
are in the SHELF frame (the simulator's world): floor at z = 0, the shelf plane
is x-z, the pad axis along y.
"""
import json
import os

import numpy as np
from scipy.spatial.transform import Rotation as Rot

ROUTES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "routes")


def available(arm="scalar"):
    d = os.path.join(ROUTES, arm)
    return sorted(f[:-5] for f in os.listdir(d) if f.endswith(".json")) if os.path.isdir(d) else []


class Route:
    def __init__(self, world, arm="scalar"):
        path = world if world.endswith(".json") else os.path.join(ROUTES, arm, world + ".json")
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
        self.raw, self.path = d, path
        self.world, self.piece, self.arm = d["world"], d["piece"], arm
        self.hand_wrench = d["hand_wrench"]            # "off" or "internal-descent"
        r = d["route"]
        self.centres = np.asarray(r["centres"], float)  # [N, 3], last one = the slot
        self.rots = [Rot.from_quat(q) for q in r["quats"]]
        self.wext = [(np.asarray(f, float), np.asarray(t, float))
                     for f, t in zip(r["wext_force"], r["wext_torque"])]
        self.target = np.asarray(d["target"]["position"], float)
        self.R_target = Rot.from_quat(d["target"]["quat"])
        self.grasp_c = np.asarray(d["grasp_pose"]["position"], float)   # piece held here when pads close
        self.grasp_R = Rot.from_quat(d["grasp_pose"]["quat"])
        h = d["hands"]
        self.centre = np.asarray(h["centre"], float)    # grasp centre, shelf frame
        self.axis = np.asarray(h["axis"], float)         # LEFT hand pushes along +axis
        self.opening = float(h["opening"])
        self.reach = float(h["reach"])                   # open pose: centre -/+ reach * axis
        self.swapped = bool(h["swapped"])
        m = d["piece_model"]
        self.mass = float(m["mass"])
        self.r_plan = np.asarray(m["r_plan"], float)     # grasp centre from the CoM, piece frame
        self.derived_N = float(m["derived_N"])           # pickup squeeze and SOCP fallback, per hand
        self.cells = np.asarray(m["cells"], float)
        self.c = d["constants"]
        self.simulated = d.get("simulated", {})
        self.neighbours = d["neighbours"]
        self.shelf = d["shelf"]

    def summary(self):
        s = self.simulated
        return (f"{self.world} ({self.piece}, arm {self.arm}, hand wrench {self.hand_wrench}): "
                f"{len(self.centres)} waypoints, mass {self.mass:.3f} kg, opening "
                f"{1000 * self.opening:.0f} mm, pickup squeeze {self.derived_N:.1f} N/hand; "
                f"PyBullet placed {s.get('placed_mm', float('nan')):.1f} mm, "
                f"slip {s.get('slip_mm', float('nan')):.1f} mm")
