"""Tri-pad geometry, frames and route pacing. numpy + scipy only.

Every number is the simulated apparatus's (10_tripad_passivo/lib/tool.py,
lib/wrist.py, tetris/insertion.py); the real tool must match them, or the
trained routes are routes for another tool.

THE TOOL FRAME: origin at the flange centre, local x = the push direction (the
pad normal, pointing AT the piece), the contact face at x = PAD_X + PAD_R.

THE REAL TOOL IS NOT THE SIMULATED ONE (Lorenzo, 2026-10-12): two printed
plastic plates, a central spring between them and a SPONGE on the face, where
the simulation has three spheres on zero-length springs. The convex program
still reasons on three points (CONSTRAINT 1): here they are three points on a
circle of radius R_TRI on the sponge face, taken from geometry - a declared
approximation of a distributed contact. `set_tool` puts the measured numbers in
(config/cell.yaml, `tool:`); the defaults are the simulated tool's.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

PAD_X = 0.020      # [m] sim: pad plane ahead of the flange
R_TRI = 0.030      # [m] sim: circumradius of the pad triangle (CONSTRAINT 1)
PAD_R = 0.010      # [m] sim: pad sphere radius (real tool: sponge thickness)
PADS_LOCAL = None


def set_tool(pad_x=0.020, r_tri=0.030, pad_r=0.010):
    """The three contact points on the face, in the tool frame."""
    global PAD_X, R_TRI, PAD_R, PADS_LOCAL
    PAD_X, R_TRI, PAD_R = float(pad_x), float(r_tri), float(pad_r)
    PADS_LOCAL = np.array([[PAD_X, 0.0, R_TRI],
                           [PAD_X, +R_TRI * np.sqrt(3) / 2, -R_TRI / 2],
                           [PAD_X, -R_TRI * np.sqrt(3) / 2, -R_TRI / 2]])


set_tool()


def quat_from_x(xdir):
    """Tool attitude with local x along `xdir` (lib/wrist.py, roll 0)."""
    x = np.asarray(xdir, float) / np.linalg.norm(xdir)
    a = np.array([0.0, 0.0, 1.0])
    if abs(a @ x) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    y = np.cross(a, x)
    y /= np.linalg.norm(y)
    return Rot.from_matrix(np.stack([x, y, np.cross(x, y)], axis=1))


def pad_contacts(position, rotation):
    """The three pad contact points of a hand and the outward normal of the face
    they touch (pointing from the piece back at the hand), shelf frame.

    The real tool has no contact sensor, so the points come from geometry: the
    sphere surface ahead of each pad centre. The plate leaning on its springs is
    ignored (the plate pose is the flange pose) - a declared approximation."""
    x = rotation.apply([1.0, 0.0, 0.0])
    pts = np.asarray(position, float) + rotation.apply(PADS_LOCAL) + PAD_R * x
    return pts, np.tile(-x, (3, 1))


def segment_times(centres, rotations, speed, turn_rate):
    """Cumulative waypoint times that keep every segment under `speed` [m/s] AND
    `turn_rate` [rad/s] (tetris/insertion.py): CONSTRAINT 2, paced by distance
    and angle, never by a fixed duration."""
    t = [0.0]
    for k in range(1, len(centres)):
        d = float(np.linalg.norm(np.asarray(centres[k]) - np.asarray(centres[k - 1])))
        a = float(np.linalg.norm((rotations[k - 1].inv() * rotations[k]).as_rotvec()))
        t.append(max(d / speed, a / turn_rate))
    return np.cumsum(t)


def interpolate(times, centres, rotations, t):
    """Object pose at time t on a paced route (linear + slerp per segment)."""
    t = min(max(t, 0.0), float(times[-1]))
    k = int(np.searchsorted(times, t, side="right") - 1)
    k = min(max(k, 0), len(times) - 1)
    j = min(k + 1, len(times) - 1)
    span = times[j] - times[k]
    mix = (t - times[k]) / span if span > 0 else 0.0
    c = (1 - mix) * np.asarray(centres[k]) + mix * np.asarray(centres[j])
    r = rotations[k] * Rot.from_rotvec(mix * (rotations[k].inv() * rotations[j]).as_rotvec())
    return c, r, k, j, mix


class Frame:
    """A rigid transform: p_parent = R p_child + t."""

    def __init__(self, t=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0)):
        self.t = np.asarray(t, float)
        self.R = Rot.from_quat(q)

    def apply(self, position, rotation):
        return self.t + self.R.apply(position), self.R * rotation

    def inverse(self):
        inv = Frame()
        inv.R = self.R.inv()
        inv.t = -inv.R.apply(self.t)
        return inv
