"""Where the shelf is for the real PAIR, without a calibration: `shelf_in_base` of both arms.

THE CELL (Lorenzo, 2026-10-13): two Kinova Gen3, bases parallel (same x
forward, same height), the LEFT base at y = +SEP (0.60 m) in the RIGHT base's
frame. The tools face each other: the right one looks along base +y (to its
left), the left one along base -y (to its right).

The route's push axis (the LEFT tool pushes along +axis, shelf +y) is turned
about z onto base -y, so the shelf frame sits turned ~180 deg in both bases.
The piece path runs in an x-z plane half way between the two bases: its mean x
is `forward` (in front of the bases), the grasp centre at y = SEP / 2 from the
right base, the heights as in the shelf frame (+ `lift`).

This is the nominal placement: the real offset between the two bases (and any
yaw between them) is what `calibrate_cell.py` measures. Where the two disagree,
the calibration wins (do not pass `--sep` then).
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

SEP = 0.60          # [m] left base at +y of the right base
FORWARD = 0.40      # [m] base x of the piece path (its mean)


def place(cfg, route, sep=SEP, forward=FORWARD, lift=0.0, shelf_x=None):
    """A copy of `cfg` with `shelf_in_base` of both arms set for the nominal pair.
    `shelf_x` [m]: the shelf origin at this base x for EVERY world (the shelf stays
    put on the bench); without it the piece path of this route is centred on `forward`."""
    a = route.axis / np.linalg.norm(route.axis)
    turn = Rot.from_rotvec([0.0, 0.0, -0.5 * np.pi - np.arctan2(a[1], a[0])])   # +axis -> base -y
    obj = np.vstack([route.grasp_c, route.centres, route.target])
    moved_x = turn.apply(obj)[:, 0].mean()
    centre = turn.apply(route.centre)
    x = shelf_x if shelf_x is not None else forward - moved_x
    t_right = np.array([x, 0.5 * sep - centre[1], lift])
    t_left = t_right - np.array([0.0, sep, 0.0])
    q = turn.as_quat().tolist()
    out = dict(cfg)
    out["right"] = dict(cfg["right"], shelf_in_base={"xyz": t_right.tolist(), "quat": q})
    out["left"] = dict(cfg["left"], shelf_in_base={"xyz": t_left.tolist(), "quat": q})
    return out


def describe(cfg, route=None):
    lines = []
    for side in ("left", "right"):
        s = cfg[side]["shelf_in_base"]
        ang = np.degrees(Rot.from_quat(s["quat"]).as_rotvec()[2])
        lines.append(f"{side}: shelf_in_base xyz {np.round(s['xyz'], 4).tolist()}, "
                     f"turned {ang:.0f} deg about z (not calibrated)")
    if route is not None:
        # where to BUILD the board, in the right base (x forward, y towards the left arm)
        s = cfg["right"]["shelf_in_base"]
        R, t = Rot.from_quat(s["quat"]), np.asarray(s["xyz"], float)
        o, slot = t, t + R.apply(route.target)
        lines.append(f"build the board here, RIGHT base frame: shelf origin (floor, left edge of "
                     f"column 0) at {np.round(1000 * o).tolist()} mm, its columns running along "
                     f"{np.round(R.apply([1, 0, 0]), 2).tolist()}; the piece at its slot at "
                     f"{np.round(1000 * slot).tolist()} mm")
    return lines


def add_args(ap):
    ap.add_argument("--sep", type=float, default=None,
                    help=f"[m] nominal pair, no calibration: left base at +y of the right one "
                         f"(the real cell: {SEP}); without it, the config's calibrated shelf_in_base")
    ap.add_argument("--forward", type=float, default=FORWARD,
                    help=f"[m] with --sep: base x of the piece path (its mean, default {FORWARD})")
    ap.add_argument("--lift", type=float, default=0.0,
                    help="[m] with --sep: the whole path this much HIGHER (tests in free air)")
    ap.add_argument("--shelf-x", type=float, default=None,
                    help="[m] with --sep: the shelf origin at this base x, the same for every world "
                         "(instead of centring each route on --forward)")
