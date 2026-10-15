"""Draw the board of a route: what to BUILD on the real shelf before `insert`.

The shelf seen from the LEFT arm's side (shelf x to the right, z up, the shelf
frame of cell.yaml: origin on the floor at the left edge of column 0). The
four neighbours where the route file puts them (the narrowed slot: they lean
into it by 12.5% of a cell), the slot of the inserted piece dashed, the path
of the piece's centre from the grasp pose down to the slot. Under it, where the
shelf and the slot are in the RIGHT base frame: the measured `shelf_in_base` of
cell.yaml, or with `--sep` the nominal pair placement (pair.py).

    python -m kinova_dual_arm_dmp_insertion.board F75_Tt1_s9007 --routes c100_five_cut800
    -> docs/boards/F75_Tt1_s9007.png, and the piece coordinates printed in mm

Needs matplotlib only (no ROS).
"""
import argparse
import glob
import json
import os

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from . import pair
from .main import load_config
from .route import ROUTES, Route

HERE = os.path.dirname(os.path.abspath(__file__))
COLOURS = {"I": "#5bc0de", "O": "#f0ad4e", "T": "#a77dc2", "L": "#e8853d", "J": "#4a78c2",
           "S": "#5cb85c", "Z": "#d9534f"}
# a flat piece turned over is its mirror: the physical kit I O T L Z also gives J and S
MIRROR = {"J": "L (girata)", "S": "Z (girata)"}
R_TRI = 0.0404       # [m] the tri-pad's contact circle (cell.yaml tool: r_tri), drawn round the grasp point


def shapes():
    """Cells of each piece in its own frame, from the routes' piece models (one per letter)."""
    out = {}
    for f in sorted(glob.glob(os.path.join(ROUTES, "*", "*.json"))):
        with open(f, encoding="utf-8") as fh:
            d = json.load(fh)
        if d.get("scale_applied") and d["piece"] not in out:
            out[d["piece"]] = np.asarray(d["piece_model"]["cells"], float)
    return out


def pieces(route):
    """[(letter, cells [4, 3] in the shelf frame, is_the_inserted_one)]."""
    sh = shapes()
    out = [(n["piece"], np.asarray(n["position"]) + Rot.from_quat(n["quat"]).apply(sh[n["piece"]]), False)
           for n in route.neighbours]
    out.append((route.piece, route.target + route.R_target.apply(route.cells), True))
    return out


def start_cells(route):
    """The corners [4, 3] of each cell of the piece held at the grasp pose, shelf frame."""
    h = 0.5 * float(route.raw["piece_model"]["cell"])
    sq = np.array([[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]])
    return [route.grasp_c + route.grasp_R.apply(c + sq) for c in route.cells]


def tilt_deg(route):
    """The start pose against the slot: degrees about the shelf y (the view axis),
    > 0 = clockwise in the figure (seen from the left arm)."""
    return float(np.degrees((route.grasp_R * route.R_target.inv()).as_rotvec()[1]))


def grasp_cell(route):
    """Which cell the tri-pads press on: its index and its centre in mm (piece frame)."""
    k = int(np.argmin(np.linalg.norm(route.cells - route.r_plan, axis=1)))
    return k, 1000 * (route.r_plan - route.cells[k])


def draw(route, out_png, placement=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle
    s = route.shelf
    cell = float(route.raw["piece_model"]["cell"])
    x0, x1 = s["inner_faces"]
    fig, ax = plt.subplots(figsize=(7.5, 8.5))
    ax.add_patch(Rectangle((x0 - s["wall"], 0), s["wall"], s["z_top"], color="0.55"))
    ax.add_patch(Rectangle((x1, 0), s["wall"], s["z_top"], color="0.55"))
    ax.plot([x0 - s["wall"], x1 + s["wall"]], [0, 0], color="0.3", lw=3)
    for c in range(s["cols"]):
        xc = 0.5 * cell + c * s["pitch"]
        ax.text(xc, -0.03, f"col {c}", ha="center", va="top", fontsize=8, color="0.4")
    for letter, cells, ins in pieces(route):
        for x, _, z in cells:
            r = Rectangle((x - cell / 2, z - cell / 2), cell, cell, lw=1.5,
                          edgecolor="k" if not ins else COLOURS[letter],
                          facecolor=COLOURS[letter] if not ins else "none",
                          ls="-" if not ins else "--", alpha=0.85 if not ins else 1.0)
            ax.add_patch(r)
        cx, cz = cells[:, 0].mean(), cells[:, 2].mean()
        if ins:   # the name on the cell farthest from the grasp point (the X)
            g = route.target + route.R_target.apply(route.r_plan)
            cx, _, cz = cells[np.argmax(np.linalg.norm(cells - g, axis=1))]
        name = letter + ("\n(SLOT)" if ins else "") + (f"\n= {MIRROR[letter]}" if letter in MIRROR else "")
        ax.text(cx, cz, name, ha="center", va="center", fontsize=10, weight="bold")
    path = np.vstack([route.grasp_c, route.centres, route.target])
    ax.plot(path[:, 0], path[:, 2], color=COLOURS[route.piece], lw=2)
    # the piece where it is HELD at the start (put it there by hand): its cells turned
    # as the grasp pose turns them, and the grasp point (the two tri-pads, one per face)
    start = start_cells(route)
    for corners in start:
        ax.add_patch(Polygon(corners[:, [0, 2]], closed=True, lw=1.5, edgecolor="k",
                             facecolor=COLOURS[route.piece], alpha=0.5))
    pts = np.vstack(start)
    ax.text(pts[:, 0].max() + 0.015, pts[:, 2].min(), route.piece + " INIZIO", ha="left",
            va="bottom", fontsize=9, weight="bold")
    for p, R in ((route.grasp_c, route.grasp_R), (route.target, route.R_target)):
        g = p + R.apply(route.r_plan)
        ax.plot(*g[[0, 2]], "x", color="k", ms=12, mew=2.5)
        ax.add_patch(Circle(g[[0, 2]], R_TRI, fill=False, lw=1, ls=":", color="k"))
    tilt = tilt_deg(route)
    g0 = route.grasp_c + route.grasp_R.apply(route.r_plan)
    ax.annotate(f"punte qui, su tutte e due le facce\ninclinato {abs(tilt):.0f} gradi "
                f"{'orario' if tilt > 0 else 'antiorario'} rispetto allo slot" if abs(tilt) >= 0.5
                else "punte qui, su tutte e due le facce\ndritto come nello slot",
                (route.grasp_c + route.grasp_R.apply(route.r_plan))[[0, 2]],
                xytext=(40 if g0[0] < 0.5 * (x0 + x1) else -40, 40), textcoords="offset points", fontsize=8,
                ha="left" if g0[0] < 0.5 * (x0 + x1) else "right",
                arrowprops=dict(arrowstyle="-", lw=0.6))
    top = max(s["z_top"], path[:, 2].max(), np.vstack(start)[:, 2].max())
    ax.set_aspect("equal")
    ax.set_xlim(x0 - 0.08, x1 + 0.08)
    ax.set_ylim(-0.08, top + 0.14)          # room for the note above the start pose
    ax.set_xticks(np.arange(0.0, x1 + 0.01, 0.1))
    ax.set_yticks(np.arange(0.0, s["z_top"] + 0.01, 0.1))
    ax.xaxis.set_major_formatter(lambda v, _: f"{1000 * v:.0f}")
    ax.yaxis.set_major_formatter(lambda v, _: f"{1000 * v:.0f}")
    ax.grid(alpha=0.25)
    ax.set_xlabel("x scaffale [mm] (dal bordo sinistro della colonna 0)   -   vista dal braccio SINISTRO, le basi a destra")
    ax.set_ylabel("z [mm] (dal pavimento dello scaffale)")
    sim = route.simulated
    title = (f"{route.world}: inserire {route.piece}   (PyBullet: placed {sim.get('placed_mm', np.nan):.1f} mm, "
             f"slip {sim.get('slip_mm', np.nan):.1f} mm)")
    ax.set_title(title, fontsize=10)
    if placement:
        fig.text(0.02, 0.01, placement, fontsize=7.5, family="monospace", va="bottom")
        fig.subplots_adjust(bottom=0.17)
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def table(route):
    """Where each piece goes, mm in the shelf frame: its cells' x and z ranges."""
    cell = float(route.raw["piece_model"]["cell"])
    lines = []
    for letter, cells, ins in pieces(route):
        lo, hi = cells.min(axis=0) - cell / 2, cells.max(axis=0) + cell / 2
        what = "SLOT (va inserito)" if ins else MIRROR.get(letter, letter)
        lines.append(f"{letter:2s} {what:20s} x {1000 * lo[0]:4.0f}..{1000 * hi[0]:4.0f} mm, "
                     f"z {1000 * lo[2]:4.0f}..{1000 * hi[2]:4.0f} mm")
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("worlds", nargs="+")
    ap.add_argument("--routes", default="c100_five_cut800")
    ap.add_argument("--out", default=os.path.join(HERE, "..", "docs", "boards"))
    ap.add_argument("--config", default=None)
    pair.add_args(ap)
    a = ap.parse_args()
    cfg, _ = load_config(a.config)
    for w in a.worlds:
        r = Route(w, a.routes, cfg.get("scale", 1.0), cfg.get("piece_mass"))
        # --sep: the nominal pair (pair.py); without it the measured shelf_in_base of cell.yaml
        pl = pair.place(cfg, r, a.sep, a.forward, a.lift, a.shelf_x) if a.sep is not None else cfg
        s = pl["right"]["shelf_in_base"]
        R, t = Rot.from_quat(s["quat"]), np.asarray(s["xyz"], float)
        far = t + R.apply([r.shelf["inner_faces"][1], 0.0, 0.0])
        slot = t + R.apply(r.target)
        where = f"a {1000 * a.sep:.0f} mm" if a.sep is not None else "cell.yaml"
        place = (f"Frame della base DESTRA (x avanti, y verso il sinistro, {where}):\n"
                 f"  origine scaffale (pavimento, bordo sx col 0): x {1000 * t[0]:.0f}, y {1000 * t[1]:.0f}, "
                 f"z {1000 * t[2]:.0f} mm\n"
                 f"  colonne verso le basi: col {r.shelf['cols'] - 1} finisce a x {1000 * far[0]:.0f} mm\n"
                 f"  il pezzo nello slot: x {1000 * slot[0]:.0f}, y {1000 * slot[1]:.0f}, z {1000 * slot[2]:.0f} mm")
        out = os.path.join(a.out, f"{w}.png")
        draw(r, out, place)
        print(f"  {w}: {os.path.normpath(out)}")
        for line in table(r):
            print("    " + line)
        g = r.grasp_c + r.grasp_R.apply(r.r_plan)
        k, off = grasp_cell(r)
        print(f"    INIZIO (lo metti tu a mano): baricentro x {1000 * r.grasp_c[0]:.0f}, "
              f"z {1000 * r.grasp_c[2]:.0f} mm, inclinato {tilt_deg(r):+.1f} gradi rispetto allo "
              f"slot (+ = orario visto dal sinistro)")
        print(f"    punte (le due facce): x {1000 * g[0]:.0f}, z {1000 * g[2]:.0f} mm, al centro del "
              f"cubo {k} del pezzo (scarto {np.linalg.norm(off):.0f} mm)")
        if place:
            print("    " + place.replace("\n", "\n    "))


if __name__ == "__main__":
    main()
