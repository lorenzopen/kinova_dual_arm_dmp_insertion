"""What the SOCP asked against what the robot did, offline, from an `insert` log. No ROS.

    python -m kinova_dual_arm_dmp_insertion.analyze_run LOG.csv WORLD --arm ARM [--config cell.yaml] [--png out.png]

The log keeps only the SOCP's scalar `target_N`, so the program is solved again on
every logged tick of to_mouth / route / slow_seat, as `Insertion._allocate` did:
pads from the logged flange positions and the COMMANDED attitudes (the log has no
measured ones), pole = grasp centre - R_cmd r_plan, load = weight + the route's
w_ext at the nearest waypoint. The re-solved target against the logged one says
how faithful the reconstruction is. Then, per hand:

  * squeeze  - SOCP normal force vs measured (the loop closes only on the SUM);
  * lift     - SOCP vertical force vs measured (who carried the piece);
  * friction - the SOCP's cone use vs the friction the measured force needs
               (|F_tangential| / F_normal, against mu_eff of the route);
  * outside  - the external force the route predicted vs the measured one, with
               the jam limit.
"""
import argparse
import os

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

from . import geometry as GE
from .controller import PRIOR, G
from .main import load_config
from .prior import HumanPrior
from .route import Route
from .socp import GraspSOCP

PHASES = ("to_mouth", "route", "slow_seat", "seat_by_feel")


def resolve(d, r):
    """Re-solve the SOCP on every logged tick of the carrying phases."""
    c = r.c
    socp = GraspSOCP(prior=HumanPrior(PRIOR), prior_mode="tpgmm", mu=float(c["mu_eff"]),
                     f_pad_max=float(c["f_pad_max"]), f_min=1.0, r_s=0.10, r_char=0.05)
    a = r.axis / np.linalg.norm(r.axis)
    qL0 = GE.quat_from_x(a)
    R0 = r.rots[0]
    route_c = r.centres[:-1]
    out = []
    for _, row in d.iterrows():
        qL = Rot.from_quat(row[["qxL", "qyL", "qzL", "qwL"]].values.astype(float))
        qR = Rot.from_quat(row[["qxR", "qyR", "qzR", "qwR"]].values.astype(float))
        R_cmd = qL * qL0.inv() * R0                     # drive(): qL = dR * qL0, dR = R_cmd R0^-1
        pL = row[["mxL", "myL", "mzL"]].values.astype(float)
        pR = row[["mxR", "myR", "mzR"]].values.astype(float)
        tmid = 0.5 * (row[["xL", "yL", "zL"]].values + row[["xR", "yR", "zR"]].values).astype(float)
        c_cmd = tmid - (R_cmd * R0.inv()).apply(r.centre - r.grasp_c)   # the commanded object centre
        pole = 0.5 * (pL + pR) - R_cmd.apply(r.r_plan)
        hands = {"L": (pL, qL), "R": (pR, qR)}
        first, second = ("R", "L") if r.swapped else ("L", "R")
        (pA, nA), (pB, nB) = GE.pad_contacts(*hands[first]), GE.pad_contacts(*hands[second])
        w = np.zeros(6)
        w[:3] = [0.0, 0.0, r.mass * G]
        wext = np.zeros(3)
        if row.phase == "route":
            k = int(np.argmin(np.linalg.norm(route_c - c_cmd, axis=1)))
            wext = r.wext[k][0]
            w[:3] += r.wext[k][0]
            w[3:] = r.wext[k][1]
        q = hands[first][1]
        res = socp.solve(posL=pA, posR=pB, baseL=hands[first][0], baseR=hands[second][0],
                         nrmL=nA, nrmR=nB, m_hat=r.mass, p_com=pole, orientation=q.as_matrix(),
                         w_des=w, f_min=float(c["f_min"]))
        FA, FB = sum(res["f_L"]), sum(res["f_R"])
        FL_s, FR_s = (FB, FA) if r.swapped else (FA, FB)
        fnL_s, fnR_s = (sum(res["fn_R"]), sum(res["fn_L"])) if r.swapped else (sum(res["fn_L"]), sum(res["fn_R"]))
        out.append(dict(t=row.t, ok=res["ok"], target_re=max(fnL_s, fnR_s, 1.0) if res["ok"] else r.derived_N,
                        eta=res["eta_max"], fnL_s=fnL_s, fnR_s=fnR_s,
                        FzL_s=FL_s[2], FzR_s=FR_s[2],
                        ftL_s=np.linalg.norm(FL_s - (FL_s @ qL.apply([1, 0, 0])) * qL.apply([1, 0, 0])),
                        ftR_s=np.linalg.norm(FR_s - (FR_s @ qR.apply([1, 0, 0])) * qR.apply([1, 0, 0])),
                        pred_x=-wext[0], pred_y=-wext[1], pred_z=-wext[2]))
    return pd.DataFrame(out)


def rebase(d, cfg, path, r):
    """Correct the logged hand forces by an empty-handed `wrench_log` CSV, as
    ros_backend does online (nearest K samples within RADIUS, minus the baseline at
    the tare pose = the last open-pose tick). Rows left uncorrected keep their force.
    Use it on a log that ran WITHOUT the right baseline. Returns the corrected share."""
    from scipy.spatial import cKDTree
    from .geometry import Frame
    K, RADIUS = 25, 0.03
    b = pd.read_csv(path)
    tare = d[d.phase == "to_open_pose"].iloc[-1]
    hit = []
    for s, name in (("L", "left"), ("R", "right")):
        rows = b[(b.arm == s) & (b.mark != "start")] if (b[b.arm == s].mark != "start").any() else b[b.arm == s]
        tree, F = cKDTree(rows[["px", "py", "pz"]].values), rows[["Fbx", "Fby", "Fbz"]].values
        sb = Frame(cfg[name]["shelf_in_base"]["xyz"], cfg[name]["shelf_in_base"]["quat"])
        R_bs = sb.R.inv()                                  # base axes -> shelf axes

        def at(p_shelf):
            dist, i = tree.query(sb.t + sb.R.apply(p_shelf), k=K)
            return None if dist[0] > RADIUS else F[i].mean(0)
        b0 = at(tare[[f"mx{s}", f"my{s}", f"mz{s}"]].values.astype(float))
        b0 = np.zeros(3) if b0 is None else b0
        cols = [f"Fx{s}", f"Fy{s}", f"Fz{s}"]
        for k, row in d.iterrows():
            bb = at(row[[f"mx{s}", f"my{s}", f"mz{s}"]].values.astype(float))
            hit.append(bb is not None)
            if bb is not None:                             # F = -f_on_tool: the correction flips sign
                d.loc[k, cols] = row[cols].values.astype(float) + R_bs.apply(bb - b0)
    e = -(d[["FxL", "FyL", "FzL"]].values + d[["FxR", "FyR", "FzR"]].values) + [0.0, 0.0, r.mass * G]
    d["ext_x"], d["ext_y"], d["ext_z"] = e.T
    return float(np.mean(hit))


def measured(d):
    """Per hand: normal (along the commanded push axis), tangential, vertical."""
    for s in ("L", "R"):
        x = Rot.from_quat(d[[f"qx{s}", f"qy{s}", f"qz{s}", f"qw{s}"]].values).apply([1, 0, 0])
        F = d[[f"Fx{s}", f"Fy{s}", f"Fz{s}"]].values
        fn = np.einsum("ij,ij->i", F, x)
        d[f"fn{s}"] = fn
        d[f"ft{s}"] = np.linalg.norm(F - fn[:, None] * x, axis=1)
        d[f"mu{s}"] = d[f"ft{s}"] / np.clip(fn, 0.5, None)
    return d


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("world")
    ap.add_argument("--arm", default="c100_five_cut800")
    ap.add_argument("--config", default=None, help="the run's cell.yaml (default: config/cell.yaml)")
    ap.add_argument("--png", default=None, help="figure (default: next to the log)")
    ap.add_argument("--baseline", default=None,
                    help="re-correct the forces offline with this empty-handed wrench_log CSV")
    a = ap.parse_args(argv)
    cfg, _ = load_config(a.config)
    r = Route(a.world, a.arm, cfg.get("scale", 1.0), cfg.get("piece_mass"))
    jam_scale = float(cfg.get("jam_scale", 1.0))
    mu = float(r.c["mu_eff"])

    d = pd.read_csv(a.log)
    if a.baseline:
        print(f"  forces re-corrected offline by {a.baseline}: {100 * rebase(d, cfg, a.baseline, r):.0f}% of the readings")
    d = measured(d)
    c = d[d.phase.isin(PHASES)].reset_index(drop=True)
    s = resolve(c, r)
    c = pd.concat([c, s.drop(columns="t")], axis=1)
    c["dt_ms"] = 1000 * c.t.diff()
    # jam limit as watch_jam had it (route: share 3; to_mouth / seat: w_ext None -> share 1)
    share = np.where(c.phase == "route", float(r.c["jam_share"]), 1.0)
    spare = 2.0 * mu * np.maximum(c.target_N.clip(lower=1.0), 1.0) - r.mass * G
    c["jam_lim"] = np.maximum(jam_scale * share * spare, 2.0)
    c["jam_res"] = np.hypot(c.ext_x - c.pred_x, c.ext_y - c.pred_y)

    print(f"  {r.world}: mass {r.mass:.3f} kg ({r.mass * G:.2f} N), mu_eff {mu}, {len(c)} carrying ticks "
          f"(median {c.dt_ms.median():.0f} ms apart)")
    print(f"  SOCP re-solved: ok {int(c.ok.sum())}/{len(c)}; target logged vs re-solved: "
          f"median |diff| {np.median(np.abs(c.target_N - c.target_re)):.2f} N")
    g = c.groupby("phase", sort=False)
    tab = pd.DataFrame({
        "SOCP fn L": g.fnL_s.median(), "mis fn L": g.fnL.median(),
        "SOCP fn R": g.fnR_s.median(), "mis fn R": g.fnR.median(),
        "SOCP Fz L": g.FzL_s.median(), "mis Fz L": g.FzL.median(),
        "SOCP Fz R": g.FzR_s.median(), "mis Fz R": g.FzR.median(),
        "SOCP cono": g.eta.max(), "mis mu L p95": g.muL.quantile(0.95), "mis mu R p95": g.muR.quantile(0.95),
        "ext pred z": g.pred_z.median(), "ext mis z": g.ext_z.median(),
        "jam res max": g.jam_res.max(), "jam lim min": g.jam_lim.min()})
    with pd.option_context("display.width", 250):
        print("  medians per phase [N] (cone / mu: fraction, max / p95):")
        print(tab.round(2).T.to_string())
    over = (c[["muL", "muR"]].max(axis=1) > mu).mean()
    print(f"  ticks where the measured force needs more friction than mu_eff {mu}: {100 * over:.0f}%")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = c.t - c.t.iloc[0]
    fig, ax = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    for side, col in (("L", "C0"), ("R", "C3")):
        ax[0].plot(t, c[f"fn{side}_s"], "--", color=col, label=f"SOCP {side}")
        ax[0].plot(t, c[f"fn{side}"], color=col, label=f"misurata {side}")
        ax[1].plot(t, c[f"Fz{side}_s"], "--", color=col, label=f"SOCP {side}")
        ax[1].plot(t, c[f"Fz{side}"], color=col, label=f"misurata {side}")
        ax[2].plot(t, c[f"mu{side}"], color=col, label=f"|Ft|/Fn misurata {side}")
    ax[0].plot(t, c.target_N, ":", color="k", label="target loggato")
    ax[0].set_ylabel("stretta normale [N]")
    ax[1].axhline(r.mass * G, color="gray", lw=0.8)
    ax[1].plot(t, c.FzL + c.FzR, color="k", lw=0.8, label="somma misurata")
    ax[1].set_ylabel("forza verticale mano->pezzo [N]")
    ax[2].plot(t, c.eta, "--", color="k", label="uso del cono SOCP (eta)")
    ax[2].axhline(mu, color="gray", lw=0.8)
    ax[2].set_ylim(0, 3)
    ax[2].set_ylabel("attrito richiesto")
    ax[3].plot(t, c.jam_res, color="C2", label="forza laterale inattesa")
    ax[3].plot(t, c.jam_lim, "--", color="C2", label="soglia JAM")
    ax[3].plot(t, c.ext_z, color="C1", label="ext z misurata")
    ax[3].plot(t, c.pred_z, "--", color="C1", label="ext z prevista (route)")
    ax[3].axhline(float(r.c["land_share"]) * r.mass * G, color="gray", lw=0.8)
    ax[3].set_ylabel("ambiente sul pezzo [N]")
    ax[3].set_xlabel("t dall'inizio del trasporto [s]")
    for x in ax:
        x.legend(loc="upper right", fontsize=7, ncol=3)
        x.grid(alpha=0.3)
        for ph in ("route", "slow_seat"):
            if (c.phase == ph).any():
                x.axvline(t[c.phase == ph].iloc[0], color="gray", ls=":", lw=0.8)
    fig.suptitle(f"{r.world}: SOCP vs robot ({os.path.basename(a.log)})")
    fig.tight_layout()
    png = a.png or os.path.splitext(a.log)[0] + ("_socp_rebased.png" if a.baseline else "_socp.png")
    fig.savefig(png, dpi=110)
    print(f"  figure: {png}")


if __name__ == "__main__":
    main()
