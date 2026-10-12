"""
The convex program: how to split the load over the six pads.

18 unknowns (three force components per pad, six pads). Hard constraints: every
pad inside its friction cone, every pad under the actuator cap, and the six
forces together balancing the believed weight. Cost: effort, plus how far the
resulting pair of wrist wrenches is from what a human does at this attitude.

FOUR RULES THIS FILE EXISTS TO OBEY (CLAUDE.md 8, after the 54 kN blow-up):

  1. NO SDN CONE. Safety written as a SHRUNKEN cone lets the optimiser buy it
     with internal force: 2828 N against 55 N on identical arguments, `optimal`
     both times. Certificates come back only as a scalar floor `f_min`, which
     does not redraw the cone and so creates no incentive to squeeze.
  2. THE COST IS COERCIVE IN ALL 18 VARIABLES. A term that only looks at the
     resultant wrench is blind to internal forces - they cancel, so they live in
     its null space and are free. Hence `w_force` on the sum of squares of every
     pad force, at weight 1, not 1e-4.
  3. THE ACTUATOR CAP IS INSIDE THE PROGRAM (`||f_i|| <= f_pad_max`), not
     applied afterwards. Outside, an impossible request is discarded and the
     fallback decides the squeeze; inside, the solution is bounded by
     construction and kilonewtons are not representable.
  4. `optimal_inaccurate` IS NOT ENOUGH to accept a solution: the residuals must
     hold too.

IT NEVER RAISES ON BAD GEOMETRY. `ok=False` with large residuals means: this
grasp, with these contacts, under this rig's cap, cannot hold the object. That
is a diagnosis, and counting how often it happens is a measurement.

Vendored from `07_torsional_ucm/lib/socp.py` (verified to give identical
allocations), translated, with the SDN paths removed.
"""
import numpy as np
import cvxpy as cp


def _skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def cone(f, n, mu, f_min=0.0, r_s=0.0):
    """Coulomb cone for one pad, in this project's sign convention.

    `n` is the normal pointing OUT of the surface, so a compressive contact has
    `n . f <= 0`. On top of the cone: a minimum compression `f_min`, and a
    shrink factor `r_s` that keeps the allocation off the cone boundary.
    """
    fn = n @ f                       # negative in compression
    ft = f - fn * n
    return [fn <= -f_min,
            cp.norm(ft, 2) <= -(1.0 - r_s) * mu * fn]


class GraspSOCP:
    """Force allocation for the six pads, bounded by construction.

    `prior` is anything with `query(orientation) -> (mean, cov, Q)` in physical
    units, i.e. `prior.HumanPrior`. Passing `None` drops the prior term and
    leaves the pure effort cost - the ablation that says how much of the result
    the human model is responsible for.
    """

    def __init__(self, prior=None, prior_mode="tpgmm",
                 mu=0.49, f_pad_max=35.0, f_min=1.0, r_s=0.0,
                 w_prior=1.0, w_force=1.0, w_slack=1e4, r_char=0.05,
                 solver_order=("CLARABEL", "ECOS", "SCS")):
        self.prior = prior
        # WHAT WEIGHS THE WRENCH DIRECTIONS:
        #   "tpgmm"    the precision learnt from the human, Q(g_local) via GMR.
        #              Trained on exp13 with a standardised BOX: using it on a
        #              polytope assumes the strategy transfers to irregular
        #              shapes, and that assumption has never been checked.
        #   "identity" same target wrench, no learnt shape - a deviation costs
        #              the same in every direction. Scaled to EQUAL TRACE, so
        #              the only variable removed is the anisotropy: bare
        #              identity would also remove ~50x of weight against the
        #              force term, and the comparison would measure two things.
        #   "none"     no prior term at all: the pure effort program.
        self.prior_mode = str(prior_mode)
        self.mu = float(mu)
        # [N] cap per pad. On the exp13 rig this is F_HAND_MAX / 3 = 35 N.
        self.f_pad_max = float(f_pad_max)
        self.f_min = float(f_min)
        self.r_s = float(r_s)
        self.w_prior = float(w_prior)
        self.w_force = float(w_force)
        # High: equilibrium is violated only when the geometry leaves no
        # alternative under the cap.
        self.w_slack = float(w_slack)
        # [m] characteristic lever, to make the torque slack dimensionless.
        self.r_char = float(r_char)
        self.std_f = float(getattr(prior, "std_f", 12.0))
        self.std_tau = float(getattr(prior, "std_tau", 0.7))
        self.solver_order = tuple(solver_order)
        self.S = np.diag([self.std_f] * 3 + [self.std_tau] * 3 +
                         [self.std_f] * 3 + [self.std_tau] * 3)
        self._cache = {}                 # the GMR query is expensive

    def _prior_scaled(self, orientation):
        """(mean, Q) in the DIMENSIONLESS deviation space.

        `W_norm = S^-1 W`. The quadratic form is worth the same number
        (`d' Q d = d_n' Q_n d_n` with `Q_n = S Q S`), but `d_n` is O(1), so the
        solver works on numbers of one scale instead of newtons against
        newton-metres.
        """
        if self.prior is None or self.prior_mode == "none":
            return None, None
        key = np.round(np.asarray(orientation, float), 3).tobytes()
        if key not in self._cache:
            if len(self._cache) > 512:
                self._cache.clear()
            mean, _cov, Q = self.prior.query(orientation)
            Q_n = 0.5 * (self.S @ Q @ self.S + (self.S @ Q @ self.S).T)
            # project onto the PSD cone: the double transform can leave
            # minutely negative eigenvalues, and `quad_form` refuses them
            w, V = np.linalg.eigh(Q_n)
            Q_n = (V * np.clip(w, 1e-9, None)) @ V.T
            if self.prior_mode == "identity":
                Q_n = np.eye(12) * (float(np.trace(Q_n)) / 12.0)
            self._cache[key] = (np.linalg.solve(self.S, mean), Q_n)
        return self._cache[key]

    def solve(self, posL, posR, baseL, baseR, nrmL, nrmR,
              m_hat, p_com, orientation,
              active_L=None, active_R=None, f_min=None, w_des=None):
        """Allocate. Returns a dict, never raises on degenerate geometry.

        `p_com` is the BELIEVED centre of mass, and it is the pole the moment
        balance is written about with target zero - i.e. the program assumes
        gravity makes no moment about that point. With the pole at the grasp
        centre and the CoM three centimetres away it would be solving the
        equilibrium of an object that does not exist.
        """
        aL = [True] * 3 if active_L is None else [bool(x) for x in active_L]
        aR = [True] * 3 if active_R is None else [bool(x) for x in active_R]
        if not any(aL) or not any(aR):
            return _empty("not a bimanual grasp: one hand is not touching")

        # the compression floor cannot exceed the cap, or the cone is empty and
        # the problem is infeasible for an arithmetic rather than physical reason
        fmin = min(self.f_min if f_min is None else float(f_min),
                   0.9 * self.f_pad_max)

        f_L = [cp.Variable(3, name=f"fL{i}") for i in range(3)]
        f_R = [cp.Variable(3, name=f"fR{i}") for i in range(3)]
        con = []
        for hand, forces, normals, active in ((0, f_L, nrmL, aL),
                                              (1, f_R, nrmR, aR)):
            for i in range(3):
                if active[i]:
                    con += cone(forces[i], np.asarray(normals[i], float),
                                self.mu, f_min=fmin, r_s=self.r_s)
                    con += [cp.norm(forces[i], 2) <= self.f_pad_max]
                else:
                    con += [forces[i] == 0]      # the pad exists, carries nothing

        # --- equilibrium, with a measured slack ------------------------------
        g = np.array([0.0, 0.0, -9.81])
        if w_des is None:
            F_des, T_des = -float(m_hat) * g, np.zeros(3)
        else:
            w_des = np.asarray(w_des, float)
            F_des, T_des = w_des[:3], w_des[3:6]
        p_com = np.asarray(p_com, float)
        sum_f = sum(f_L) + sum(f_R)
        sum_t = (sum([_skew(np.asarray(posL[i], float) - p_com) @ f_L[i] for i in range(3)])
                 + sum([_skew(np.asarray(posR[i], float) - p_com) @ f_R[i] for i in range(3)]))
        sF, sT = cp.Variable(3, name="slack_F"), cp.Variable(3, name="slack_T")
        con += [sum_f == F_des + sF, sum_t == T_des + sT]

        # --- cost ------------------------------------------------------------
        f_ref = max(float(m_hat) * 9.81, 1e-6)
        t_ref = f_ref * self.r_char
        cost = self.w_force * sum([cp.sum_squares(f) for f in f_L + f_R]) / f_ref ** 2
        cost += self.w_slack * (cp.sum_squares(sF) / f_ref ** 2 +
                                cp.sum_squares(sT) / t_ref ** 2)

        mean_n, Q_n = self._prior_scaled(orientation)
        if Q_n is not None:
            Rb = np.asarray(orientation, float)
            W = []
            for pads, forces, base in ((posL, f_L, baseL), (posR, f_R, baseR)):
                Fw = sum(forces)
                Tw = sum([_skew(np.asarray(pads[i], float)
                                - np.asarray(base, float)) @ forces[i]
                          for i in range(3)])
                W += [Rb.T @ Fw / self.std_f, Rb.T @ Tw / self.std_tau]
            dW = cp.hstack(W) - mean_n
            cost += self.w_prior * 0.5 * cp.quad_form(dW, cp.psd_wrap(Q_n))

        prob = cp.Problem(cp.Minimize(cost), con)
        if not prob.is_dcp():
            return _empty("problem is not DCP")

        installed = set(cp.installed_solvers())
        errors = []
        for s in self.solver_order:
            if s not in installed:
                continue
            try:
                kw = {"solver": s}
                if s == "SCS":
                    kw.update({"max_iters": 20000, "eps": 1e-5})
                prob.solve(**kw)
            except cp.error.SolverError as e:
                errors.append(f"{s}: {e}")
                continue
            if prob.status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) and f_L[0].value is not None:
                r = self._pack(prob, s, f_L, f_R, nrmL, nrmR, sF, sT, aL, aR)
                # on a badly conditioned problem a solver can return
                # "inaccurate" with meaningless magnitudes: accept it only if
                # the residuals hold
                if prob.status == cp.OPTIMAL or r["ok"]:
                    return r
                errors.append(f"{s}: inaccurate, residuals "
                              f"{r['residual_force']:.1f} N / {r['residual_torque']:.2f} Nm")
        return _empty("no solver succeeded: " + "; ".join(errors))

    def _pack(self, prob, solver, f_L, f_R, nrmL, nrmR, sF, sT, aL, aR):
        FL = [np.asarray(f.value, float) for f in f_L]
        FR = [np.asarray(f.value, float) for f in f_R]
        fnL = [max(-float(np.asarray(nrmL[i], float) @ FL[i]), 0.0) for i in range(3)]
        fnR = [max(-float(np.asarray(nrmR[i], float) @ FR[i]), 0.0) for i in range(3)]
        eta = 0.0                      # largest fraction of a cone used
        for F, nrm, act in ((FL, nrmL, aL), (FR, nrmR, aR)):
            for i in range(3):
                if not act[i]:
                    continue
                n = np.asarray(nrm[i], float)
                fn = float(n @ F[i])
                if fn >= -1e-9:
                    continue
                ft = float(np.linalg.norm(F[i] - fn * n))
                eta = max(eta, ft / (self.mu * (-fn)))
        rF = float(np.linalg.norm(np.asarray(sF.value, float)))
        rT = float(np.linalg.norm(np.asarray(sT.value, float)))
        # 0.5 N and 0.05 Nm are below the noise of the contacts read from the
        # manifold: a smaller slack is not distinguishable from zero.
        ok = (rF < 0.5) and (rT < 0.05)
        return dict(status=prob.status, solver=solver, f_L=FL, f_R=FR,
                    fn_L=fnL, fn_R=fnR, peak=float(max(sum(fnL), sum(fnR))),
                    residual_force=rF, residual_torque=rT, eta_max=float(eta),
                    cost=float(prob.value), ok=bool(ok),
                    reason="" if ok else f"equilibrium not reachable under the "
                                         f"cap ({rF:.1f} N, {rT:.2f} Nm)")


def _empty(reason):
    z = [np.zeros(3)] * 3
    return dict(status="not_solved", solver=None, f_L=z, f_R=z,
                fn_L=[0.0] * 3, fn_R=[0.0] * 3, peak=0.0,
                residual_force=float("nan"), residual_torque=float("nan"),
                eta_max=float("nan"), cost=float("nan"), ok=False, reason=reason)
