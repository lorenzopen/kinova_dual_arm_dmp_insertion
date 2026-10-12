"""
The human coordination prior: what wrench a person applies at this attitude.

Answers ONE question, asked once per control tick:

    query(R_grasp)  ->  (mean, cov, Q)     12 numbers and a 12x12 precision

where the 12 are the two wrist wrenches (F_L, T_L, F_R, T_R) in the grasp frame.
`Q = cov^-1` is what the convex program uses to weigh a deviation: it is the
SHAPE of the human strategy, and it changes with the attitude - which is the
whole point of a task-parameterised model.

WHERE THE MODEL COMES FROM. `prior_torsional_C.npz`, i.e. exp13 session C at
Kp_rot 40, runs 024-036. Produced by `07_torsional_ucm/C_prior_socp/25_`, then
flattened to plain arrays. Do NOT substitute the exp11 model
(`human_gmm_wielding_model_updated_scale.pkl`): it was recorded at Kp_rot 0.5,
where the virtual coupling bled off 78% of the moment, so it describes a
different rig (CLAUDE.md 9). A missing file is an error, not a fallback.

WHY PLAIN ARRAYS. The original was a pickled TPGMM object, so loading it needed
the class `tpgmm_core` importable from another folder - a dependency nothing
declares and that fails halfway through a run with `ModuleNotFoundError`. Five
arrays carry the same information and need nothing but numpy.

THE MATH, all of it. The model is K Gaussians in 15 dimensions, stored in P = 2
FRAMES (world and box). Per tick:

  1. project every component into the common frame with that frame's rotation,
     and combine the two frames by PRODUCT OF GAUSSIANS - inverse covariances
     add up, so the frame that is confident about a direction wins it;
  2. condition on x = gravity in the box frame (dims 0-2), giving the mean and
     covariance of the remaining 12 (GMR);
  3. undo the scaling the model was fitted in, and un-mirror the right hand.
"""
import os

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL = os.path.join(HERE, "prior_torsional_C.npz")

# The right hand was fitted mirrored (the two hands face each other), so its
# force and torque come back with x and y flipped. CONSTRAINT 4.
MIRROR = np.diag([-1.0, -1.0, 1.0])
X_DIM = np.arange(3)         # input:  gravity in the box frame
Y_DIM = np.arange(3, 15)     # output: the two wrist wrenches


def _sym(M):
    return 0.5 * (M + M.T)


class HumanPrior:
    """The TP-GMM, queried by grasp attitude. Read-only, one method."""

    def __init__(self, path=None):
        path = path or MODEL
        if not os.path.exists(path):
            raise SystemExit(
                f"missing TP-GMM prior: {path}\n"
                "  Regenerate it with 07_torsional_ucm/C_prior_socp/25_ and "
                "flatten it with `python -m lib.prior --rebuild <pkl>`.\n"
                "  Do NOT fall back to the exp11 model: it comes from a rig at "
                "Kp_rot 0.5 (CLAUDE.md 9).")
        d = np.load(path)
        self.pi = d["pi"]                       # (K,)   component weights
        self.mu = d["mu"]                       # (P,K,D) means, per frame
        self.sigma = d["sigma"]                 # (P,K,D,D) covariances
        self.reg = float(d["reg"])              # ridge added before inverting
        self.std_f = float(d["std_f"])          # [N]  the fit was scaled by these
        self.std_tau = float(d["std_tau"])      # [Nm]
        self.K, self.D = self.mu.shape[1], self.mu.shape[2]
        self.S = np.diag([self.std_f] * 3 + [self.std_tau] * 3 +
                         [self.std_f] * 3 + [self.std_tau] * 3)
        print(f"  prior: {self.K} components, std_f {self.std_f:.2f} N, "
              f"std_tau {self.std_tau:.4f} Nm")

    def _combined(self, R_box):
        """The K Gaussians of the two frames, merged into one set by Product of
        Gaussians. `A1` rotates the box frame into the common one; the world
        frame is already there, so its `A2` is the identity."""
        A = [_block15(R_box.inv().as_matrix()), np.eye(self.D)]
        mu_c = np.zeros((self.K, self.D))
        sigma_c = np.zeros((self.K, self.D, self.D))
        for k in range(self.K):
            prec_sum = np.zeros((self.D, self.D))
            mu_sum = np.zeros(self.D)
            for j in range(len(A)):
                s = _sym(A[j] @ self.sigma[j, k] @ A[j].T
                         + self.reg * np.eye(self.D))
                prec = _sym(np.linalg.inv(s))
                prec_sum += prec                       # precisions add up
                mu_sum += prec @ (A[j] @ self.mu[j, k])
            cov = _sym(np.linalg.inv(_sym(prec_sum + self.reg * np.eye(self.D))))
            sigma_c[k] = cov
            mu_c[k] = cov @ mu_sum
        return mu_c, sigma_c

    def query(self, orientation):
        """(mean, cov, Q) in physical units, at this grasp attitude."""
        R_box = _rotation(orientation)
        g_local = R_box.inv().apply([0.0, 0.0, -1.0])     # gravity in the box
        mu_c, sigma_c = self._combined(R_box)

        # --- GMR: condition the 12 outputs on the 3 inputs -------------------
        mu_x, mu_y = mu_c[:, X_DIM], mu_c[:, Y_DIM]
        A, cond_cov, logw = [], [], np.zeros(self.K)
        for k in range(self.K):
            sxx = _sym(sigma_c[k][np.ix_(X_DIM, X_DIM)] + self.reg * np.eye(3))
            sxy = sigma_c[k][np.ix_(X_DIM, Y_DIM)]
            syy = sigma_c[k][np.ix_(Y_DIM, Y_DIM)]
            Ak = sigma_c[k][np.ix_(Y_DIM, X_DIM)] @ np.linalg.inv(sxx)
            A.append(Ak)
            cond_cov.append(_sym(syy - Ak @ sxy + self.reg * np.eye(12)))
            logw[k] = np.log(self.pi[k] + 1e-300) + _log_gauss(g_local, mu_x[k],
                                                               sigma_c[k][np.ix_(X_DIM, X_DIM)])
        w = np.exp(logw - logw.max())
        w = w / w.sum() if w.sum() > 1e-300 else np.ones(self.K) / self.K

        means = np.array([mu_y[k] + A[k] @ (g_local - mu_x[k])
                          for k in range(self.K)])
        mean = (w[:, None] * means).sum(0)
        cov = sum(w[k] * (cond_cov[k] + np.outer(means[k] - mean, means[k] - mean))
                  for k in range(self.K))
        cov = _sym(cov + self.reg * np.eye(12))

        # --- back to newtons, and un-mirror the right hand -------------------
        cov = self.S @ cov @ self.S + np.eye(12) * 1e-5
        mean = self.S @ mean
        U = np.eye(12)
        U[6:9, 6:9] = U[9:12, 9:12] = MIRROR
        cov = U @ cov @ U.T
        mean = U @ mean
        return mean, cov, _sym(np.linalg.inv(cov))


def _rotation(orientation):
    """The grasp attitude, however it was handed in: 3x3, quaternion, or a
    pitch in degrees (the oldest calling convention, kept because the model was
    fitted against it)."""
    if np.isscalar(orientation):
        return Rot.from_euler("y", float(orientation), degrees=True)
    a = np.asarray(orientation, float)
    if a.shape == (3, 3):
        return Rot.from_matrix(a)
    if a.shape == (4,):
        return Rot.from_quat(a)
    raise ValueError(f"orientation not handled: shape {a.shape}")


def _block15(R):
    """15x15 block diagonal of R: the 15 dimensions are five 3-vectors, and a
    change of frame rotates each of them."""
    M = np.zeros((15, 15))
    for i in range(5):
        M[3 * i:3 * i + 3, 3 * i:3 * i + 3] = R
    return M


def _log_gauss(x, mean, cov):
    """Log density, with the same ridge the model was fitted with."""
    d = np.asarray(x, float) - mean
    C = _sym(cov + 1e-9 * np.eye(len(d)))
    return float(-0.5 * (d @ np.linalg.solve(C, d)
                         + np.linalg.slogdet(C)[1] + len(d) * np.log(2 * np.pi)))


def rebuild(pkl_path, out=None):
    """Flatten a pickled TPGMM into the npz this module reads. Run it once,
    after `25_` produces a new model."""
    import pickle
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(HERE),
                                    "04_tp_gmm_wielding", "01_standardized_box"))
    with open(pkl_path, "rb") as f:
        d = pickle.load(f)
    t = d["tpgmm"]
    np.savez(out or MODEL, pi=np.asarray(t.pi, float),
             mu=np.asarray(t.mu, float), sigma=np.asarray(t.sigma, float),
             reg=float(t.reg_factor), std_f=float(d["std_f"]),
             std_tau=float(d["std_tau"]))
    return out or MODEL


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 2 and sys.argv[1] == "--rebuild":
        print("->", rebuild(sys.argv[2]))
    else:
        p = HumanPrior()
        m, c, Q = p.query(np.eye(3))
        print(f"  at level: |mean| {np.linalg.norm(m):.2f}, "
              f"trace(Q) {np.trace(Q):.1f}")
