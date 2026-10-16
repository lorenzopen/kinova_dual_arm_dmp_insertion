"""THE INSERTION, step for step as the trainer drives it (38_learn_pybullet.py
`drive_hands`, tetris/handling.py `Hands`), on any backend: the real Kinova
pair through ROS 2, or the dry run.

What the robot is sent, and nothing else:
  * two flange poses, streamed every control tick - the OBJECT pose the route
    asks for, carried rigidly to the two hands (one object frame, two hands);
  * on top, ONE scalar `ds`: the extra squeeze along the grasp axis, closed on
    the MEASURED normal force, at the target the SOCP (with the TP-GMM prior)
    re-solves every 100 ms;
  * the wrist impedance is the arm's controller (1500 N/m, 40 Nm/rad about the
    push axis, 8 Nm/rad tilt): the simulated wrist was that spring, so the
    trained routes assume it.
What is read: each wrist's F/T sensor (gravity-compensated by the backend) and
each flange pose. The jam sentry, the seat by feel and the stop on a high
landing run on those two readings only - no contact sensor, no vision.

Order of a run: open pose -> (operator presents the piece) -> approach ->
squeeze -> (operator lets go) -> to the route's mouth -> the learnt route ->
slow straight seat -> seat by feel -> release -> retreat.
"""
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from . import geometry as GE
from .prior import HumanPrior
from .socp import GraspSOCP

import os
PRIOR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "prior_torsional_C.npz")

# ---- the simulated apparatus's numbers (tetris/handling.py, lib/wrist.py) ----
KP = 1500.0                     # [N/m] wrist translational stiffness
K_TILT_SEAT, K_TILT_HOLD = 3.0, 8.0   # [Nm/rad] wrist tilt stiffness: approaching / holding
K_TORSION = 40.0                # [Nm/rad] about the push axis
CONTROL_PERIOD = 0.020          # [s] squeeze update (EVERY = 20 ticks of 1 ms)
SOCP_PERIOD = 0.100             # [s] the convex program (EVERY_SOCP)
GAIN, DS_MAX = 1.0 / (4.0 * KP), 2e-4   # squeeze integrator [m/N] and slew per update [m]
S_MAX, RELEASE = 0.060, 0.012   # [m] how far the hands may close past contact / back off
F_TOUCH = 2.0                   # [N] "in contact" (both hands summed)
V_APP = 0.040                   # [m/s] approach
TURN_FLOOR = 2.0                # [s] a turn never takes less
SEAT_DROP, SEAT_SPEED = 0.060, 0.020   # seat by feel: at most 60 mm down at 20 mm/s
PICKUP_N = 6.0                 # [N] per hand: the pickup squeeze on the REAL pieces (polystyrene),
                                # capped below the route's derived_N (25 N); the route then asks 3-6 N
# THE HAND WRENCH (hwd routes only, `internal-descent`): Wong et al. eq. 49-50 as an integrator
GAIN_F, STEP_F, MAX_F = 1.0 / (4.0 * KP), 2e-4, 0.020
GAIN_T, STEP_T, MAX_T = 1.0 / (4.0 * 40.0), 2e-3, 0.15
G = 9.81


class Abort(RuntimeError):
    pass


class Insertion:
    def __init__(self, route, backend, log=None, f_abort=60.0, track_abort=0.040,
                 confirm=None, jam_scale=1.0):
        self.r, self.b = route, backend
        c = route.c
        self.mu = float(c["mu_eff"])                  # 1.0 for narrow75 (pad-piece rubber)
        self.jam_share = float(c["jam_share"])        # 3 for narrow75
        self.motion_speed = float(c["motion_speed"])  # 0.085
        self.turn_rate = float(c["turn_rate"])        # 0.25
        self.seat_speed = float(c["seat_speed"])      # 0.030
        self.seat_band = float(c["seat_band"])        # 0.040
        self.land_share = float(c["land_share"])      # 0.6
        self.seat_stops = bool(c["seat_stops"])
        self.hand_wrench = route.hand_wrench != "off"
        self.f_min = float(c["f_min"])
        self.socp = GraspSOCP(prior=HumanPrior(PRIOR), prior_mode="tpgmm", mu=self.mu,
                              f_pad_max=float(c["f_pad_max"]), f_min=1.0, r_s=0.10, r_char=0.05)
        self.log = log
        self.f_abort, self.track_abort = float(f_abort), float(track_abort)   # robot-only safety
        self.jam_scale = float(jam_scale)                                     # robot: looser sentry
        self.confirm = confirm or (lambda msg: input(f"\n  >>> {msg} [Enter] "))
        # the open pose (Hands.__init__): LEFT pushes along +axis
        a = route.axis / np.linalg.norm(route.axis)
        self.axis = a
        self.qL, self.qR = GE.quat_from_x(a), GE.quat_from_x(-a)
        # the reach with the REAL tool's face (geometry.set_tool), the sim's air gap
        reach = 0.5 * route.opening + GE.PAD_X + GE.PAD_R + float(c["open_clear"])
        self.cL = route.centre - reach * a
        self.cR = route.centre + reach * a
        self.frame = None
        self.ds, self.target, self.w_ext = 0.0, None, None
        self.off = {"L": np.zeros(6), "R": np.zeros(6)}
        self.w_des = {}
        self.c_cmd = self.R_cmd = None
        self.t_ctrl = self.t_socp = -1e9
        self.jammed, self.jam_info, self.landed_high = False, None, False
        self.seat_goal = None
        self.phase = "start"
        self.k_tilt = K_TILT_SEAT
        self.stats = dict(socp_ok=0, socp_ticks=0, f_ext_max=0.0, asked=[])

    # ------------------------------------------------------------ one tick
    def tick(self, tL, qL, tR, qR, k_tilt):
        """Send both flange targets, wait one period, read, update the loops."""
        self._q_cmd, self._q_cmd_R = qL, qR
        self.k_tilt = k_tilt
        self.b.command(tL, qL, tR, qR, k_tilt)
        self.b.wait()
        self.meas = self.b.read()          # dict: pL qL pR qR FL TL FR TR (hand ON piece, shelf frame)
        self._safety(tL, tR)
        now = self.b.now()
        if self.frame is not None and now - self.t_ctrl >= CONTROL_PERIOD - 1e-9:
            self.t_ctrl = now
            self._control(now)
        if self.log is not None:
            self.log.row(self, tL, qL, tR, qR)

    def _safety(self, tL, tR):
        m = self.meas
        for side, F, p_meas, p_cmd in (("left", m["FL"], m["pL"], tL), ("right", m["FR"], m["pR"], tR)):
            if np.linalg.norm(F) > self.f_abort:
                raise Abort(f"{side} hand force {np.linalg.norm(F):.1f} N over {self.f_abort:.0f} N")
            # the impedance lags by F / KP; far more than that is not a lag
            if np.linalg.norm(np.asarray(p_meas) - np.asarray(p_cmd)) > self.track_abort + np.linalg.norm(F) / KP:
                raise Abort(f"{side} flange {1000 * np.linalg.norm(np.asarray(p_meas) - p_cmd):.0f} mm "
                            f"from its target")

    # ------------------------------------------------------------ readings
    def normal_force(self):
        """[N] both hands, the component along each hand's push axis."""
        m = self.meas
        xL = Rot.from_quat(m["qL"]).apply([1.0, 0.0, 0.0])
        xR = Rot.from_quat(m["qR"]).apply([1.0, 0.0, 0.0])
        return max(0.0, float(m["FL"] @ xL)) + max(0.0, float(m["FR"] @ xR))

    def external_force(self):
        """[N] what the environment pushes on the piece: weight minus what the hands carry."""
        return -(self.meas["FL"] + self.meas["FR"]) + np.array([0.0, 0.0, self.r.mass * G])

    def grasp_centre(self):
        """Midpoint of the two plates = midpoint of the two flanges (opposite push axes)."""
        return 0.5 * (np.asarray(self.meas["pL"]) + np.asarray(self.meas["pR"]))

    def piece_attitude(self):
        return Rot.from_quat(self.meas["qL"]) * self.R_rel.inv()

    def believed_height(self, goal):
        origin = self.grasp_centre() - self.piece_attitude().apply(self.r.r_plan)
        return float(origin[2] - goal[2])

    # ------------------------------------------------------------ the loops
    def _control(self, now):
        if self.target is None or now - self.t_socp >= SOCP_PERIOD - 1e-9:
            self.t_socp = now
            self._allocate()
        f_now = self.normal_force()
        step = float(np.clip(GAIN * (2.0 * self.target - f_now), -DS_MAX, DS_MAX))
        self.ds = float(np.clip(self.ds + step, -RELEASE, S_MAX))
        self.stats["asked"].append(self.target)
        if self.hand_wrench and self.w_des and self.w_ext is not None:
            self._chase_wrench()

    def _allocate(self):
        """The SOCP on the pads as the geometry puts them (no contact sensor)."""
        m = self.meas
        pole = self.grasp_centre() - self.R_cmd.apply(self.r.r_plan)
        hands = {"L": (m["pL"], Rot.from_quat(m["qL"])), "R": (m["pR"], Rot.from_quat(m["qR"]))}
        first, second = ("R", "L") if self.r.swapped else ("L", "R")
        q = self._q_cmd_R if self.r.swapped else self._q_cmd
        (pA, nA), (pB, nB) = GE.pad_contacts(*hands[first]), GE.pad_contacts(*hands[second])
        w = np.zeros(6)
        w[:3] = [0.0, 0.0, self.r.mass * G]
        if self.w_ext is not None:
            w[:3] += self.w_ext[0]
            w[3:] = self.w_ext[1]
        self.stats["socp_ticks"] += 1
        res = self.socp.solve(posL=pA, posR=pB, baseL=np.asarray(hands[first][0]),
                              baseR=np.asarray(hands[second][0]), nrmL=nA, nrmR=nB,
                              m_hat=self.r.mass, p_com=pole, orientation=q.as_matrix(),
                              w_des=w, f_min=self.f_min)
        if res["ok"]:
            self.stats["socp_ok"] += 1
            self.target = max(float(max(sum(res["fn_L"]), sum(res["fn_R"]))), 1.0)
            if self.hand_wrench:
                c = self.grasp_centre()
                self.w_des = {side: (sum(np.asarray(f) for f in forces),
                                     sum(np.cross(pp - c, np.asarray(f)) for pp, f in zip(pads, forces)))
                              for side, pads, forces in ((first, pA, res["f_L"]), (second, pB, res["f_R"]))}
        else:
            self.target = self.r.derived_N
            self.w_des = {}

    def _chase_wrench(self):
        """hwd routes: only how the two hands SHARE the load (the null space of G),
        equal and opposite flange offsets, the squeeze axis left to `ds`."""
        m, c = self.meas, self.grasp_centre()
        a = self.R_cmd.apply(self.frame["ax"])
        e = {}
        for side in ("L", "R"):
            F, T, p = m["F" + side], m["T" + side], np.asarray(m["p" + side])
            T_c = T + np.cross(p - c, F)                  # measured, about the pole
            F_des, T_des = self.w_des[side]
            e[side] = (F - F_des, T_c - T_des)
        eF, eT = 0.5 * (e["L"][0] - e["R"][0]), 0.5 * (e["L"][1] - e["R"][1])
        for side, sF, sT in (("L", eF, eT), ("R", -eF, -eT)):
            sF = sF - (sF @ a) * a
            o = self.off[side]
            o[:3] += np.clip(-GAIN_F * sF, -STEP_F, STEP_F)
            o[3:] += np.clip(-GAIN_T * sT, -STEP_T, STEP_T)
            for sl, cap in ((slice(0, 3), MAX_F), (slice(3, 6), MAX_T)):
                n = np.linalg.norm(o[sl])
                if n > cap:
                    o[sl] *= cap / n

    def jam_limit(self):
        f = self.target if self.target else self.r.derived_N
        spare = 2.0 * self.mu * max(float(f), 1.0) - self.r.mass * G
        share = self.jam_share if (self.w_ext is not None or not self.seat_stops) else 1.0
        return max(self.jam_scale * share * spare, 2.0)

    def watch_jam(self):
        meas = self.external_force()
        pred = -self.w_ext[0] if self.w_ext is not None else np.zeros(3)
        self.stats["f_ext_max"] = max(self.stats["f_ext_max"], float(np.linalg.norm(meas[:2])))
        e = (meas - pred)[:2]
        if float(np.linalg.norm(e)) > self.jam_limit() and not self.jammed:
            self.jammed = True
            self.jam_info = dict(res=float(np.linalg.norm(e)), limit=self.jam_limit())
            print(f"      JAM: unexpected side force {self.jam_info['res']:.1f} N "
                  f"> {self.jam_info['limit']:.1f} N")
        return self.jammed

    def lands_high(self):
        if not self.seat_stops or self.external_force()[2] < self.land_share * self.r.mass * G:
            return False
        if not self.seat_band < self.believed_height(self.seat_goal) < 0.200:
            return False
        self.landed_high = True
        print("      LANDED HIGH: the environment carries the piece above its seat")
        return True

    # ------------------------------------------------------------ the object frame
    def freeze(self, origin, R_obj):
        c = np.asarray(origin, float)
        self.frame = dict(R0=R_obj, c0=c, oL=R_obj.inv().apply(self.cL - c),
                          oR=R_obj.inv().apply(self.cR - c), ax=R_obj.inv().apply(self.axis))

    def drive(self, centre, R_obj, k_tilt=K_TILT_HOLD):
        f = self.frame
        dR = R_obj * f["R0"].inv()
        c = np.asarray(centre, float)
        d = self.ds * f["ax"]
        self.R_cmd, self.c_cmd = R_obj, c
        tL, tR = c + R_obj.apply(f["oL"] + d), c + R_obj.apply(f["oR"] - d)
        qL, qR = dR * self.qL, dR * self.qR
        oL, oR = self.off["L"], self.off["R"]
        self.tick(tL + oL[:3], Rot.from_rotvec(oL[3:]) * qL, tR + oR[:3], Rot.from_rotvec(oR[3:]) * qR,
                  k_tilt)

    def move(self, c_from, R_from, c_to, R_to, seconds, watch=False):
        c_from, c_to = np.asarray(c_from, float), np.asarray(c_to, float)
        turn = float(np.linalg.norm((R_from.inv() * R_to).as_rotvec()))
        seconds = max(float(seconds), float(np.linalg.norm(c_to - c_from)) / self.motion_speed,
                      max(TURN_FLOOR, turn / self.turn_rate))
        t0 = self.b.now()
        while True:
            u = min(1.0, (self.b.now() - t0) / seconds)
            R = R_from * Rot.from_rotvec(u * (R_from.inv() * R_to).as_rotvec())
            self.drive(c_from + u * (c_to - c_from), R)
            if watch and self.watch_jam():
                return False
            if watch and self.seat_goal is not None and self.lands_high():
                return False
            if u >= 1.0:
                return True

    def follow(self, centres, rots, wext, seconds):
        """The learnt route, paced by distance and angle (Hands.follow, paced=True)."""
        times = GE.segment_times(centres, rots, self.motion_speed, self.turn_rate)
        if times[-1] > 0:
            times = times * max(1.0, float(seconds) / times[-1])
        t0 = self.b.now()
        while True:
            t = self.b.now() - t0
            c, R, k, j, mix = GE.interpolate(times, centres, rots, t)
            self.w_ext = tuple((1 - mix) * wext[k][i] + mix * wext[j][i] for i in (0, 1))
            self.drive(c, R)
            if self.watch_jam():
                return False
            if t >= times[-1]:
                return True

    # ------------------------------------------------------------ the run
    def open_pose(self, seconds=None):
        """Bring both tools to the open pose around the presented piece, in a
        straight line from wherever they are (start the arms close to it)."""
        self.phase = "to_open_pose"
        m = self.b.read()
        pL, pR = np.asarray(m["pL"]), np.asarray(m["pR"])
        dist = max(np.linalg.norm(pL - self.cL), np.linalg.norm(pR - self.cR))
        turns = max(float(np.linalg.norm((Rot.from_quat(m["qL"]).inv() * self.qL).as_rotvec())),
                    float(np.linalg.norm((Rot.from_quat(m["qR"]).inv() * self.qR).as_rotvec())))
        seconds = max(seconds or 0.0, dist / self.motion_speed, turns / self.turn_rate, TURN_FLOOR)
        qL0, qR0 = Rot.from_quat(m["qL"]), Rot.from_quat(m["qR"])
        t0 = self.b.now()
        while True:
            u = min(1.0, (self.b.now() - t0) / seconds)
            self.tick(pL + u * (self.cL - pL), qL0 * Rot.from_rotvec(u * (qL0.inv() * self.qL).as_rotvec()),
                      pR + u * (self.cR - pR), qR0 * Rot.from_rotvec(u * (qR0.inv() * self.qR).as_rotvec()),
                      K_TILT_SEAT)
            if u >= 1.0:
                return

    def approach(self, seconds=3.0):
        self.phase = "approach"
        t0, d = self.b.now(), 0.0
        while self.b.now() - t0 < seconds:
            d = V_APP * (self.b.now() - t0)
            self.tick(self.cL + d * self.axis, self.qL, self.cR - d * self.axis, self.qR, K_TILT_SEAT)
            if self.normal_force() > F_TOUCH:
                break
        self.cL, self.cR = self.cL + d * self.axis, self.cR - d * self.axis

    def squeeze(self, force_n, seconds=6.0):
        """Closed loop on the MEASURED normal force up to 2 x force_n (both hands)."""
        self.phase = "squeeze"
        t0, s, t_u = self.b.now(), 0.0, -1e9
        while self.b.now() - t0 < seconds:
            if self.b.now() - t_u >= CONTROL_PERIOD - 1e-9:
                t_u = self.b.now()
                f = self.normal_force()
                s = float(np.clip(s + np.clip(GAIN * (2 * force_n - f), -DS_MAX, DS_MAX), 0.0, S_MAX))
                if abs(f - 2 * force_n) < 0.05 * force_n:
                    break
            self.tick(self.cL + s * self.axis, self.qL, self.cR - s * self.axis, self.qR, K_TILT_HOLD)
        self.cL, self.cR = self.cL + s * self.axis, self.cR - s * self.axis

    def lower_until_seated(self, goal):
        """Down from the target until the shelf carries 60% of the weight with the
        piece believed within SEAT_BAND of its seat (Hands.lower_until_seated)."""
        self.phase = "seat_by_feel"
        goal = np.asarray(goal, float)
        c = np.asarray(self.c_cmd, float).copy()
        R = self.R_cmd
        if self.jammed and c[2] - goal[2] > 0.5 * self.r.raw["piece_model"]["cell"]:
            return float(goal[2] - c[2]), False
        want = self.land_share * self.r.mass * G
        t0, z0 = self.b.now(), c[2]
        while z0 - c[2] < SEAT_DROP:
            here = self.believed_height(goal)
            carried = self.external_force()[2] >= want
            if abs(here) <= self.seat_band and carried:
                return float(goal[2] - c[2]), True
            if carried and self.seat_stops and here < 0.200:
                self.landed_high = True
                return float(goal[2] - c[2]), False
            c[2] = z0 - SEAT_SPEED * (self.b.now() - t0)
            self.drive(c, R)
        return float(goal[2] - c[2]), False

    def release(self, by=0.10, seconds=2.0):
        """Open along the (rotated) axis; the hand-wrench offsets are kept."""
        self.phase = "release"
        f = self.frame
        c, R = self.c_cmd, self.R_cmd
        dR = R * f["R0"].inv()
        a = dR.apply(self.axis)
        d = self.ds * f["ax"]
        oL, oR = self.off["L"], self.off["R"]
        self.frame = None                       # the loops stop: nothing left to allocate
        t0 = self.b.now()
        while True:
            u = min(1.0, (self.b.now() - t0) / seconds)
            o = by * u
            self.tick(c + R.apply(f["oL"] + d) - o * a + oL[:3], Rot.from_rotvec(oL[3:]) * dR * self.qL,
                      c + R.apply(f["oR"] - d) + o * a + oR[:3], Rot.from_rotvec(oR[3:]) * dR * self.qR,
                      K_TILT_HOLD)
            if u >= 1.0:
                return

    def retreat(self, up=0.15):
        """ROBOT ONLY (the simulator parks the tools): both tools straight up."""
        self.phase = "retreat"
        m = self.meas
        pL, pR = np.asarray(m["pL"]), np.asarray(m["pR"])
        qL, qR = Rot.from_quat(m["qL"]), Rot.from_quat(m["qR"])
        t0, secs = self.b.now(), up / self.motion_speed
        while True:
            u = min(1.0, (self.b.now() - t0) / secs)
            self.tick(pL + [0, 0, up * u], qL, pR + [0, 0, up * u], qR, K_TILT_HOLD)
            if u >= 1.0:
                return

    def run(self, from_mouth=False):
        r = self.r
        print("  " + r.summary())
        self.b.tare()                           # F/T zero, hands in free air
        self.open_pose()
        self.confirm("hold the piece at the grasp pose between the pads (or close the fixture)")
        self.b.tare()                           # again, at the open pose, nothing touching
        self.approach()
        print(f"      approach: {self.normal_force():.1f} N measured")
        pickup = min(r.derived_N, PICKUP_N)
        self.squeeze(pickup)
        print(f"      squeeze: {self.normal_force():.1f} N measured, asked {2 * pickup:.1f}")
        self.confirm("let go of the piece (open the fixture)")
        R0 = r.rots[0]
        self.freeze(r.grasp_c, R0)
        self.R_rel = R0.inv() * Rot.from_quat(self.meas["qL"])
        self.R_cmd, self.c_cmd = R0, r.grasp_c
        c0 = r.centres[0]
        route_c, route_R, route_w = r.centres[:-1], r.rots[:-1], r.wext[:-1]
        arc = float(np.sum(np.linalg.norm(np.diff(route_c, axis=0), axis=1)))
        self.phase = "to_mouth"
        ok = self.move(r.grasp_c, R0, c0, R0, max(1.0, np.linalg.norm(r.grasp_c - c0) / self.motion_speed),
                       watch=True)
        if ok:
            self.phase = "route"
            ok = self.follow(route_c, route_R, route_w, max(4.0, arc / self.motion_speed))
        jammed = self.jammed
        self.w_ext = None
        from_c = route_c[-1] if ok else self.c_cmd
        from_R = route_R[-1] if ok else self.R_cmd
        self.jammed = False
        self.seat_goal, self.landed_high = r.target, False
        self.phase = "slow_seat"
        ok_seat = self.move(from_c, from_R, r.target, r.R_target,
                            max(2.0, np.linalg.norm(r.target - from_c) / self.seat_speed), watch=True)
        self.seat_goal = None
        if self.landed_high:
            ok_seat = False
        drop, sat = 0.0, False
        if ok_seat:
            drop, sat = self.lower_until_seated(r.target)
        believed = self.believed_height(r.target)
        self.release()
        self.retreat()
        out = dict(route_followed=ok, jammed=jammed or self.jammed, seat_found=sat,
                   landed_high=self.landed_high, drop_mm=1000 * drop, believed_mm=1000 * believed,
                   socp_ok=self.stats["socp_ok"], socp_ticks=self.stats["socp_ticks"],
                   side_force_max=self.stats["f_ext_max"],
                   squeeze_median=float(np.median(self.stats["asked"])) if self.stats["asked"] else float("nan"))
        print("  RESULT: " + ", ".join(f"{k} {v:.1f}" if isinstance(v, float) else f"{k} {v}"
                                       for k, v in out.items()))
        print("  measure the placement by hand (worst cell against its target) - the robot does not see it")
        return out
