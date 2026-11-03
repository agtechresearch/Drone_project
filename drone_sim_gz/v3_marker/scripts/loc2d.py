#!/usr/bin/env python3
"""Map-based 2-D localisation (v3_marker --wk-loc, 2026-10-30): the "where am I on the known map" half of SLAM, no mapping.

The rack map is a digital twin of the real room (same structure and dimensions; only the building's orientation and the
vehicle's set-down pose are unknown), so the vehicle pose ON THE MAP is estimated directly and continuously:
  state x = [px, py, cY]   px, py: body centre in the MAP frame (m), cY: VIO compass - map compass (deg),
                           i.e. map compass heading = PX4 yaw - cY  (the same cY as wk_mission's frame rotation)
  predict : PX4/VIO displacement, rotated into the map by -cY (VIO = odometry). Random walk: VIO drift and the PX4 heading
            wander seen with EKF2_EV_CTRL 7 (no VIO yaw: ~3.5 deg in 70 s, 2026-10-30 runs ev7d0_*)
  updates : takeoff prior   start cell of the plan, heading ~ the planned start heading (set down by hand)
            tag             id -> mapped post; observation = tag position in the camera (right, forward). The tag's own
                            rotation (PnP) is used only as a weak heading hint: +-5 deg per view (logs/tagtest/geom_20261030.json)
            ToF face        d (body -> face, perpendicular) -> across-aisle position; alpha -> heading (0.6 deg per frame)
  Every update is an EKF step with a chi-square gate (people, carts, plants in front of the face are rejected as outliers).
Python 3.6 + numpy (runs in starling-flight and on the VOXL)."""
import math
import os

import numpy as np

import tag_quality

CAM_FWD = 0.075              # camera ahead of the body centre (wk3/localize.py)
CAM_LEFT = 0.0
Q_POS = 0.012                # m/sqrt(s)   VIO drift (0.4-0.8 cm/s injected)
Q_YAW = 0.10                 # deg/sqrt(s) PX4 heading wander with EV_CTRL 7
P0_POS, P0_YAW = 0.15, 20.0  # takeoff prior: set down near the start cell, roughly facing the planned heading
TAG_SD0, TAG_SD_K = 0.03, 0.04   # m, tag position in the camera: sd = 0.03 + 0.04 * range (2.5 cm at 0.42 m, x2 for frame correlation)
TAG_YAW_SD = 6.0             # deg, PnP heading (biased per view geometry -> weak)
TAG_MAX_OFFAXIS = 0.8        # |right| / forward for the position update (~39 deg; the tag POSITION stays good off-axis:
                             # 2026-10-30 run loc_r3 hovered 14 s at the start cell with tag 30 at 0.5-0.8 and drifted 8.9 cm)
TAG_YAW_OFFAXIS = 0.45       # the weak PnP heading only near the image centre
TAG_MIN_GAP = 0.2            # s, one detection per tag per 0.2 s (consecutive frames are not independent)
TAG_SD_RANGE = False         # sd from the forward distance (68 deg sim camera) or from the full range
if os.environ.get("WK_SENSORS", "") == "real":   # tracking_front, extrinsics.conf (cameras_2026-10-30.zip)
    CAM_FWD, CAM_LEFT = 0.0665, 0.0065
    # undistorted fisheye, hfov 133 deg: at the start cell the tags are mostly 45-52 deg off the axis (run b1800real
    # 2026-10-30: off-axis 0.8 kept 10 updates in the first 4 s of the scan, then none for 25 s -> 8 cm along-aisle drift)
    TAG_MAX_OFFAXIS = float(os.environ.get("WK_TAG_OFFAXIS") or 1.4)
    TAG_SD_RANGE = True
TOF_D_SD = 0.02              # m   (0.01 per frame, x2 for frame correlation)
TOF_A_SD = 1.5               # deg (0.6 per frame, x2.5 for frame correlation)
TOF_MIN_CONF, TOF_MIN_SPAN, TOF_MAX_ALPHA = 0.5, 0.4, 20.0
TOF_MIN_D_ANGLE = 0.32       # m: closer, the trays leave the vertical FOV and alpha is biased +2..4 deg
TOF_USE_SD_POS, TOF_USE_SD_YAW = 0.10, 5.0   # face association needs a converged estimate
GATE = 13.8                  # chi-square, 2 dof 99.9 % (1 dof uses the same, conservative)
# WK_LOC_DRIFTV=1 (2026-10-31): two more states, the VIO drift velocity in the map (vx, vy). Learnt at the tags and applied between
# them (with only a random walk on the position, the along-aisle error grows between posts 4 m apart: R1C1 -3.5 cm).
DRIFTV = os.environ.get("WK_LOC_DRIFTV", "1") == "1"      # default on since 2026-10-31 (batches 1900dv0 / 1901dv1 / 1902early)
P0_DV, Q_DV = 0.01, 0.0005   # m/s initial sd, m/s/sqrt(s) random walk of the drift velocity
NX = 5 if DRIFTV else 3


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def rot_ne(n, e, deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    return n * c - e * s, n * s + e * c


class Loc2D(object):
    def __init__(self, tag_map, map_face):
        """tag_map: tags_map.json "tags" (id -> x, y, z, normal), map_face(mx, my, face_yaw) -> (name, d, h, face_yaw) or None"""
        self.tag_map = tag_map; self.map_face = map_face
        self.x = None; self.P = None
        self.vio_prev = None
        self.tag_last = {}
        self.n_tag = self.n_tof_d = self.n_tof_a = self.n_rej = 0
        self.last_tag = None; self.last_tof = None
        self.n_bad = 0; self.bad = None            # rejected non-finite inputs / reason the state became unusable
        self.tag_rej_streak = 0              # tag detections in a row that did not fit the map (lost / wrong map -> land)
        self.tag_recent = []                 # outcome (True = used) of the last tag detections that reached the gate

    # ---------------------------------------------------------------- prior / predict
    def init(self, vio, start_xy, start_compass):
        """vio = (n, e, yaw_deg) raw PX4 at takeoff; the vehicle was set down at start_xy facing ~start_compass"""
        self.x = np.array([start_xy[0], start_xy[1], wrap180(vio[2] - start_compass)] + [0.0, 0.0] * DRIFTV)
        self.P = np.diag([P0_POS ** 2, P0_POS ** 2, P0_YAW ** 2] + [P0_DV ** 2, P0_DV ** 2] * DRIFTV)
        self.vio_prev = vio
        if not (self.healthy() and all(math.isfinite(v) for v in vio)):   # 2026-10-31 codex nonfinite-measurements reply-2
            self.bad = self.bad or "state not finite after init (VIO %r)" % (vio,)

    def disp(self, vio_a, vio_b, cY):
        """map displacement (dx, dy) of the VIO motion a -> b"""
        dn, de = rot_ne(vio_b[0] - vio_a[0], vio_b[1] - vio_a[1], -cY)
        return de, dn

    def predict(self, vio, dt):
        if self.x is None:
            return
        if not (all(math.isfinite(v) for v in vio) and math.isfinite(dt)):
            self.n_bad += 1; self.bad = "non-finite VIO pose or time step"
            return
        a = self.vio_prev; self.vio_prev = vio
        dx, dy = self.disp(a, vio, self.x[2])
        self.x[0] += dx; self.x[1] += dy
        F = np.eye(NX)                                 # d(dx, dy)/d cY: a heading error turns the travelled vector
        ex, ey = self.disp(a, vio, self.x[2] + 1e-3)
        F[0, 2] = (ex - dx) / 1e-3; F[1, 2] = (ey - dy) / 1e-3
        q = [Q_POS ** 2 * dt, Q_POS ** 2 * dt, Q_YAW ** 2 * dt]
        if DRIFTV:                                     # position also moves by the learnt drift velocity
            self.x[0] += self.x[3] * dt; self.x[1] += self.x[4] * dt; F[0, 3] = dt; F[1, 4] = dt
            q += [Q_DV ** 2 * dt, Q_DV ** 2 * dt]
        self.P = F.dot(self.P).dot(F.T) + np.diag(q)
        if not self.healthy():
            self.bad = self.bad or "state / covariance not finite after predict"

    # ---------------------------------------------------------------- generic EKF update
    def _update(self, z, hfun, Rm, ang=(), gate=True):
        x0 = self.x.copy(); P0 = self.P.copy()
        z = np.asarray(z, float)
        if not np.all(np.isfinite(z)):                 # 2026-10-31 codex nonfinite-measurements: NaN passes "chi2 > GATE"
            self.n_bad += 1
            return False, float("nan"), z
        h0 = np.asarray(hfun(x0), float)
        H = np.zeros((len(z), NX))
        for j, eps in enumerate((1e-4, 1e-4, 1e-3, 1e-4, 1e-4)[:NX]):
            xp = x0.copy(); xp[j] += eps
            H[:, j] = (np.asarray(hfun(xp), float) - h0) / eps
        y = np.asarray(z, float) - h0
        for i in ang:
            y[i] = wrap180(y[i])
        S = H.dot(self.P).dot(H.T) + Rm
        if not (np.all(np.isfinite(h0)) and np.all(np.isfinite(H)) and np.all(np.isfinite(S))):
            self.n_bad += 1
            return False, float("nan"), y
        Si = np.linalg.inv(S)
        chi2 = float(y.dot(Si).dot(y))
        if not math.isfinite(chi2):
            self.n_bad += 1
            return False, chi2, y
        if gate and chi2 > GATE:
            self.n_rej += 1
            return False, chi2, y
        K = self.P.dot(H.T).dot(Si)
        self.x = x0 + K.dot(y); self.x[2] = wrap180(self.x[2])
        IKH = np.eye(NX) - K.dot(H)
        self.P = IKH.dot(self.P).dot(IKH.T) + K.dot(Rm).dot(K.T)
        if not self.healthy():                          # roll back and flag: the flight side treats it as lost
            self.x, self.P = x0, P0; self.bad = "update produced a non-finite or non-positive state"
            return False, chi2, y
        return True, chi2, y

    def healthy(self):
        return bool(np.all(np.isfinite(self.x)) and np.all(np.isfinite(self.P)) and np.all(np.diag(self.P) > 0))

    def sd(self):
        """standard deviations; NaN (never an exception) for a non-finite or non-positive variance, so every limit check
        written as `not (sd <= limit)` fails on it"""
        return tuple(math.sqrt(v) if math.isfinite(v) and v > 0 else float("nan") for v in (self.P[0, 0], self.P[1, 1], self.P[2, 2]))

    # ---------------------------------------------------------------- tag
    def update_tag(self, det, t, vio_t, vio_now, yaw_rate):
        """det: {"id", "t_cam": [right, down, fwd], "R", "margin", "hamming"} seen at time t; vio_t: raw PX4 then"""
        tg = self.tag_map.get(str(det["id"]))
        if self.x is None or tg is None or not tag_quality.ok(det, 30.0) or abs(yaw_rate) > 10.0:
            return None
        right, fwd = det["t_cam"][0], det["t_cam"][2]
        if fwd > 1.2 or abs(right) > TAG_MAX_OFFAXIS * fwd:
            return None
        if t - self.tag_last.get(det["id"], -1e9) < TAG_MIN_GAP:
            return None
        self.tag_last[det["id"]] = t

        def h(x):
            dx, dy = self.disp(vio_t, vio_now, x[2])          # pose at the image time = now - motion since then
            bx, by = x[0] - dx, x[1] - dy
            th = math.radians(90.0 - (vio_t[2] - x[2]))       # map heading (CCW from +x) at the image time
            f = (math.cos(th), math.sin(th)); r = (math.sin(th), -math.cos(th))
            vx, vy = tg["x"] - (bx + CAM_FWD * f[0] - CAM_LEFT * r[0]), tg["y"] - (by + CAM_FWD * f[1] - CAM_LEFT * r[1])
            return [vx * r[0] + vy * r[1], vx * f[0] + vy * f[1]]

        sd = TAG_SD0 + TAG_SD_K * (math.hypot(fwd, right) if TAG_SD_RANGE else fwd)
        ok, chi2, y = self._update([right, fwd], h, np.diag([sd ** 2, sd ** 2]))
        if ok:
            self.n_tag += 1
        self.tag_rej_streak = 0 if ok else self.tag_rej_streak + 1
        self.tag_recent = (self.tag_recent + [bool(ok)])[-15:]
        if ok and abs(right) <= TAG_YAW_OFFAXIS * fwd:
            # weak heading from the tag rotation (camera axis vs the tag normal)
            R = det["R"]; ang = math.degrees(math.atan2(R[0][2], R[2][2]))
            th_map = ang + (0.0 if tg["normal"] < 0 else 180.0)        # camera heading, CCW from +x
            comp = (90.0 - th_map) % 360.0
            self._update([comp], lambda x: [vio_t[2] - x[2]], np.diag([TAG_YAW_SD ** 2]), ang=(0,))
        self.last_tag = {"t": t, "id": det["id"], "ok": ok, "chi2": round(chi2, 2), "innov": [round(v, 4) for v in y]}
        return ok

    # ---------------------------------------------------------------- ToF face
    def update_tof(self, m, vio_t, vio_now, yaw_rate):
        """m: wall_sense frame {"t", "d" (body -> plane), "alpha" (deg, compass = face_yaw - alpha), "conf", "span"} measured at
        time m["t"]; vio_t: raw PX4 (n, e, yaw deg) AT that time, vio_now: raw PX4 at the state's time (vio_prev).
        2026-11-01 codex tof-time-alignment: the residual is built at the frame's time - the body position then is the state
        minus the VIO motion since (as for the tags) and the heading then is vio_t's yaw - so a frame and the state no
        longer describe two different poses while moving or turning. The caller hands frames in only once the pose history
        covers their time (wk_mission.loc_step).
        2026-11-01 codex survey-face-observation: map_face may return the face LINE (x0, k, y_ref, sgn): x_face(y) = x0 +
        k (y - y_ref) (a surveyed, turned face); the prediction is then the perpendicular distance
        sgn (x - x_face(y)) / sqrt(1 + k^2), with its y derivative, the same geometry as the batch survey residual."""
        if self.x is None or m.get("d") is None:
            return None
        sx, sy, syaw = self.sd()
        if max(sx, sy) > TOF_USE_SD_POS or syaw > TOF_USE_SD_YAW:
            return None
        if m["conf"] < TOF_MIN_CONF or m["span"] < TOF_MIN_SPAN or abs(m["alpha"]) > TOF_MAX_ALPHA:
            return None
        comp = (vio_t[2] - self.x[2]) % 360.0                   # map compass heading at the frame's time
        face_yaw = 270.0 if abs(wrap180(comp - 270.0)) < 25.0 else 90.0 if abs(wrap180(comp - 90.0)) < 25.0 else None
        dx0, dy0 = self.disp(vio_t, vio_now, self.x[2])
        mf = self.map_face(self.x[0] - dx0, self.x[1] - dy0, face_yaw) if face_yaw is not None else None
        if mf is None:
            return None
        name, fy = mf[0], mf[3]
        sgn = 1.0 if name.endswith("W") else -1.0              # W face: d = px - x_face; E face: d = x_face - px (fy may be turned)
        if len(mf) > 4 and mf[4] is not None:
            x0f, k, y_ref = mf[4][0], mf[4][1], mf[4][2]
        else:                                                  # old 4-tuple map_face: a face parallel to y through the given point
            x0f, k, y_ref = (self.x[0] - dx0) - sgn * mf[1], 0.0, 0.0
        nk = math.sqrt(1.0 + k * k)

        def h_d(x):
            dx, dy = self.disp(vio_t, vio_now, x[2])
            bx, by = x[0] - dx, x[1] - dy                       # body at the frame's time
            return [sgn * (bx - (x0f + k * (by - y_ref))) / nk]

        res = {"face": name, "d": m["d"], "alpha": m["alpha"], "t": m.get("t")}
        ok_d, chi_d, y_d = self._update([m["d"]], h_d, np.diag([TOF_D_SD ** 2]))
        res["ok_d"] = ok_d; res["nu_d"] = round(float(y_d[0]), 5); self.n_tof_d += ok_d
        if m["d"] >= TOF_MIN_D_ANGLE and abs(yaw_rate) <= 10.0:
            ok_a, chi_a, y_a = self._update([m["alpha"]], lambda x: [wrap180(fy - (vio_t[2] - x[2]))], np.diag([TOF_A_SD ** 2]), ang=(0,))
            res["ok_a"] = ok_a; res["nu_a"] = round(float(y_a[0]), 4); self.n_tof_a += ok_a
        self.last_tof = res
        return res

    # ---------------------------------------------------------------- output as wk_mission's frame correction
    @staticmethod
    def to_c(px, py, cY, raw_now, origin, spawn):
        """(cY, cN, cE) such that wk_mission's est_map(raw_now) == (px, py) with the frame rotated by cY about the VIO
        origin (--wk-rotate): the correction is applied about the vehicle, so a heading change does not swing it sideways"""
        on, oe = origin
        dn, de = rot_ne(raw_now[0] - on, raw_now[1] - oe, -cY)
        return cY, dn - (py - spawn[1]), de - (px - spawn[0])
