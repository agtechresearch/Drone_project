#!/usr/bin/env python3
"""grid_mission.py + wall-keep layer (v2_wallkeep, 2026-10-29). Runs the UNCHANGED real flight script
path_flight_phase1_v13.py exactly like grid_mission.py does, and inserts a frame-correction layer between the script and
PX4 when --wallkeep on:

  setpoint sent to PX4   = script target + c        (c = (cN, cE) metres, cY degrees)
  feedback seen by script = PX4 estimate  - c
so the script keeps flying its plan in "map" coordinates while c absorbs what the estimate gets wrong. c is estimated
from the front depth sensor (wall_sense.py, UDP JSON per depth frame):
  - position: innovation nu = d_measured - d_predicted, d_predicted = distance from the corrected estimate to the rack
    face in the QVIX map (qvix_grid) for the aisle the vehicle is in and the face it looks at (heading W or E);
    c += (dt / TAU_POS) * nu * h  (h = unit vector toward the face), rate-limited, capped,
  - heading: the face's true direction is known (W face = 270, E face = 90) -> true yaw = face yaw - alpha_measured;
    cY follows (estimate yaw - true yaw) with TAU_YAW.
Updates only when the measurement is trustworthy: heading within 0.5 deg of 90/270 (survey legs and climbs, not turns),
above MIN_ALT, inside the rack span of the room +-MAP_MARGIN (map), conf >= MIN_CONF, inlier span >= 0.3 m, |nu| < MAX_NU, |alpha| < 12 deg.
Otherwise c is held (last correction kept). With --wallkeep off the layer only LOGS (c stays 0), so the CSV shows what
the sensor saw in the uncorrected runs too.
Usage (starling-flight, cd ~/workspace):
  python3 v2_wallkeep/scripts/wk_mission.py --wallkeep on --wk-port 14652 --wk-csv logs/x_wk.csv --route "..." [grid_mission args] -- --port 50053
Python 3.6.
"""
import asyncio
import csv
import json
import math
import os
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import grid_mission as GM  # noqa: E402
import safety_patch as SP  # noqa: E402   (2026-11-01: shared with real_flight_safe.py)
import qvix_grid as G  # noqa: E402
import loc2d  # noqa: E402
import tag_quality  # noqa: E402   (2026-11-01 codex real-tag-quality-contract)

TAU_POS = 0.3          # s, position-correction time constant
TAU_YAW = 0.8          # s
RATE_POS = 0.15        # m/s max change of c
RATE_YAW = 5.0         # deg/s
CAP_POS = 0.35         # m   around the last tag reference (cN0, cE0) ...
CAP_GROW = 0.005       # m/s ... growing with the time since that reference: a fixed cap around the scan fix was used up by
                       # 0.4 cm/s of lateral drift after ~90 s and the ToF channel then stood at nu = -0.10 m (2026-10-29 run lk_off_d4)
CAP_YAW = 8.0          # deg
ROTOR_FWD = G.ROTOR_RADIUS_FWD
MIN_ALT = 0.45         # m above the start: survey heights are >= 0.68 above the floor; below this (preland, takeoff) the
                       # sensor sees the lowest tray/base structure, not the face (2026-10-29 run D: c jumped 0.10 m on preland)
MAP_MARGIN = 0.10      # m beyond the rack ends where the face is still accepted
MIN_CONF = 0.3         # inlier fraction; at a rack end only ~half the image sees the face (conf ~0.49) but the plane is
                       # clean (2026-10-29 run F2: 0.5 skipped 8 s at row 1 -> 4.1 cm drift). The 0.3 m inlier span is the real guard.
MAX_NU = 0.12          # m, innovation gate
# alignment hold (--wk-align, 2026-10-29): before the first survey leg, hover facing the rack until the correction has
# converged and the vehicle sits on the corrected target, so the leg does not start with the drift/heading error that built
# up during takeoff and the turn (no correction there on purpose: no face in view, low height, fast yaw).
ROT_LEVER_RATE = 0.02  # m/s, --wk-rotate: max sideways setpoint speed caused by changing cY (lever arm from the VIO origin)
# --wk-tof-heading (2026-10-30): the tag fix's heading comes from the PnP rotation of a 4 cm tag, which is off by up to
# +-5 deg per view (logs/tagtest/geom_20261030.json: -4.5 deg even square-on and centred at 0.42 m). The ToF plane angle of
# the rack face is accurate to ~0.6 deg per frame, so the scan's heading is taken from the ToF frames recorded while the
# vehicle faced a rack face during the scan; the tags give the face identity, the coarse heading and the position.
# A ToF frame counts only if it passes every check (real racks also show plants, people, carts):
TOF_MIN_CONF = 0.5     # inlier fraction of the valid columns
TOF_MIN_SPAN = 0.4     # m lateral extent of the inliers (a post or a leaf is narrower)
TOF_MIN_D = 0.32       # m  closer than ~0.30 m the trays leave the vertical FOV and the plane tilts: +2.3..+3.7 deg
                       # (2026-10-30 sense.csv, live runs: 0.26-0.30 m mean +2.25 deg; 0.33-0.40 m mean -0.05..+0.06 deg)
TOF_MAX_ALPHA = 20.0   # deg plane angle (the scan stops face the rack within the VIO heading error + world turn)
TOF_MAX_RATE = 2.0     # deg/s yaw rate (still, not turning)
TOF_GATE_D = 0.05      # m |measured - map-predicted distance| from the coarse tag fix: the plane must BE the mapped face
TOF_GATE_YAW = 6.0     # deg |ToF heading - tag heading|
TOF_MIN_N = 10         # frames after the robust (median +- 3 MAD) cut, else the tag heading is kept
# The ToF frames are taken in a still hold AFTER the tag fix, at the first stop facing a rack face (s00, W), not during the
# scan: PX4's heading trails the VIO input in the 25 deg/s scan turns and recovers over ~20 s (2026-10-30 batch 1110: -0.45 deg
# at the 90 deg stop where CE was seen, -0.2 deg at the fix), so scan frames gave the heading of that moment: -0.30 deg mean.
# At the cell centre the sensor is only ~0.30 m from the face (inside TOF_MIN_D), so the hold steps back from the face
# sideways (no turn: a turn would bring the PX4 heading error back) and returns to the cell afterwards.
TOF_HOLD_S = 6.0       # s sim time held
TOF_HOLD_SKIP = 2.5    # s of it skipped (the step back)
TOF_HOLD_STEP = 0.07   # m away from the face -> ~0.37 m, the leg standoff
POSE_LAG = float(os.environ.get("WK_POSE_LAG", "0.23"))  # s, --wk-loc: age of the PX4 POSITION when read and of the
YAW_LAG = float(os.environ.get("WK_YAW_LAG", "0.12"))    # ATTITUDE (separate MAVSDK streams, each polled at 20 Hz). SITL fit on
                       # batch 1710loc (4 runs): position 0.225-0.24 s, yaw 0.115-0.125 s; with one 0.2 s lag the heading led the
                       # truth by 0.08 s = 2.5 deg rms in the 30 deg/s turns. Real vehicle: take both from the pipe timestamps.
TOF_MIN_VALID = float(os.environ.get("WK_TOF_MIN_VALID", "0.25"))   # ToF health: valid-pixel fraction below this ...
TOF_BLIND_S, TOF_BLIND_LAND_S = 2.0, 5.0     # ... for 2 s = blind (ToF updates off), for 5 s in flight = land (2026-10-31: sv2
                                             # world, 0.06-0.09 valid; normal 0.48-1.00 with the ToF model, 0.61-1.00 ideal)
# ---- safety (2026-10-31 codex review, feedback/open/*): stale inputs, task failure, stop without kill ----
TOF_NOFRAME_S = 3.0          # s airborne without any ToF frame -> blind
TOF_STALE_S = 1.0            # s since the last ToF frame (current clock) -> blind
TOF_Q_MAX_AGE = 0.5          # s: --wk-loc drops a ToF frame this much older than the pose time it would be applied at
POSE_STALE_S = 1.0           # s since the last position OR attitude message (each stream on its own receive time) -> land
YAW_FRESH_S = SP.YAW_FRESH_S # s: an attitude older than this is not a yaw (safety_patch.feedback_state gives NaN)
CLOCK_LOST_S = 3.0           # s of WALL time without a sim clock datagram -> land (the loop clock is only extrapolated then)


def clock_age():
    """wall seconds since the last sim clock datagram (grid_mission.install_sim_clock_loop), None without the sim clock"""
    f = getattr(sys.modules.get("grid_mission"), "SIM_CLOCK_AGE", None)
    return f() if f else None


def _finite(*v):
    """every value a real number (int / float, not bool, not a numeric string: the raw value is what the estimator uses,
    2026-10-31 codex nonfinite-measurements reply-2) and finite"""
    return all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)


def valid_tag_det(d):
    """tag_detections entry: id int, t_cam 3 finite, R 3x3 finite, margin finite number or null, hamming int or null.
    2026-11-01 codex real-tag-quality-contract: a null margin / hamming (real VOXL detector: no decision margin) is a valid
    record with UNAVAILABLE quality; whether it is used is tag_quality.ok()'s decision (default: rejected), not a crash"""
    try:
        return (isinstance(d.get("id"), int) and len(d["t_cam"]) == 3 and _finite(*d["t_cam"]) and len(d["R"]) == 3
                and all(len(r) == 3 and _finite(*r) for r in d["R"]) and (d.get("margin") is None or _finite(d["margin"]))
                and (d.get("hamming") is None or (isinstance(d["hamming"], int) and not isinstance(d["hamming"], bool))))
    except (TypeError, KeyError, AttributeError):
        return False


def valid_tof_msg(m):
    """wall_sense frame: t finite; d null (no plane) or d/alpha/conf/span finite; valid (if present) finite in [0, 1]"""
    try:
        if not _finite(m["t"]):
            return False
        if m.get("valid") is not None and not (_finite(m["valid"]) and 0.0 <= m["valid"] <= 1.0):
            return False
        return m.get("d") is None or _finite(m["d"], m["alpha"], m["conf"], m["span"])
    except (TypeError, KeyError, AttributeError):
        return False


LOC_EARLY = os.environ.get("WK_LOC_EARLY", "1") == "1"   # apply the estimate as soon as it is good (not only at the scan end)
LOC_EARLY_TAGS, LOC_EARLY_SD_POS, LOC_EARLY_SD_YAW = 5, 0.04, 2.0
LOC_LOST_TAG_REJ, LOC_LOST_SD = 8, 0.12
LOC_LOST_REJ_N, LOC_LOST_REJ_FRAC = 10, 0.6  # ... or more than 60 % of the last 10-15 tag detections rejected   # --wk-loc in flight: this many tag detections in a row rejected, or position sd above -> land
LOC_MAX_SD_POS, LOC_MAX_SD_YAW = 0.06, 4.0   # --wk-loc: estimate must be at least this good when the scan ends (else land)
LOC_RATE_POS = 0.15    # m/s   --wk-loc: applied frame follows the estimate at most this fast
LOC_RATE_YAW = 3.0     # deg/s
ALIGN_NU = 0.01        # m  |distance innovation|
ALIGN_NUY = 0.5        # deg |heading innovation|
ALIGN_POS = 0.015      # m  vehicle (corrected estimate) to target
ALIGN_HOLD = 1.0       # s  all three held this long
ALIGN_TIMEOUT = 4.0    # s  start the leg anyway after this
GUARD_CLEAR = 0.04     # warn when the rotor tip is closer than this to the face (measured)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def rot_ne(n, e, deg):
    """rotate the NED vector (n, e) by yaw deg (clockwise positive, like NED heading)"""
    a = math.radians(deg); c_, s_ = math.cos(a), math.sin(a)
    return n * c_ - e * s_, n * s_ + e * c_


# mid-route tag re-check (--wk-tag-recheck, v3 2026-10-29): when the camera passes a post tag during a leg, the map pose
# from the tag is compared with the corrected estimate at the image time. One correction per PASS (all detections of
# that tag, applied after the tag has left the view), not per frame, so a single bad frame cannot jerk the vehicle.
RC_GAIN_ALONG = 1.0    # along the aisle: nothing else observes it (ToF sees the face distance only)
RC_GAIN_LAT = 0.5      # across the aisle and heading: shared with the continuous ToF channel
RC_GAIN_YAW = 0.5
RC_MIN_N = 3           # detections per pass
# gate: the tag ids are unique per post, so a large innovation is not a wrong association; it means the estimate has drifted.
# A fixed 0.15 m gate locked the correction out for good once the drift had passed it (2026-10-29 run rc_rcon_d8: 0.23 ->
# 0.44 m rejected, vehicle overshot the aisle end and crashed). The gate grows with the time since the last applied
# correction; a bad pose is screened by the per-pass consistency (RC_MIN_N, RC_MAX_SD) instead.
RC_GATE_POS = 0.15     # m   right after a correction
RC_GATE_GROW = 0.02    # m/s (drift rates up to 2 cm/s)
RC_GATE_MAX = 0.60     # m   (< half the 3.95 m post spacing)
RC_GATE_YAW = 3.0      # deg right after a correction, + 0.1 deg/s up to 8 deg
RC_MAX_SD = 0.02       # m   spread of the along/lateral innovations inside one pass
RC_PASS_GAP = 0.5      # s   no detection of the tag for this long = pass finished
RC_RATE_POS = 0.10     # m/s the correction is fed into c as a ramp
RC_RATE_YAW = 1.0      # deg/s


class WallKeep(SP.SafetyState):
    def __init__(self, enabled, port, csv_path, spawn_map_xy):
        SP.SafetyState.__init__(self)      # feedback stamps, landing state (safety_patch.py)
        self.enabled = enabled
        self.cN = self.cE = self.cY = 0.0
        self.spawn = spawn_map_xy          # v3: MAP (model) coordinates of the planned start cell = the flight script's origin
        self.cN0 = self.cE0 = 0.0; self.cap_t0 = None
        self.recheck = False; self.rc_seen = 0.0; self.rc_pass = {}; self.rc_pending = [0.0, 0.0, 0.0]; self.rc_events = []
        self.rc_t = None; self.rc_last_ok = None; self.tag_fix_t = 0.0
        self.lost = None; self.lost_done = False; 
        self.tof_valid = []; self.tof_blind_since = None; self.tof_blind_logged = False
        self.tof_rx_t = None; self.airborne_since = None; self.low_since = None; self.n_bad_tof = 0; self.n_bad_tag = 0
        self.tof_blind_why = None
        self.scan_checked = False
        # feedback freshness per stream (2026-10-31 codex feedback-pose-freshness reply-2): receive time (loop clock) of the last
        # position / attitude message, stamped where the message arrives (patched MAVSDK telemetry generators, install()); on
        # the real vehicle (VOXL pipe, one message with both) the pipe's own timestamp changing counts as a new message
        self.clock = None; self.tof_rx_local = None; self.tof_t_last = None; self.airborne = False; self.blind_now = False
        self.loc = None; self.raw_yaw = []; self.loc_applied = None; self.loc_t = 0.0; self.loc_log_t = 0.0
        self.tof_hist = []; self.tof_rec = False; self.tof_heading = False; self.cY0 = 0.0; self.tags = False; self.ready = True; self.tag_frames = []; self.hist = []; self.tag_fix = None
        self.lock = threading.Lock()
        self.meas = None
        self.tof_q = []; self.n_tof_old = 0; self.n_tof_nopose = 0   # --wk-loc ToF frames waiting for their pose time
        self.pose_f = None
        self.last_t = None
        self.n_upd = 0
        self.align = False         # --wk-align
        self.rotate = False        # --wk-rotate: also rotate the setpoint/feedback frame by cY about the VIO origin
        self.origin = None         # (N, E) of the VIO origin in the PX4 frame = the vehicle's initial position
        self.aligning = False
        self.align_done = False
        self.last = None           # last measurement evaluation: state, nu, nu_y
        self.ctrl = None           # --wk-policy: wk3.controllers object for the lateral channel (None = v2 estimator below)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        threading.Thread(target=self._rx, daemon=True).start()
        self.f = open(csv_path, "w")
        self.w = csv.writer(self.f)
        self.w.writerow(["wall", "t_sim", "enabled", "state", "stage", "target_yaw", "est_wx", "est_wy", "face", "d_meas", "alpha_meas",
                         "d_pred", "nu", "yaw_est", "yaw_true_est", "cN", "cE", "cY", "conf", "span"])

    def _rx_tags(self, port):
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); sk.bind(("127.0.0.1", port))
        while True:
            try:
                m = json.loads(sk.recv(65536).decode())
            except ValueError:
                continue
            if not isinstance(m, dict) or not _finite(m.get("t")) or not isinstance(m.get("dets"), list):
                self.n_bad_tag += 1
                continue
            good = [d for d in m["dets"] if isinstance(d, dict) and valid_tag_det(d)]
            self.n_bad_tag += len(m["dets"]) - len(good)
            m["dets"] = good
            if m.get("dets"):
                with self.lock:
                    self.tag_frames.append(m)
                    self.tag_frames = self.tag_frames[-600:]

    def enable_tags(self, port, tag_map_path):
        self.tags = True; self.ready = False
        jm = json.load(open(tag_map_path))
        self.tag_map = jm["tags"]
        self.face_corr = jm.get("faces", {})        # surveyed map: rack face offset (m) and turn (deg) about y_ref
        # 2026-11-01 (codex survey-validation-independence, fork A): survey_map.py marks a map "eligible": false when the data
        # alone pin it down less well than the priors - such a map is refused unless WK_MAP_ALLOW_INELIGIBLE=1; a map without
        # the mark (the design map, older surveys) is flown and logged as "not checked"
        obs = jm.get("observability")
        self.map_eligible = None if not isinstance(obs, dict) else obs.get("eligible")
        if self.map_eligible is False and os.environ.get("WK_MAP_ALLOW_INELIGIBLE", "0") != "1":
            raise SystemExit("[wk_mission] map %s is marked NOT eligible by its survey (observability) - not flying on it "
                             "(WK_MAP_ALLOW_INELIGIBLE=1 to fly anyway)" % tag_map_path)
        print("[wk_mission] map eligibility: %s" % {None: "not checked (no observability mark)", True: "eligible",
                                                     False: "NOT eligible - flown on WK_MAP_ALLOW_INELIGIBLE=1"}[self.map_eligible], flush=True)
        print("[wk_mission] map %s: %d tags, face corrections %s" % (os.path.basename(tag_map_path), len(self.tag_map),
              {k: (v["offset"], v["turn_deg"]) for k, v in self.face_corr.items()}), flush=True)
        threading.Thread(target=self._rx_tags, args=(port,), daemon=True).start()

    def yaw_at(self, t_arr):
        """--wk-loc: attitude yaw (rad) interpolated at ARRIVAL time t_arr from the raw 20 Hz samples"""
        h = self.raw_yaw
        for i in range(len(h) - 1, 0, -1):
            if h[i - 1][0] <= t_arr <= h[i][0]:
                a, b = h[i - 1], h[i]
                f = (t_arr - a[0]) / max(1e-6, b[0] - a[0])
                return a[1] + f * math.atan2(math.sin(b[1] - a[1]), math.cos(b[1] - a[1]))
        return None

    def pose_at(self, t):
        """raw PX4 (n, e, yaw rad) interpolated at sim time t from the 20 Hz history, and the yaw rate (deg/s)"""
        h = self.hist
        for i in range(1, len(h)):
            if h[i][0] >= t:
                a, b = h[i - 1], h[i]
                if b[0] - a[0] > 0.3 or t < a[0]:
                    return None
                f = (t - a[0]) / max(1e-6, b[0] - a[0])
                dy = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
                return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dy), math.degrees(dy) / max(1e-6, b[0] - a[0])
        return None

    def fix_from_tags(self, origin, log, dpsi_override=None):
        """map frame from the scan: per tag detection -> body pose on the map (wk3.localize) + PX4 pose at the same sim time
        -> (dpsi, t); robust mean -> cY, (cN, cE) so that  PX4 target = origin + R(cY) (script target + c - origin)"""
        sys.path.insert(0, os.path.join(HERE, ".."))
        from wk3 import localize as LZ
        with self.lock:
            frames = list(self.tag_frames)
        samples = []
        for fr in frames:
            pr = self.pose_at(fr["t"])
            if pr is None or abs(pr[1]) > 4.0:          # only while (nearly) still: the scan hovers 1.5 s per 90 deg
                continue
            for d in fr["dets"]:
                tg = self.tag_map.get(str(d["id"]))
                if tg is None or not tag_quality.ok(d, 30.0) or math.hypot(d["t_cam"][0], d["t_cam"][2]) > 1.2 \
                        or abs(d["t_cam"][0]) > 0.45 * d["t_cam"][2]:   # off the image edge (> ~24 deg from the axis): pose less reliable
                    continue
                bp = LZ.body_pose_from_tag(d["t_cam"], d["R"], tg)
                dpsi, tv = LZ.frame_from_pose(bp, pr[0])
                samples.append((dpsi, tv[0], tv[1], d["id"], bp, fr["t"], math.degrees(pr[0][2]), d["t_cam"], pr[0]))
        if len(samples) < 3:
            log("[tags] NO FIX: %d usable detections (frames %d)" % (len(samples), len(frames)))
            return False
        ds = sorted(x[0] for x in samples); med = ds[len(ds) // 2]
        keep = [x for x in samples if abs(LZ.wrap(x[0] - med)) < math.radians(1.5)]
        sx = sum(math.sin(x[0]) for x in keep); cx_ = sum(math.cos(x[0]) for x in keep)
        dpsi = math.atan2(sx, cx_)
        dpsi_tag = dpsi
        if dpsi_override is not None:
            dpsi = dpsi_override
        # translation with the COMMON rotation, not each detection's own: the map origin is ~8.5 m from the vehicle, so a
        # 1 deg per-tag heading bias moves that detection's t by ~15 cm (2026-10-30 run live_on_d4: tag 38 -0.31 deg at
        # t 36 s, tag 30 +1.28 deg at t 61 s -> the time fit read it as -1.0 cm/s drift and the fix was 0.25 m off)
        c_, s_ = math.cos(dpsi), math.sin(dpsi)
        tr = [(x[8][0] - (c_ * x[4][1] - s_ * x[4][0]), x[8][1] - (s_ * x[4][1] + c_ * x[4][0])) for x in keep]
        tN = sum(v[0] for v in tr) / len(tr); tE = sum(v[1] for v in tr) / len(tr)
        # translation: the scan takes ~40 s and the VIO drifts meanwhile, so the mean lags by ~half the scan's drift
        # (2026-10-29 runs rc_*_d8: 0.21 m right after the fix at 0.8 cm/s). Fit t(time) linearly and take it at the fix time.
        ts = [x[5] for x in keep]; t_now = self.hist[-1][0] if self.hist else max(ts)
        tm = sum(ts) / len(ts); var = sum((v - tm) ** 2 for v in ts)
        drift = (0.0, 0.0)
        if max(ts) - min(ts) > 5.0 and var > 0:
            bN = sum((x[5] - tm) * (v[0] - tN) for x, v in zip(keep, tr)) / var; bE = sum((x[5] - tm) * (v[1] - tE) for x, v in zip(keep, tr)) / var
            tN += bN * (t_now - tm); tE += bE * (t_now - tm); drift = (bN, bE)
        self.fix_ref = (t_now, drift)                  # the translation holds at t_now; at time t it was c - drift (t_now - t)
        m0N, m0E = self.spawn[1], self.spawn[0]        # planned start cell (map N = y, E = x) = flight-script origin
        a = -dpsi; c_, s_ = math.cos(a), math.sin(a)
        vN, vE = tN - origin[0], tE - origin[1]
        self.cN = c_ * vN - s_ * vE + m0N
        self.cE = s_ * vN + c_ * vE + m0E
        self.cY = math.degrees(dpsi)
        self.cY0 = self.cY                         # CAP_YAW now limits the ToF yaw channel around the tag fix
        self.cN0, self.cE0 = self.cN, self.cE      # CAP_POS likewise
        self.rc_seen = max(fr["t"] for fr in frames) if frames else 0.0
        self.tag_fix_t = self.rc_seen
        self.cap_t0 = self.rc_seen
        self.rotate = True; self.ready = True
        ids = sorted({x[3] for x in keep})
        self.tag_fix = {"n_samples": len(samples), "n_used": len(keep), "ids": ids, "dpsi_deg": self.cY, "cN": self.cN, "cE": self.cE,
                        "spread_deg": math.degrees(max(abs(LZ.wrap(x[0] - dpsi_tag)) for x in keep)), "dpsi_tag_deg": math.degrees(dpsi_tag), "drift_rate_ne": list(drift), "t_span": max(ts) - min(ts),
                        # per detection, for the accuracy analysis (tagfix_eval.py): sim time, id, dpsi, PX4 yaw, map body pose, tag in camera
                        "samples": [{"t": x[5], "id": x[3], "dpsi_deg": math.degrees(x[0]), "px4_yaw_deg": x[6], "map_pose": list(x[4]),
                                     "t_cam": x[7], "px4_ne": [x[8][0], x[8][1]], "used": any(x is k for k in keep)} for x in samples]}
        log("[tags] FIX: %d/%d detections, tags %s, frame rotation %+.2f deg (spread %.2f), c = (N%+.3f, E%+.3f), "
            "drift fit N%+.4f E%+.4f m/s over %.1f s" % (len(keep), len(samples), ids, self.cY, self.tag_fix["spread_deg"],
                                                        self.cN, self.cE, drift[0], drift[1], max(ts) - min(ts)))
        return True

    def tof_dpsi(self, log, t_min=0.0):
        """heading of the VIO frame from the ToF face angle (see TOF_* above), using the coarse tag fix (c, cY) already set
        to identify the face and predict its distance. Returns (dpsi rad or None, info dict)."""
        with self.lock:
            hist = list(self.tof_hist)
        cnt = {"frames": len(hist), "quality": 0, "still": 0, "face": 0, "dist": 0, "yaw": 0}
        cand = []
        for m in hist:
            if m["t"] < t_min:
                continue
            if m["conf"] < TOF_MIN_CONF or m["span"] < TOF_MIN_SPAN or abs(m["alpha"]) > TOF_MAX_ALPHA or m["d"] < TOF_MIN_D:
                continue
            cnt["quality"] += 1
            pr = self.pose_at(m["t"])
            if pr is None or abs(pr[1]) > TOF_MAX_RATE:
                continue
            cnt["still"] += 1
            # the fix's translation is valid at fix_ref time; the VIO had drifted less at this frame's time
            # (2026-10-30 run live_tofh: without this the CE frames 23 s earlier were 9 cm off the map and rejected)
            t_ref, (bN, bE) = self.fix_ref
            raw = (pr[0][0] + bN * (t_ref - m["t"]), pr[0][1] + bE * (t_ref - m["t"]), pr[0][2])
            mx, my, mh = self.est_map(raw)
            comp = (90.0 - mh) % 360.0                      # coarse map compass heading from the tag fix
            face_yaw = 270.0 if abs(wrap180(comp - 270.0)) < 25.0 else 90.0 if abs(wrap180(comp - 90.0)) < 25.0 else None
            mf = self.map_face(mx, my, face_yaw) if face_yaw is not None else None
            if mf is None:
                continue
            cnt["face"] += 1
            # map_face gives the distance along the face normal; the sensor reports the perpendicular distance too
            if abs(m["d"] - mf[1]) > TOF_GATE_D:
                continue
            cnt["dist"] += 1
            true_comp = face_yaw - m["alpha"]
            dp = wrap180(math.degrees(pr[0][2]) - true_comp)
            if abs(wrap180(dp - self.cY)) > TOF_GATE_YAW:
                continue
            cnt["yaw"] += 1
            cand.append((dp, m["t"], mf[0]))
        info = dict(cnt)
        if len(cand) >= TOF_MIN_N:
            ds = sorted(c[0] for c in cand); med = ds[len(ds) // 2]
            mad = sorted(abs(wrap180(c[0] - med)) for c in cand)[len(cand) // 2] * 1.4826
            kept = [c for c in cand if abs(wrap180(c[0] - med)) <= max(0.5, 3.0 * mad)]
            info.update(kept=len(kept), median_deg=med, mad_deg=mad, faces=sorted({c[2] for c in kept}),
                        t_span=max(c[1] for c in kept) - min(c[1] for c in kept) if kept else 0.0)
            if len(kept) >= TOF_MIN_N:
                s_ = sum(math.sin(math.radians(c[0])) for c in kept); c_ = sum(math.cos(math.radians(c[0])) for c in kept)
                dp = math.degrees(math.atan2(s_, c_))
                info["dpsi_deg"] = dp
                log("[tof-heading] %d/%d frames (quality %d, still %d, face %d, dist %d, yaw %d), faces %s, "
                    "dpsi %+.2f deg (tag %+.2f, MAD %.2f)" % (len(kept), len(hist), cnt["quality"], cnt["still"], cnt["face"],
                                                              cnt["dist"], cnt["yaw"], info["faces"], dp, self.cY, mad))
                return math.radians(dp), info
        log("[tof-heading] NOT USED: %d usable frames of %d (quality %d, still %d, face %d, dist %d, yaw %d) -> tag heading kept"
            % (len(cand), len(hist), cnt["quality"], cnt["still"], cnt["face"], cnt["dist"], cnt["yaw"]))
        return None, info

    def est_map(self, raw):
        """raw PX4 (n, e, yaw rad) -> corrected MAP pose (x, y, heading deg CCW from +x) with the CURRENT c / cY,
        the same transform as feedback_state() + step() (map x = spawn x + rel_e, map y = spawn y + rel_n)"""
        n, e, yaw = raw
        on, oe = self.origin
        if self.rotate:
            dn, de = rot_ne(n - on, e - oe, -self.cY); n, e = on + dn, oe + de
        n -= self.cN; e -= self.cE
        return self.spawn[0] + (e - oe), self.spawn[1] + (n - on), wrap180(90.0 - (math.degrees(yaw) - self.cY))

    def recheck_step(self, now, log):
        """called at 20 Hz (sim time now) once the map frame is fixed: collect tag passes, apply finished ones, ramp c"""
        sys.path.insert(0, os.path.join(HERE, ".."))
        from wk3 import localize as LZ
        with self.lock:
            frames = [fr for fr in self.tag_frames if fr["t"] > self.rc_seen]
        for fr in frames:
            self.rc_seen = max(self.rc_seen, fr["t"])
            pr = self.pose_at(fr["t"])
            if pr is None or abs(pr[1]) > 4.0:
                continue
            for d in fr["dets"]:
                tg = self.tag_map.get(str(d["id"]))
                if tg is None or not tag_quality.ok(d, 30.0) or math.hypot(d["t_cam"][0], d["t_cam"][2]) > 1.2 \
                        or abs(d["t_cam"][0]) > 0.45 * d["t_cam"][2]:
                    continue
                bx, by, bh = LZ.body_pose_from_tag(d["t_cam"], d["R"], tg)
                self.rc_pass.setdefault(d["id"], []).append((fr["t"], bx, by, math.degrees(bh), pr[0]))
        for tid in list(self.rc_pass):
            ps = self.rc_pass[tid]
            if now - ps[-1][0] < RC_PASS_GAP:
                continue
            del self.rc_pass[tid]
            # innovations with the CURRENT c for every sample (c may have moved during the pass), see below for the pending part
            ps = [(p[0], p[1] - e[0], p[2] - e[1], wrap180(p[3] - e[2])) for p in ps for e in [self.est_map(p[4])]]
            dx = sorted(p[1] for p in ps)[len(ps) // 2]; dy = sorted(p[2] for p in ps)[len(ps) // 2]
            dh = sorted(p[3] for p in ps)[len(ps) // 2]
            # part of an earlier correction may still be ramping in (adjacent posts 0.35 m apart are passed ~1 s apart):
            # the estimate will still move by -pending (c units) -> only the remainder is new (2026-10-29 run rc_rcon2_d4:
            # tags 28 and 26 both applied ~-0.13 m -> overshoot to -0.12 m). Heading ramps rotate about the vehicle, so
            # their position effect is already compensated in c.
            dx += self.rc_pending[1]; dy += self.rc_pending[0]; dh -= self.rc_pending[2]
            # robust spread (1.4826 x median absolute deviation): one edge-of-image pose flip must not throw away the pass
            # (2026-10-29 run lk_on_d8: 6 detections of the start tag, sd 0.54 m from one outlier -> the 0.28 m fix was lost)
            sd = max(_mad([p[1] for p in ps]), _mad([p[2] for p in ps]))
            since = ps[-1][0] - (self.rc_last_ok if self.rc_last_ok is not None else self.tag_fix_t)
            gp = min(RC_GATE_MAX, RC_GATE_POS + RC_GATE_GROW * since); gy = min(8.0, RC_GATE_YAW + 0.1 * since)
            ok = len(ps) >= RC_MIN_N and max(abs(dx), abs(dy)) < gp and abs(dh) < gy and sd < RC_MAX_SD
            if ok:
                self.rc_last_ok = ps[-1][0]
                # an accepted tag pass is an absolute reference: re-anchor the ToF cap at the corrected c
                self.cN0, self.cE0, self.cap_t0 = self.cN, self.cE, ps[-1][0]   # the ramp moves c and c0 together
            ev = {"t": ps[-1][0], "id": tid, "n": len(ps), "dx": dx, "dy": dy, "dh": dh, "sd": sd, "gate_m": gp, "applied": ok}
            self.rc_events.append(ev)
            if ok:
                # est must move by (+dx, +dy, +dh): map x = E, map y = N, so c moves the opposite way; heading via cY
                self.rc_pending[0] += -RC_GAIN_ALONG * dy
                self.rc_pending[1] += -RC_GAIN_LAT * dx
                self.rc_pending[2] += RC_GAIN_YAW * dh
            log("[tags] re-check tag %d: %d det, innovation x %+.3f y %+.3f m heading %+.2f deg (sd %.3f, gate %.2f m) -> %s" % (
                tid, len(ps), dx, dy, dh, sd, gp, "applied" if ok else "REJECTED"))
        # ramp the pending correction into c (N = along the aisle for these legs, but kept general in map axes)
        dt = 0.0 if self.rc_t is None else max(0.0, min(0.2, now - self.rc_t)); self.rc_t = now
        if dt <= 0 or not self.hist:
            return
        mv = []
        for k, rate in ((0, RC_RATE_POS), (1, RC_RATE_POS)):
            st = max(-rate * dt, min(rate * dt, self.rc_pending[k])); self.rc_pending[k] -= st; mv.append(st)
        self.cN += mv[0]; self.cE += mv[1]; self.cN0 += mv[0]; self.cE0 += mv[1]
        sy = max(-RC_RATE_YAW * dt, min(RC_RATE_YAW * dt, self.rc_pending[2]))
        if sy and self.rotate:
            # turn the frame about the vehicle's CURRENT position (not the VIO origin): compensate c for the lever arm
            _, n, e, _y = self.hist[-1]
            on, oe = self.origin
            b = rot_ne(n - on, e - oe, -self.cY); a = rot_ne(n - on, e - oe, -(self.cY + sy))
            self.cN += a[0] - b[0]; self.cE += a[1] - b[1]; self.cN0 += a[0] - b[0]; self.cE0 += a[1] - b[1]
            self.cY += sy; self.cY0 += sy; self.rc_pending[2] -= sy

    def _rx(self):
        while True:
            data = self.sock.recv(2048)
            try:
                m = json.loads(data.decode())
            except ValueError:
                continue
            if not isinstance(m, dict) or not valid_tof_msg(m):
                self.n_bad_tof += 1
                continue
            now_l = self.clock() if self.clock else None
            if (now_l is not None and m["t"] > now_l + 0.5) or (self.tof_t_last is not None and m["t"] <= self.tof_t_last):
                self.n_bad_tof += 1          # stamped in the future, older than or AS OLD AS the previous frame (a duplicate,
                continue                     # 2026-11-01 codex tof-time-alignment): not a new measurement
                continue
            with self.lock:
                self.tof_rx_t = m["t"]; self.tof_t_last = m["t"]; self.tof_rx_local = now_l
                self.meas = m
                if m.get("d") is not None:   # --wk-loc: applied at the frame's own time, once the pose history covers it
                    self.tof_q.append(m); self.tof_q = self.tof_q[-50:]
                if m.get("valid") is not None and m.get("t") is not None:
                    self.tof_valid.append((m["t"], m["valid"])); self.tof_valid = self.tof_valid[-200:]
                if self.tof_rec and m.get("d") is not None:
                    self.tof_hist.append(m); self.tof_hist = self.tof_hist[-3000:]

    def tof_blind(self, now, airborne):
        """ToF unusable at the current (sim) clock `now`: no frame since take-off for TOF_NOFRAME_S, the last frame older than
        TOF_STALE_S, or every frame below TOF_MIN_VALID for a real TOF_BLIND_S (2026-10-31 codex tof-watchdog: the old check
        timed everything with the last frame's stamp, so a stopped stream never aged). Sets tof_blind_since / tof_blind_why."""
        if airborne and self.airborne_since is None:
            self.airborne_since = now
        why = None
        if self.tof_rx_t is None:
            if airborne and now - self.airborne_since >= TOF_NOFRAME_S:
                why = "no ToF frame for %.1f s after take-off" % (now - self.airborne_since)
        elif now - (self.tof_rx_local if self.tof_rx_local is not None else self.tof_rx_t) > TOF_STALE_S:
            why = "last ToF frame received %.1f s ago" % (now - (self.tof_rx_local if self.tof_rx_local is not None else self.tof_rx_t))
        else:
            last_v = self.tof_valid[-1][1] if self.tof_valid else 1.0
            if last_v >= TOF_MIN_VALID:
                self.low_since = None
            elif self.low_since is None:
                self.low_since = self.tof_valid[-1][0]
            if self.low_since is not None and now - self.low_since >= TOF_BLIND_S:
                why = "valid pixels %.2f < %.2f for %.1f s" % (last_v, TOF_MIN_VALID, now - self.low_since)
        if why is None:
            self.tof_blind_since = None
        elif self.tof_blind_since is None:
            self.tof_blind_since = now
        self.tof_blind_why = why
        return why is not None

    def safety_step(self, now, airborne):
        """every 50 ms tick of the correction task, whether feedback_state() has a value or not (2026-10-31 codex reply-2:
        feedback-pose-freshness, tof-watchdog, sim clock). Sets self.lost; wk_task lands on it in every mode."""
        if self.rx_start is None:
            self.rx_start = now
        if airborne:
            self.airborne = True
        age = self.feed_age(now)
        if self.lost is None:
            old = [k for k in ("pos", "att") if age[k] >= POSE_STALE_S]
            if old:
                self.lost = "%s feedback: no new message for %s s (stream stopped)" % (
                    "/".join({"pos": "position", "att": "attitude"}[k] for k in old), "/".join("%.1f" % age[k] for k in old))
        ca = clock_age()
        if self.lost is None and ca is not None and ca >= CLOCK_LOST_S:
            self.lost = "sim clock: no datagram for %.1f s wall (loop time only extrapolated)" % ca
        if self.loc is not None:                 # the ToF watchdog belongs to the --wk-loc pipeline (ToF updates + clearance)
            self.blind_now = self.tof_blind(now, self.airborne)
            if (self.lost is None and self.blind_now and self.airborne and self.tof_blind_since is not None
                    and now - self.tof_blind_since >= TOF_BLIND_LAND_S):
                # the ToF is also the clearance sensor: flying on without it is not safe on the real vehicle. Independent of
                # `ready` (2026-10-31 codex tof-watchdog reply-2: a ToF dead from the start was never landed on)
                self.lost = "ToF blind for %.0f s (%s)" % (now - self.tof_blind_since, self.tof_blind_why)

    def loc_step(self, ctl, fb):
        """--wk-loc (2026-10-30): one map-based estimator (loc2d.Loc2D) replaces the scan fix, the wall-keep channels and the
        re-check. Called at 20 Hz. Before the scan's last stop nothing is applied (as --wk-tags); after it the applied frame
        follows the estimate at <= LOC_RATE_POS / LOC_RATE_YAW."""
        L = self.loc
        if not self.hist or self.origin is None:
            return
        t, n, e, yaw = self.hist[-1]        # pose time (hist is already shifted by POSE_LAG / YAW_LAG): the state lives there
        vio = (n, e, math.degrees(yaw))
        if L.x is None:
            L.init(vio, self.spawn, ctl.current_target_yaw)
            self.loc_t = t
            return
        dt = max(0.0, t - self.loc_t); self.loc_t = t
        L.predict(vio, dt)
        with self.lock:
            frames = [fr for fr in self.tag_frames if self.rc_seen < fr["t"] <= t]    # wait until the PX4 pose of the image time is in
            m, self.meas = self.meas, None
        for fr in frames:
            self.rc_seen = max(self.rc_seen, fr["t"])
            pr = self.pose_at(fr["t"])
            if pr is None:
                continue
            vt = (pr[0][0], pr[0][1], math.degrees(pr[0][2]))
            for d in fr["dets"]:
                L.update_tag(d, fr["t"], vt, L.vio_prev, pr[1])
        tof = None
        if not self.ready and LOC_EARLY and L.n_tag >= LOC_EARLY_TAGS:
            sx_, sy_, syaw_ = L.sd()
            if max(sx_, sy_) < LOC_EARLY_SD_POS and syaw_ < LOC_EARLY_SD_YAW:
                # 2026-10-31 rotor-clearance check: until the scan's last stop the vehicle holds its drifting VIO position
                # (0.6-0.8 cm/s injected) and swept a rotor 1.7-2.2 cm into the CW face while turning at the start cell.
                # The estimate is good after a few tags, so the correction starts now (still rate-limited), not at the scan end.
                self.ready = True
                print("[loc] apply early (in the scan): map x %.3f y %.3f, cY %+.2f deg, sd %.3f/%.3f m %.2f deg, tag updates %d"
                      % (L.x[0], L.x[1], L.x[2], sx_, sy_, syaw_, L.n_tag), flush=True)
        airborne = fb["rel_u"] >= MIN_ALT
        blind = self.blind_now                 # safety_step() ran the watchdog this tick
        if blind and not self.tof_blind_logged:
            self.tof_blind_logged = True
            print("[loc] ToF blind (%s): ToF updates off" % self.tof_blind_why, flush=True)
        elif not blind and self.tof_blind_logged:
            self.tof_blind_logged = False
            print("[loc] ToF back: ToF updates on", flush=True)
        # ToF frames at their own time (2026-11-01 codex tof-time-alignment): a frame waits in tof_q until the PX4 pose history
        # (shifted by POSE_LAG) reaches its stamp, then the residual is built with the pose AT the frame's time (pose_at) and
        # the state's motion since. A frame older than TOF_Q_MAX_AGE behind the pose time, or one the history no longer
        # covers, is dropped and counted (n_tof_old / n_tof_nopose); duplicates / older stamps are refused in _rx.
        with self.lock:
            due = [q for q in self.tof_q if q["t"] <= t]; self.tof_q = [q for q in self.tof_q if q["t"] > t]
        for q in due:
            if t - q["t"] > TOF_Q_MAX_AGE:
                self.n_tof_old += 1
                continue
            if fb["rel_u"] < MIN_ALT or blind:
                continue
            pr = self.pose_at(q["t"])
            if pr is None:
                self.n_tof_nopose += 1
                continue
            vt = (pr[0][0], pr[0][1], math.degrees(pr[0][2]))
            tof = L.update_tof(q, vt, L.vio_prev, pr[1]) or tof
        if self.lost is None and getattr(L, "bad", None):
            self.lost = "estimator state not finite (%s)" % L.bad
        if self.ready and self.lost is None:
            # 2026-10-31 survey sv2 (racks missing in the sim, ToF blind): every tag was rejected from 77 s on and the estimate
            # drifted 2.7 m from the truth while the flight went on. Lost = the map no longer explains what the camera sees.
            sx, sy, _ = L.sd()
            rec = L.tag_recent; rej = (len(rec) - sum(rec)) / float(len(rec)) if rec else 0.0
            if not L.healthy():
                self.lost = "loc lost: estimator state / covariance not finite"
            elif L.tag_rej_streak >= LOC_LOST_TAG_REJ or not (sx <= LOC_LOST_SD and sy <= LOC_LOST_SD):
                self.lost = "loc lost: %d tag detections in a row rejected, sd %.3f/%.3f m" % (L.tag_rej_streak, sx, sy)
            elif len(rec) >= LOC_LOST_REJ_N and rej > LOC_LOST_REJ_FRAC:
                # an occasional tag that still passes the gate resets the streak (2026-10-31 sv2: guard 14 min late); the ratio does not
                self.lost = "loc lost: %d of the last %d tag detections rejected" % (len(rec) - sum(rec), len(rec))
        if self.ready:
            s = self.loc_applied
            if s is None:
                # start from the map pose the CURRENT correction implies (2026-10-31 codex correction-first-step: the first
                # apply jumped straight to the estimate, past the rate limits); the slew below then moves it to the estimate
                on, oe = self.origin
                dn0, de0 = loc2d.rot_ne(vio[0] - on, vio[1] - oe, -self.cY)
                s = self.loc_applied = [self.spawn[0] + de0 - self.cE, self.spawn[1] + dn0 - self.cN, self.cY, vio]
            else:
                dx, dy = L.disp(s[3], vio, s[2]); s[0] += dx; s[1] += dy; s[3] = vio
                for i, r in ((0, LOC_RATE_POS * dt), (1, LOC_RATE_POS * dt)):
                    s[i] += max(-r, min(r, L.x[i] - s[i]))
                s[2] += max(-LOC_RATE_YAW * dt, min(LOC_RATE_YAW * dt, wrap180(L.x[2] - s[2])))
            self.cY, self.cN, self.cE = L.to_c(s[0], s[1], s[2], vio, self.origin, self.spawn)
        # wk.csv in the same columns as step() (tagfix_eval reads it), with the correction just applied and the pose time
        fb = ctl.feedback_state() or fb
        stage = (ctl.active_stage or {}).get("name", "")
        wx = self.spawn[0] + fb["rel_e"]; wy = self.spawn[1] + fb["rel_n"]
        state = ("loc" if self.enabled else "loc_observe") + ("" if self.ready else "_wait")
        self.w.writerow(["%.3f" % time.time(), "%.3f" % t, int(self.enabled), state, stage, "%.2f" % (ctl.current_target_yaw % 360.0), "%.4f" % wx, "%.4f" % wy,
                         (tof or {}).get("face", ""), "" if not m or m.get("d") is None else "%.4f" % m["d"],
                         "" if not m or m.get("d") is None else "%.3f" % m["alpha"], "", "", "%.2f" % fb["yaw_deg"], "",
                         "%.4f" % self.cN, "%.4f" % self.cE, "%.3f" % self.cY,
                         "" if not m or m.get("d") is None else "%.3f" % m["conf"], "" if not m or m.get("d") is None else "%.3f" % m["span"]])
        if t - self.loc_log_t >= 0.1:
            self.loc_log_t = t
            sx, sy, syaw = L.sd()
            rec = {"t": round(t, 3), "x": round(L.x[0], 4), "y": round(L.x[1], 4), "comp": round((vio[2] - L.x[2]) % 360.0, 2),
                   "cY": round(L.x[2], 3), "sd": [round(sx, 4), round(sy, 4), round(syaw, 3)], "ready": self.ready, "stage": stage,
                   "applied": None if self.loc_applied is None else [round(v, 4) for v in self.loc_applied[:3]],
                   "tag": L.last_tag, "tof": L.last_tof, "n": [L.n_tag, L.n_tof_d, L.n_tof_a, L.n_rej]}
            self.loc_f.write(json.dumps(rec) + "\n"); self.loc_f.flush()

    def map_face(self, mx, my, heading):
        """mx, my: MAP coordinates (v3)"""
        room = "R1" if my < 11 else "R2"
        ry0, ry1 = G.ROOMS[room]["racks_y"]
        # rack span +-MAP_MARGIN: the wide ToF still sees enough of the face at the very end of the rack (the sensor's own
        # conf/span checks reject frames without a plane). 2026-10-29 run F: a -0.25 m margin skipped the 9 s reach+climb at
        # row 1 and the drift piled up to 5.7 cm.
        if not (ry0 - MAP_MARGIN <= my <= ry1 + MAP_MARGIN):
            return None
        for col, (x0, x1) in G.AISLES.items():
            if x0 <= mx <= x1:
                break
        else:
            return None
        fc = getattr(self, "face_corr", {})

        def line(name, xf):
            """face line x_face(y) = x0 + k (y - y_ref): the design x, or the surveyed one = design x + offset + tan(turn)
            (y - y_ref); its facing direction turns by the same angle"""
            c = fc.get(name)
            if not c:
                return xf, 0.0, 0.0, 0.0
            return xf + c["offset"], math.tan(math.radians(c["turn_deg"])), c.get("y_ref", 4.75), c["turn_deg"]
        # 2026-11-01 codex survey-face-observation: the distance is the PERPENDICULAR one to the (turned) face line,
        # sgn (x - x_face(y)) / sqrt(1 + k^2) - what the sensor reports and what survey_map's batch residual uses - and the
        # line itself is returned (5th item: x0, k, y_ref) so loc2d predicts with the same geometry and its y derivative
        if heading == 270.0 and col != "A":
            x0, k, yr, turn = line(col + "W", x0)
            return col + "W", (mx - (x0 + k * (my - yr))) / math.sqrt(1 + k * k), (0.0, -1.0), 270.0 + turn, (x0, k, yr)
        if heading == 90.0 and col != "E":
            x1_, k, yr, turn = line(col + "E", x1)
            return col + "E", ((x1_ + k * (my - yr)) - mx) / math.sqrt(1 + k * k), (0.0, 1.0), 90.0 + turn, (x1_, k, yr)
        return None

    def step(self, ctl, fb):
        """called at 20 Hz from the asyncio task with the CORRECTED feedback (fb already minus c)."""
        with self.lock:
            m, self.meas = self.meas, None
        if m is None:
            return
        t = m.get("t")
        dt = 0.0 if self.last_t is None else max(0.0, min(0.5, t - self.last_t))
        self.last_t = t
        stage = (ctl.active_stage or {}).get("name", "")
        tgt = ctl.current_target_yaw % 360.0
        heading = 270.0 if abs(wrap180(tgt - 270.0)) < 0.5 else 90.0 if abs(wrap180(tgt - 90.0)) < 0.5 else None
        if self.tags and not self.ready:
            heading = None                 # --wk-tags: nothing is corrected until the tag scan has fixed the map frame
        wx = self.spawn[0] + fb["rel_e"]; wy = self.spawn[1] + fb["rel_n"]
        row = {"face": "", "d_pred": "", "nu": "", "yaw_true_est": ""}
        state = "hold"
        mf = self.map_face(wx, wy, heading) if heading is not None else None
        if m.get("d") is None:
            state = "no_plane"
        elif heading is None:
            state = "heading_not_survey"
        elif fb["rel_u"] < MIN_ALT:
            state = "low_alt"
        elif mf is None:
            state = "no_face_in_map"
        else:
            face, d_pred, h, face_yaw = mf[:4]
            nu = m["d"] - d_pred
            yaw_true = (face_yaw - m["alpha"]) % 360.0
            row.update(face=face, d_pred="%.4f" % d_pred, nu="%.4f" % nu, yaw_true_est="%.2f" % yaw_true)
            nu_y = wrap180(yaw_true - fb["yaw_deg"])          # fb yaw is already corrected (raw - cY)
            if self.ctrl is not None and self.enabled:
                # v3 (2026-10-29): lateral channel from a wk3 controller. x_est = RAW PX4 lateral position relative to the
                # aisle centre line, + = away from the face; u (+ = away) -> c = -u along h (h points to the face)
                ok = m["conf"] >= MIN_CONF and m["span"] >= 0.3 and abs(m["alpha"]) <= 12.0
                col = face[0]; cx, cy = 0.5 * sum(G.AISLES[col]), 0.0
                x_est = -((wx + self.cE - cx) * h[1] + (wy + self.cN - cy) * h[0])
                u = self.ctrl.update(t, m["d"] if ok else None, x_est)
                self.cN = -u * h[0]; self.cE = -u * h[1]
            if m["conf"] < MIN_CONF or m["span"] < 0.3:
                state = "low_conf"
            elif (self.ctrl is None and abs(nu) > MAX_NU) or abs(m["alpha"]) > 12.0:
                state = "gated"
            else:
                state = "update" if self.enabled else "observe"
                if self.enabled and dt > 0 and self.ctrl is None:
                    k = min(1.0, dt / TAU_POS)
                    step = max(-RATE_POS * dt, min(RATE_POS * dt, k * nu))
                    self.cN += step * h[0]; self.cE += step * h[1]
                    along = (self.cN - self.cN0) * h[0] + (self.cE - self.cE0) * h[1]     # around the last tag reference (v3)
                    cap = CAP_POS + CAP_GROW * max(0.0, t - self.cap_t0) if self.cap_t0 is not None else CAP_POS
                    if abs(along) > cap:
                        ex = along - math.copysign(cap, along)
                        self.cN -= ex * h[0]; self.cE -= ex * h[1]
                if self.enabled and dt > 0:
                    ky = min(1.0, dt / TAU_YAW)
                    ry = RATE_YAW * dt
                    if self.rotate and self.origin is not None:
                        # with --wk-rotate a cY change moves the setpoint sideways by (distance from the VIO origin) x dcY:
                        # cap that lever-arm speed (2026-10-29: angle jitter at a rack end, 8 m out, shook the vehicle +-10 cm)
                        r = math.hypot(fb["cmd_n"] + self.cN - self.origin[0], fb["cmd_e"] + self.cE - self.origin[1])
                        ry = min(ry, math.degrees(ROT_LEVER_RATE * dt / max(r, 0.5)))
                    stepy = max(-ry, min(ry, ky * nu_y))
                    self.cY = max(self.cY0 - CAP_YAW, min(self.cY0 + CAP_YAW, self.cY - stepy))   # cap around the tag fix (v3)
                    self.n_upd += 1
            if m["d"] - ROTOR_FWD < GUARD_CLEAR:
                state += "+GUARD"
            self.last = {"t": t, "state": state, "nu": nu, "nuy": nu_y}
        if mf is None or m.get("d") is None:
            self.last = {"t": t, "state": state, "nu": None, "nuy": None}
        if self.aligning:
            state = "align:" + state
        self.w.writerow(["%.3f" % time.time(), "%.3f" % t, int(self.enabled), state, stage, "%.2f" % tgt, "%.4f" % wx, "%.4f" % wy,
                         row["face"], "" if m.get("d") is None else "%.4f" % m["d"], "" if m.get("d") is None else "%.3f" % m["alpha"],
                         row["d_pred"], row["nu"], "%.2f" % fb["yaw_deg"], row["yaw_true_est"],
                         "%.4f" % self.cN, "%.4f" % self.cE, "%.3f" % self.cY,
                         "" if m.get("d") is None else "%.3f" % m["conf"], "" if m.get("d") is None else "%.3f" % m["span"]])


def _mad(v):
    m = sorted(v)[len(v) // 2]
    return 1.4826 * sorted(abs(x - m) for x in v)[len(v) // 2]


def _sd(v):
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


def install(fs, wk):
    """monkeypatch the flight script's controller class (the file itself is untouched)"""
    import asyncio
    # the safety layer (landing, stop, yaw wait, landing deadline, safe log, feedback stamps) is shared with the real-vehicle
    # launcher real_flight_safe.py: safety_patch.py (2026-11-01). Installed first; this layer wraps around it.
    safe_log = SP.install(fs, wk)
    from mavsdk.offboard import PositionNedYaw
    C = fs.DroneController
    orig_fb = C.feedback_state               # safety_patch's (stale yaw = NaN)
    raw_fb = C._sp_orig["feedback_state"]    # the flight script's own: raw PX4 pose for the estimator

    def rot(n, e, deg):
        """rotate the NED vector (n, e) by yaw deg (clockwise positive, like NED heading)"""
        a = math.radians(deg); c_, s_ = math.cos(a), math.sin(a)
        return n * c_ - e * s_, n * s_ + e * c_

    def applied():
        """the correction the vehicle actually flies: wk's (cN, cE, cY) with --wallkeep on, none with --wallkeep off.
        2026-11-01 codex loc-observe-mode: "off" is observe only for EVERY channel (wall-keep, tag fix, --wk-loc): the
        estimators run and log what they would apply, the setpoints / feedback stay uncorrected (before, --wk-loc and the
        tag fix moved the vehicle while the log said "OFF (observe only)")"""
        return (wk.cN, wk.cE, wk.cY) if wk.enabled else (0.0, 0.0, 0.0)

    def feedback_state(self):
        fb = orig_fb(self)
        if fb is None:
            return None
        fb = dict(fb)                        # (safety_patch already gave a stale yaw as NaN)
        cN, cE, cY = applied()
        if wk.rotate and wk.origin is not None:
            # --wk-rotate (2026-10-29 sweep: at 4 deg the frame rotation's sideways offset grew past CAP_POS along the leg):
            # undo the estimated frame rotation cY about the VIO origin, then the translation c
            on, oe = wk.origin
            dn, de = rot(fb["cmd_n"] - on, fb["cmd_e"] - oe, -cY)
            n_, e_ = on + dn, oe + de
            fb["rel_n"] += n_ - fb["cmd_n"]; fb["rel_e"] += e_ - fb["cmd_e"]
            fb["cmd_n"], fb["cmd_e"] = n_, e_
        fb["cmd_n"] -= cN; fb["cmd_e"] -= cE
        fb["rel_n"] -= cN; fb["rel_e"] -= cE
        fb["yaw_deg"] = wrap180(fb["yaw_deg"] - cY)
        return fb

    async def setpoint_streamer(self):
        period = 1.0 / fs.SETPOINT_HZ
        while self.streaming:
            t = self.target
            cN, cE, cY = applied()
            n_, e_ = t.north_m + cN, t.east_m + cE
            if wk.rotate and wk.origin is not None:
                on, oe = wk.origin
                dn, de = rot(n_ - on, e_ - oe, cY)
                n_, e_ = on + dn, oe + de
            try:
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(n_, e_, t.down_m, t.yaw_deg + cY))
                self._stream_fail_count = 0
            except Exception as e:
                self._stream_fail_count += 1
                fs.log("       [streamer] send failed ({}/{}): {}".format(self._stream_fail_count, fs.MAX_STREAM_FAIL, e))
                if self._stream_fail_count >= fs.MAX_STREAM_FAIL:
                    asyncio.ensure_future(self.trigger_emergency_land("setpoint stream failed {} times".format(fs.MAX_STREAM_FAIL)))
                    break
            await asyncio.sleep(period)

    async def wk_task_supervised(self):
        """2026-10-31 codex localization-task-supervision: if the correction/safety task dies, nothing would correct or watch
        any more while the setpoint streamer flies on -> treat it as a failure and land"""
        try:
            await wk_task(self)
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001
            # 2026-10-31 codex reply-2: the failure state is set and the landing requested whatever the logging does
            wk.lost = wk.lost or "correction task crashed: %r" % e; wk.lost_done = True
            try:
                import traceback
                fs.log("[wk] correction/safety task crashed: %r -> land\n%s" % (e, traceback.format_exc()))
            except Exception:  # noqa: BLE001
                pass
            await self.trigger_emergency_land("correction task crashed: %r" % e)

    async def wk_task(self):
        last = 0.0
        while self.streaming and not self.aborted:
            raw = raw_fb(self); nowp = asyncio.get_event_loop().time()
            wk.poll_voxl(self, nowp)
            if raw is not None:
                wk.airborne = wk.airborne or raw["rel_u"] >= MIN_ALT
            wk.safety_step(nowp, raw is not None and raw["rel_u"] >= MIN_ALT)
            if raw is not None and wk.tags and wk.loc is not None:
                # --wk-loc: hist entries at the POSE time; the yaw is the attitude sample that describes that same time
                now = asyncio.get_event_loop().time()
                wk.raw_yaw.append((now, math.radians(raw["yaw_deg"]))); wk.raw_yaw = wk.raw_yaw[-200:]
                yaw = wk.yaw_at(now - POSE_LAG + YAW_LAG)
                if yaw is not None:
                    wk.hist.append((now - POSE_LAG, raw["cmd_n"], raw["cmd_e"], yaw))
                    wk.hist = wk.hist[-1200:]
                    # raw VIO/PX4 pose log for the offline survey map (survey_map.py): pose time, N E D (m), yaw (deg)
                    if wk.pose_f is None:
                        wk.pose_f = open(os.path.join(os.path.dirname(wk.f.name), "pose.csv"), "w", 1)
                        wk.pose_f.write("t,n,e,d,yaw_deg,u\n")
                    wk.pose_f.write("%.3f,%.4f,%.4f,%.4f,%.3f,%.4f\n" % (now - POSE_LAG, raw["cmd_n"], raw["cmd_e"], raw["cmd_d"], math.degrees(yaw), raw["rel_u"]))
            elif raw is not None and wk.tags:
                wk.hist.append((asyncio.get_event_loop().time(), raw["cmd_n"], raw["cmd_e"], math.radians(raw["yaw_deg"])))
                wk.hist = wk.hist[-1200:]
            fb = self.feedback_state()
            if fb is not None and wk.loc is not None:
                wk.loc_step(self, fb)
            if wk.lost and not wk.lost_done:          # every mode, also while feedback_state() is None
                wk.lost_done = True
                try:
                    fs.log("[loc] %s -> land" % wk.lost)
                except Exception:  # noqa: BLE001
                    pass
                await self.trigger_emergency_land(wk.lost)
                break
            if fb is not None and wk.loc is None:
                wk.step(self, fb)
                if wk.recheck and wk.tags and wk.ready and wk.tag_fix:
                    wk.recheck_step(asyncio.get_event_loop().time(), fs.log)
                now = time.time()
                if now - last >= 2.0:
                    last = now
                    fs.log("       [wallkeep %s] c=(N%+.3f, E%+.3f, yaw%+.2f) updates %d" % (
                        "ON" if wk.enabled else "observe", wk.cN, wk.cE, wk.cY, wk.n_upd))
            await asyncio.sleep(0.05)

    orig_goto = C.go_to_relative

    async def go_to_relative(self, n_rel, e_rel, alt, name, expected_label, speed_mps=None):
        if wk.enabled and wk.align and not wk.align_done and expected_label == "GOTO":
            wk.align_done = True
            await align_wait(self, name)
        return await orig_goto(self, n_rel, e_rel, alt, name, expected_label, speed_mps)

    async def align_wait(self, name):
        loop = asyncio.get_event_loop()
        t0 = loop.time(); ok_since = None; why = "timeout"
        wk.aligning = True
        fs.log("[align] hold before %s: wait until |nu|<%.0f cm, |yaw|<%.1f deg, on target <%.1f cm for %.1f s (max %.1f s)" % (
            name, ALIGN_NU * 100, ALIGN_NUY, ALIGN_POS * 100, ALIGN_HOLD, ALIGN_TIMEOUT))
        while loop.time() - t0 < ALIGN_TIMEOUT and not self.aborted:
            st, fb = wk.last, self.feedback_state()
            good = (st is not None and st["state"].startswith("update") and st["nu"] is not None and abs(st["nu"]) < ALIGN_NU
                    and abs(st["nuy"]) < ALIGN_NUY and fb is not None
                    and fs.horizontal_norm(fb["cmd_n"] - self.target.north_m, fb["cmd_e"] - self.target.east_m) < ALIGN_POS)
            now = loop.time()
            if good:
                ok_since = ok_since or now
                if now - ok_since >= ALIGN_HOLD:
                    why = "converged"
                    break
            else:
                ok_since = None
            await asyncio.sleep(0.05)
        wk.aligning = False
        wk.align_result = (why, loop.time() - t0)
        fs.log("[align] %s after %.2f s: c=(N%+.3f, E%+.3f, yaw%+.2f)" % (why, loop.time() - t0, wk.cN, wk.cE, wk.cY))

    orig_yaw = C.go_yaw

    async def go_yaw(self, target_yaw, name, label, yaw_speed_deg_s=None, relative_delta=None):
        r = await orig_yaw(self, target_yaw, name, label, yaw_speed_deg_s, relative_delta)
        if wk.loc is not None and not wk.scan_checked and str(name).endswith("_last"):
            wk.scan_checked = True
            L = wk.loc; sx, sy, syaw = L.sd()
            fs.log("[loc] apply from here: map x %.3f y %.3f, cY %+.2f deg, sd %.3f/%.3f m %.2f deg, updates tag %d tof %d/%d rejected %d"
                   % (L.x[0], L.x[1], L.x[2], sx, sy, syaw, L.n_tag, L.n_tof_d, L.n_tof_a, L.n_rej))
            # 2026-10-30 run b1800real (tag detector and ToF dead): applied with 0 updates = the takeoff prior, flew on and hit
            # the rack in the sim. Not localised on the map -> land where it is instead of flying the route.
            # written as "not (... <= limit)": a NaN sd fails it (2026-10-31 codex reply-2), and the whole state must be healthy
            if L.n_tag == 0 or not L.healthy() or L.bad or not (sx <= LOC_MAX_SD_POS and sy <= LOC_MAX_SD_POS and syaw <= LOC_MAX_SD_YAW):
                fs.log("[loc] NOT localised (tag updates %d, sd %.3f/%.3f m %.2f deg; limits %.2f m %.1f deg) -> land"
                       % (L.n_tag, sx, sy, syaw, LOC_MAX_SD_POS, LOC_MAX_SD_YAW))
                await self.trigger_emergency_land("loc not localised")
                return False
            if wk.tof_blind_why:
                fs.log("[loc] scan end: ToF not usable (%s) -> land" % wk.tof_blind_why)
                await self.trigger_emergency_land("ToF not usable at the scan end")
                return False
            fs.log("[loc] scan-end check passed%s" % (" (correction already applied early)" if wk.ready else ""))
            wk.ready = True
            wk.tag_fix = {"dpsi_deg": float(L.x[2]), "n_used": L.n_tag, "n_samples": L.n_tag, "ids": sorted(int(k) for k in L.tag_last),
                          "sd": [sx, sy, syaw], "mode": "loc"}
            json.dump(wk.tag_fix, open(os.path.splitext(wk.f.name)[0] + "_tagfix.json", "w"), indent=1)
        elif wk.tags and not wk.ready and str(name).endswith("_last"):
            if not wk.fix_from_tags(wk.origin, fs.log):
                wk.ready = True                       # no fix: fly on the uncorrected frame (logged), ToF still works
            elif wk.tof_heading and wk.tof_hold:
                wk.tof_pending = True                 # measured at the next stop that faces a rack face
            elif wk.tof_heading:                      # frames recorded during the scan
                wk.tof_rec = False
                tag_fix = wk.tag_fix
                dp, info = wk.tof_dpsi(fs.log)
                if dp is not None:
                    wk.fix_from_tags(wk.origin, fs.log, dpsi_override=dp)   # same tag samples, translation with the ToF heading
                wk.tag_fix["tof_heading"] = info; wk.tag_fix["tag_only"] = {k: tag_fix[k] for k in ("dpsi_deg", "cN", "cE")}
            if wk.tag_fix:
                json.dump(wk.tag_fix, open(os.path.splitext(wk.f.name)[0] + "_tagfix.json", "w"), indent=1)
        elif getattr(wk, "tof_pending", False) and round(target_yaw % 360.0) in (90, 270):
            wk.tof_pending = False
            await tof_hold(self, target_yaw % 360.0)
        return r

    async def tof_hold(self, face_yaw):
        tag_fix = wk.tag_fix
        cell = self.target
        step = TOF_HOLD_STEP if face_yaw == 270.0 else -TOF_HOLD_STEP   # map x = script east; W face is at -x
        self.target = PositionNedYaw(cell.north_m, cell.east_m + step, cell.down_m, cell.yaw_deg)
        t0 = wk.hist[-1][0]
        with wk.lock:
            wk.tof_hist = []; wk.tof_rec = True
        fs.log("[tof-heading] hold %.1f s facing %.0f, %.2f m back from the face" % (TOF_HOLD_S, face_yaw, TOF_HOLD_STEP))
        while wk.hist[-1][0] < t0 + TOF_HOLD_S and not self.aborted:
            await asyncio.sleep(0.05)
        wk.tof_rec = False
        dp, info = wk.tof_dpsi(fs.log, t_min=t0 + TOF_HOLD_SKIP)
        if dp is not None:
            wk.fix_from_tags(wk.origin, fs.log, dpsi_override=dp)   # same tag samples, translation with the ToF heading
        wk.tag_fix["tof_heading"] = info; wk.tag_fix["tag_only"] = {k: tag_fix[k] for k in ("dpsi_deg", "cN", "cE")}
        json.dump(wk.tag_fix, open(os.path.splitext(wk.f.name)[0] + "_tagfix.json", "w"), indent=1)
        self.target = cell
        t1 = wk.hist[-1][0]
        while wk.hist[-1][0] < t1 + 1.5 and not self.aborted:   # back on the cell before the route goes on
            await asyncio.sleep(0.05)

    orig_arm = C.arm_and_start_offboard

    async def arm_and_start_offboard(self):
        r = await orig_arm(self)
        wk.origin = (self.initial_x, self.initial_y)
        wk.clock = asyncio.get_event_loop().time
        wk.task = asyncio.ensure_future(wk_task_supervised(self))
        return r

    C.feedback_state = feedback_state
    C.setpoint_streamer = setpoint_streamer
    C.arm_and_start_offboard = arm_and_start_offboard
    C.go_to_relative = go_to_relative
    C.go_yaw = go_yaw


def main():
    argv = sys.argv[1:]
    mine = {"--wallkeep": "off", "--wk-port": "14652", "--wk-csv": "wk.csv"}
    rest = []
    align = "--wk-align" in argv
    rotate = "--wk-rotate" in argv
    argv = [a for a in argv if a not in ("--wk-align", "--wk-rotate")]
    policy = params = None
    if "--wk-policy" in argv:                       # v3: lateral controller from wk3.controllers
        i = argv.index("--wk-policy"); policy = argv[i + 1]; del argv[i:i + 2]
    if "--wk-params" in argv:
        i = argv.index("--wk-params"); params = argv[i + 1]; del argv[i:i + 2]
    tags = "--wk-tags" in argv; argv = [a for a in argv if a != "--wk-tags"]
    recheck = "--wk-tag-recheck" in argv; argv = [a for a in argv if a != "--wk-tag-recheck"]
    tof_heading = "--wk-tof-heading" in argv; argv = [a for a in argv if a != "--wk-tof-heading"]
    tof_hold = "--wk-tof-hold" in argv; argv = [a for a in argv if a != "--wk-tof-hold"]
    use_loc = "--wk-loc" in argv; argv = [a for a in argv if a != "--wk-loc"]   # map-based estimator (loc2d.py), needs --wk-tags   # ToF frames from a hold at s00, not the scan
    tag_port = 14662
    if "--wk-tag-port" in argv:
        i = argv.index("--wk-tag-port"); tag_port = int(argv[i + 1]); del argv[i:i + 2]
    i = 0
    while i < len(argv):
        if argv[i] in mine and i + 1 < len(argv):
            mine[argv[i]] = argv[i + 1]; i += 2
        else:
            rest.append(argv[i]); i += 1
    enabled = mine["--wallkeep"] == "on"
    orig_run = GM.run_flight

    def run_flight(compiled, extra_args, csv_mode, sim_speed=1.0, sim_clock_port=0):
        sp = compiled["spawn"].get("model_xy") or G.world_to_model(*compiled["spawn"]["world_xy"])[:2]
        wk = WallKeep(enabled, int(mine["--wk-port"]), mine["--wk-csv"], (sp[0], sp[1]))
        if tags:
            # WK_MAP (2026-10-31): fly on a surveyed map (survey_map.py survey_map.json: corrected tags + rack face offsets/turns)
            # instead of the design map; path relative to v3_marker/
            wmap = os.environ.get("WK_MAP")
            wk.enable_tags(tag_port, os.path.join(HERE, "..", wmap) if wmap else os.path.join(HERE, "..", "gz", "tags_map.json"))
            wk.recheck = recheck; wk.tof_heading = tof_heading; wk.tof_hold = tof_hold
            wk.tof_rec = tof_heading and not tof_hold
            if use_loc:
                import loc2d
                wk.loc = loc2d.Loc2D(wk.tag_map, wk.map_face)
                wk.loc_f = open(os.path.join(os.path.dirname(os.path.abspath(mine["--wk-csv"])), "loc.jsonl"), "w")
                wk.rotate = True
                print("[wk_mission] map-based localisation ON (loc2d)")
            print("[wk_mission] tag scan fix ON (udp %d)%s" % (tag_port, ", mid-route re-check ON" if recheck else ""))
        wk.align = align
        wk.rotate = rotate or wk.loc is not None
        if policy:
            sys.path.insert(0, os.path.join(HERE, ".."))
            from wk3 import controllers as WC
            kw = json.load(open(params)) if params else {}
            if policy == "mlp" and "weights" in kw and not os.path.isabs(kw["weights"]):
                kw["weights"] = os.path.join(HERE, "..", kw["weights"])
            wk.ctrl = WC.make(policy, d_ref=0.5 * (G.AISLES["C"][1] - G.AISLES["C"][0]), **kw)
            print("[wk_mission] lateral controller: %s %s" % (policy, kw))
        print("[wk_mission] wall-keep %s%s, sensor udp %s, log %s" % ("ON" if enabled else "OFF (observe only: wall-keep%s estimated and logged, NOT applied)" % (" / tag fix / loc" if use_loc else " / tag fix" if tags else ""), (" + align hold" if align and enabled else "") + (" + frame rotation" if rotate and enabled else ""), mine["--wk-port"], mine["--wk-csv"]))
        sys.path.insert(0, GM.PROJECT_ROOT)
        import path_flight_phase1_v13 as fs
        install(fs, wk)
        rpath = os.path.join(os.path.dirname(os.path.abspath(mine["--wk-csv"])), "mission_result.json")
        err = SP.prepare_result(wk, rpath)
        if err:                                     # nothing flies without a place for the result (codex decisions reply-7 S3)
            fs.log("[result] !!! %s - NOT starting (no flight) !!!" % err); raise SystemExit(SP.EXIT["report"])
        fs.log("[result] run id %s, result file %s" % (wk.run_id, rpath))   # run_status.sh checks the JSON against this id
        try:
            orig_run(compiled, extra_args, csv_mode, sim_speed, sim_clock_port)
        except RuntimeError:
            if wk.final is None:                    # not the STOP supervisor's loop stop
                raise
        finally:
            # the same end as real_flight_safe.py (2026-11-01 codex real-mission-result-contract): the layer's tasks are ended
            # and reported, "[result]" lines + mission_result.json next to wk.csv. The process exit code is v13's; a result file
            # that was not written reaches status.json / the batch through the "[result] !!! RESULT FILE NOT WRITTEN" line
            # (run_status.sh: result_file not_written, exit 16 when nothing else failed)
            res = SP.report(wk, fs.log, rpath)
            wk.f.flush()
            main = wk.flow_task
            if wk.sup_owner and main is not None and not main.done():    # a flight-script task ignored its cancel
                sys.stdout.flush(); os._exit(res["exit"] or 1)
            print("[wk_mission] final correction c=(N%+.4f, E%+.4f, yaw%+.3f), updates %d%s" % (wk.cN, wk.cE, wk.cY, wk.n_upd, "" if wk.enabled else " (observe only: NOT applied)"))
            if wk.rc_events:
                json.dump(wk.rc_events, open(os.path.splitext(wk.f.name)[0] + "_recheck.json", "w"), indent=1)

    GM.run_flight = run_flight
    GM.main(rest)


if __name__ == "__main__":
    main()
