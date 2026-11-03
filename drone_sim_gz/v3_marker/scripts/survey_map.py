#!/usr/bin/env python3
"""Survey map from one flight (v3_marker, 2026-10-31): correct the DESIGN map with what the vehicle measured.
  <cad venv python> survey_map.py <run_dir> [--config config/survey_starling2.json] [--design gz/tags_map.json] [--out <run>/survey]

Inputs (all from the vehicle; no simulator truth is read here):
  pose.csv        raw PX4/VIO local position at the pose time (t, n, e, d, yaw_deg)       wk_mission.py --wk-loc
  tags_raw.jsonl  every tag detection {"t", "dets": [{"id", "t_cam": [right, down, fwd], "R", "margin", "hamming"}]}
  sense.csv       ToF plane fit per frame (d body -> plane, alpha)                         wall_sense.py
  tof_raw/        raw depth frames (WK_TOF_RAW=1)                                          wall_sense.py
  loc.jsonl       the online estimator (start values only), plan.json (start cell)
Method (one batch least squares, Gauss-Newton + Huber, numeric Jacobian):
  unknowns  trajectory = start position + VIO increments turned by a heading offset cY(t) + position drift d(t)
            (cY and d piecewise linear on knots, random-walk priors = VIO drift / PX4 heading wander),
            every SEEN rack face: offset and turn of its line (prior: design +- face_sd, face_angle_sd),
            every SEEN tag: position along the face, height, stand-off (prior: design +- tag_sd_*),
  measured  tag position in the camera (3 axes), ToF distance and angle to the face in front.
  The design map fixes the frame (gauge): the result is the design map moved to where the measurements put it.
Then the ToF depth frames are stacked on the solved trajectory: top view, one front view per seen face, line fits of the
faces and walls from the points (an INTERNAL consistency check - the same ToF frames on the same solved path, not an
independent validation; codex survey-validation-independence 2026-11-01). Outputs in <out>/:
  survey_map.json   corrected map (tags in tags_map.json format + sd, faces, walls)      traj.csv  solved trajectory
  points.npz        stacked points (map frame, 2 cm voxels)                              top.png, front_<face>.png
Everything vehicle- or site-specific is in the config (sensor poses, noise, rates, priors).
2026-11-01 (codex survey-uncertainty / survey-anchor-lines / survey-replay-consistency / survey-validation-independence):
  tag "cov"/"sd" = J COV J^T over [face offset, face slope, stand-off, y, z] incl. cross terms, turned with the anchor turn
  (R C R^T; a shift leaves it); the anchor itself is taken as FIXED (its own uncertainty is not in the numbers, "uncertainty").
  Face lines (solved and points fit) are moved by the anchor as lines (two points, survey_geom.xform_line), slope kept.
  The merged config (after --set), input / code hashes and the anchor transform are saved for survey_web_export.py.
  "observability": data-only (no map priors) vs prior sd per map parameter and the weakest parameter combinations;
  "eligible" false = the data alone do not pin the map down as well as the priors do (not a map for the next flight)."""
import argparse, csv, glob, hashlib, json, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); V3 = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import qvix_grid as G  # noqa: E402
import survey_geom as SG  # noqa: E402
import tag_quality as TQ  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("--config", default=os.path.join(V3, "config", "survey_starling2.json"))
ap.add_argument("--design", default=os.path.join(V3, "gz", "tags_map.json")); ap.add_argument("--out")
ap.add_argument("--no-plots", action="store_true")
ap.add_argument("--set", action="append", default=[], help="config override section.key=value (JSON value), e.g. vio.q_vel=0.0002")
A = ap.parse_args()
CFG = json.load(open(A.config))
for kv in A.set:
    k, val = kv.split("=", 1); sec, key = k.split(".", 1); CFG[sec][key] = json.loads(val)
TC, TF, VC, MP, SV, GR = (CFG[k] for k in ("tag_camera", "tof", "vio", "map_prior", "solver", "grid"))
OUT = A.out or os.path.join(A.run, "survey"); os.makedirs(OUT, exist_ok=True)
DESIGN = json.load(open(A.design))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------ design map (faces, walls)
room = "R1"; RY0, RY1 = G.ROOMS[room]["racks_y"]; YMID = 0.5 * (RY0 + RY1); UNITS = G.RACK_UNITS_Y[room]
FACES = {}
for col, (x0, x1) in G.AISLES.items():
    if col != "A":
        FACES[col + "W"] = {"x": x0, "sgn": 1.0, "look": 270.0, "col": col}     # vehicle looks W at the face on the aisle's west side
    if col != "E":
        FACES[col + "E"] = {"x": x1, "sgn": -1.0, "look": 90.0, "col": col}
WALLS = {"S": ("y", G.ROOMS[room]["y"][0]), "N": ("y", G.ROOMS[room]["y"][1]), "W": ("x", G.WALL), "E": ("x", G.X_MAX - G.WALL)}

# ------------------------------------------------------------------ load the run
P = np.genfromtxt(os.path.join(A.run, "pose.csv"), delimiter=",", names=True)
P = P[np.argsort(P["t"])]; P = P[np.concatenate([[True], np.diff(P["t"]) > 1e-4])]
pt, pn, pe, pd, pyaw = P["t"], P["n"], P["e"], P["d"], np.degrees(np.unwrap(np.radians(P["yaw_deg"])))
plan = json.load(open(os.path.join(A.run, "plan.json")))
SPAWN = np.array(plan["spawn"]["model_xy"], float)
LOC = [json.loads(s) for s in open(os.path.join(A.run, "loc.jsonl")) if s.strip()]
lt = np.array([r["t"] for r in LOC]); lcy = np.degrees(np.unwrap(np.radians([r["cY"] for r in LOC])))
lx = np.array([r["x"] for r in LOC]); ly = np.array([r["y"] for r in LOC])
t0, t1 = pt[0], pt[-1]
HAS_U = "u" in P.dtype.names                        # height above the take-off point (wk_mission pose.csv since 2026-10-31)
pu = P["u"] if HAS_U else pd[0] - pd                 # older logs: relative to the first logged sample (start height unknown)
Z0_SD = 0.02 if HAS_U else 0.3

# yaw rate and height at any time (frames taken while turning fast or near the floor are not used)
_yr = np.gradient(pyaw, pt); _hu = (P["u"] if "u" in P.dtype.names else pd[0] - pd)


def steady(t):
    return abs(np.interp(t, pt, _yr)) <= CFG["filters"]["max_yaw_rate_deg_s"] and np.interp(t, pt, _hu) >= CFG["filters"]["min_height"]


TAGOBS = []                                          # (t, id, right, down, fwd)
last = {}
for s in open(os.path.join(A.run, "tags_raw.jsonl")):
    m = json.loads(s)
    for dt in m["dets"]:
        k = str(dt["id"]); r, dn, f = dt["t_cam"]
        # one quality rule with the online consumers (2026-11-01 codex real-tag-quality-contract): a detection without a margin
        # (real VOXL decoder: null, "unavailable") is rejected unless WK_TAG_QUALITY_UNAVAILABLE=accept; never compared as a number
        if k not in DESIGN["tags"] or not TQ.ok(dt, TC["min_margin"]):
            continue
        if not (0.05 < f < TC["max_range"]) or abs(r) > TC["max_offaxis"] * f or not (t0 < m["t"] < t1) or not steady(m["t"]):
            continue
        if m["t"] - last.get(k, -1e9) < TC["min_gap_s"]:
            continue
        last[k] = m["t"]; TAGOBS.append((m["t"], k, r, dn, f))
SENSE = []
with open(os.path.join(A.run, "sense.csv")) as f:
    nxt = -1e9
    for r in csv.DictReader(f):
        if not r["d_meas"]:
            continue
        t = float(r["t_sim"])
        if t < nxt or not (t0 < t < t1) or not steady(t):
            continue
        if float(r["conf"]) < TF["min_conf"] or float(r["span"]) < TF["min_span"] or abs(float(r["alpha_meas_deg"])) > TF["max_alpha_deg"]:
            continue
        nxt = t + 1.0 / TF["fit_hz"]; SENSE.append((t, float(r["d_meas"]), float(r["alpha_meas_deg"])))
SENSE = np.array(SENSE)
# tray edges as a height reference (2026-10-31: PX4 height was +1.2 / +2.4 cm off at 0.68 / 1.265 m in sv3, so the upper tags came
# out 2.3 cm high). Facing a rack face, the ToF column through the image centre alternates tray fronts (near) and the open gaps
# between trays (far); each near/far switch is a tray top or bottom edge, whose height is in the design (qvix_grid.TRAY_Z, the
# racks do not move up or down). One edge gives z_body + z_rel(edge in the sensor) = design edge; noisy (~2.5 cm) but many.
EDGE_Z = np.array(sorted(z_ for a_ in G.TRAY_Z for z_ in a_))
EDGES = []
_rawdir = os.path.join(A.run, "tof_raw")
if TF.get("edge_hz", 0) > 0 and os.path.isdir(_rawdir):
    _m = json.load(open(os.path.join(_rawdir, "meta.json"))); _W, _H = _m["width"], _m["height"]
    _f = (_W / 2.0) / math.tan(math.radians(TF["hfov_deg"]) / 2.0); _cy = (_H - 1) / 2.0; _nxt = -1e9
    for _fn in sorted(glob.glob(os.path.join(_rawdir, "chunk_*.npz"))):
        _Z = np.load(_fn)
        for _t, _dep in zip(_Z["t"], _Z["depth"]):
            if _t < _nxt or not (t0 < _t < t1) or not steady(_t):
                continue
            _nxt = _t + 1.0 / TF["edge_hz"]
            _d = _dep.astype(np.float32); _d[~np.isfinite(_d)] = np.nan
            with np.errstate(all="ignore"):
                _p = np.nanmedian(_d[:, _W // 2 - 8:_W // 2 + 8], axis=1)
                _n0 = np.nanpercentile(_p, 20)
            if not (0.2 < _n0 < 0.7):
                continue
            _near = np.abs(_p - _n0) < 0.04; _far = np.isnan(_p) | (_p > _n0 + 0.10)
            for _r in range(_H - 1):
                if (_near[_r] and _far[_r + 1]) or (_far[_r] and _near[_r + 1]):
                    EDGES.append((_t, _n0 * (-((_r + 0.5) - _cy) / _f) + TF["pos_body_flu"][2]))
EDGES = np.array(EDGES).reshape(-1, 2)
print("[survey] pose %d (%.0f s), tag obs %d (%d tags), ToF fits %d, tray edges %d" % (len(pt), t1 - t0, len(TAGOBS), len(set(o[1] for o in TAGOBS)), len(SENSE), len(EDGES)))

# ------------------------------------------------------------------ parameters
KN = np.arange(t0, t1 + VC["knot_s"], VC["knot_s"]); K = len(KN)
tags_used = sorted(set(o[1] for o in TAGOBS), key=int)


def rot_ne(n, e, deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return n * c - e * s, n * s + e * c


def base_traj(X0, cyk, dxk, dyk, z0, dzk=None):
    cy = np.interp(pt, KN, cyk)
    dn, de = rot_ne(np.diff(pn), np.diff(pe), -0.5 * (cy[1:] + cy[:-1]))
    x = X0[0] + np.concatenate([[0.0], np.cumsum(de)]) + np.interp(pt, KN, dxk)
    y = X0[1] + np.concatenate([[0.0], np.cumsum(dn)]) + np.interp(pt, KN, dyk)
    return x, y, z0 + pu + (np.interp(pt, KN, dzk) if dzk is not None else 0.0), pyaw - cy


# faces seen: associate the ToF fits with the initial (online) trajectory, refreshed every outer iteration
def associate(x, y, comp):
    fx = np.interp(SENSE[:, 0], pt, x); fy = np.interp(SENSE[:, 0], pt, y); fc = np.interp(SENSE[:, 0], pt, comp)
    out = []
    for X, Y, C in zip(fx, fy, fc):
        name = None
        # inside a rack unit, away from its ends: at the ends and at the 0.3 m gap between units the image also sees what is
        # behind the face line and the plane fit is not the face (2026-10-31 run sv3: +14..+40 cm outliers)
        if any(u0 + TF["unit_margin"] <= Y <= u1 - TF["unit_margin"] for u0, u1 in UNITS):
            for col, (a0, a1) in G.AISLES.items():
                if a0 <= X <= a1:
                    for fn, fc_ in FACES.items():
                        if fc_["col"] == col and abs(wrap(C - fc_["look"])) < 25.0 and (FACE_SET is None or fn in FACE_SET):
                            name = fn
        out.append(name)
    return out


FACE_SET = None
x_init = np.interp(pt, lt, lx); y_init = np.interp(pt, lt, ly)
cy_k0 = np.interp(KN, lt, lcy)
bx, by, _, bcomp = base_traj(np.array([x_init[0], y_init[0]]), cy_k0, np.zeros(K), np.zeros(K), VC["body_height_at_rest"])
faces_used = sorted(set(f for f in associate(x_init, y_init, bcomp) if f) | set(DESIGN["tags"][k]["face"] for k in tags_used))
FACE_SET = set(faces_used)

# parameter vector layout
idx = {}; n = 0


def block(name, size):
    global n
    idx[name] = slice(n, n + size); n += size


block("X0", 2); block("cy", K); block("dx", K - 1); block("dy", K - 1); block("z0", 1); block("dz", K - 1)
for fn in faces_used:
    block("face_" + fn, 2)                      # offset (m), turn (rad, dx/dy)
for k in tags_used:
    block("tag_" + k, 3)                        # stand-off, y, z
x0 = np.zeros(n)
x0[idx["X0"]] = [x_init[0], y_init[0]]; x0[idx["cy"]] = cy_k0
dxk = np.interp(KN, pt, x_init - bx); dyk = np.interp(KN, pt, y_init - by)
x0[idx["dx"]] = (dxk - dxk[0])[1:]; x0[idx["dy"]] = (dyk - dyk[0])[1:]; x0[idx["X0"]] += [dxk[0], dyk[0]]
x0[idx["z0"]] = VC["body_height_at_rest"]
for k in tags_used:
    x0[idx["tag_" + k]] = [MP["tag_standoff"], DESIGN["tags"][k]["y"], DESIGN["tags"][k]["z"]]


def unpack(v):
    dx = np.concatenate([[0.0], v[idx["dx"]]]); dy = np.concatenate([[0.0], v[idx["dy"]]]); dz = np.concatenate([[0.0], v[idx["dz"]]])
    return base_traj(v[idx["X0"]], v[idx["cy"]], dx, dy, v[idx["z0"]][0], dz)


# fix each edge's design edge once, from the PX4 height (a few cm off at most): edges spaced 0.2 / 0.38 m would let a free
# nearest-edge choice slide the height onto the next tray (2026-10-31 first try: tags 14 cm off); detections that are not within
# edge_assoc of a design edge (posts, tags, plants) are dropped
if len(EDGES):
    _z0 = unpack(x0)[2]; _ze = np.interp(EDGES[:, 0], pt, _z0) + EDGES[:, 1]
    _k = np.abs(_ze[:, None] - EDGE_Z[None, :]).argmin(1); _keep = np.abs(_ze - EDGE_Z[_k]) < TF.get("edge_assoc", 0.04)
    EDGES, EDGE_T = EDGES[_keep], EDGE_Z[_k][_keep]
    print("[survey] tray edges kept %d (within %.2f m of a design edge at the PX4 height)" % (len(EDGES), TF.get("edge_assoc", 0.04)))
else:
    EDGE_T = np.zeros(0)


def face_line(v, fn):
    b, phi = v[idx["face_" + fn]]
    return FACES[fn]["x"] + b, phi                  # x_f(y) = x0 + b + phi * (y - YMID)


def tag_xyz(v, k):
    s, y, z = v[idx["tag_" + k]]; fn = DESIGN["tags"][k]["face"]; xf, phi = face_line(v, fn)
    return xf + phi * (y - YMID) + FACES[fn]["sgn"] * s, y, z


TO = np.array([(o[0], o[2], o[3], o[4]) for o in TAGOBS]); TK = [o[1] for o in TAGOBS]
if len(TO) and TC.get("yaw_deg", 0.0):          # camera mount turned by yaw_deg (+ = left) about the vertical: undo it
    _a = math.radians(TC["yaw_deg"]); _r, _f = TO[:, 1].copy(), TO[:, 3].copy()
    TO[:, 1] = _r * math.cos(_a) - _f * math.sin(_a); TO[:, 3] = _r * math.sin(_a) + _f * math.cos(_a)
if len(SENSE) and TF.get("d_offset", 0.0):      # calibration: added to every plane distance
    SENSE[:, 1] += TF["d_offset"]
CAM = np.array(TC["pos_body_flu"]); TOFP = np.array(TF["pos_body_flu"])
q_cy = VC["q_yaw_deg"] * math.sqrt(VC["knot_s"]); q_d = VC["q_pos"] * math.sqrt(VC["knot_s"])
q_a = VC.get("q_vel", 0.001) * VC["knot_s"] ** 1.5          # second difference of the drift (velocity random walk)
ASSOC = None


def residuals(v):
    x, y, z, comp = unpack(v)
    r = []
    # tag observations
    if len(TO):
        tx = np.interp(TO[:, 0], pt, x); ty = np.interp(TO[:, 0], pt, y); tz = np.interp(TO[:, 0], pt, z)
        th = np.radians(90.0 - np.interp(TO[:, 0], pt, comp)); c, s = np.cos(th), np.sin(th)
        cx = tx + CAM[0] * c - CAM[1] * s; cy = ty + CAM[0] * s + CAM[1] * c; cz = tz + CAM[2]
        T = np.array([tag_xyz(v, k) for k in TK])
        vx, vy, vz = T[:, 0] - cx, T[:, 1] - cy, T[:, 2] - cz
        fwd = vx * c + vy * s; right = vx * s - vy * c; down = -vz
        rng = np.sqrt(TO[:, 1] ** 2 + TO[:, 3] ** 2); sd = TC["sd0"] + TC["sd_k"] * rng
        r += [(TO[:, 1] - right) / sd, (TO[:, 2] - down) / (sd * TC["down_mult"]), (TO[:, 3] - fwd) / sd]
    # ToF plane fits
    if ASSOC is not None:
        m = np.array([a is not None for a in ASSOC])
        sx = np.interp(SENSE[m, 0], pt, x); sy = np.interp(SENSE[m, 0], pt, y); sc = np.interp(SENSE[m, 0], pt, comp)
        dd, aa = [], []
        for X, Y, C, fn, dm, am in zip(sx, sy, sc, [a for a in ASSOC if a is not None], SENSE[m, 1], SENSE[m, 2]):
            xf, phi = face_line(v, fn); sg = FACES[fn]["sgn"]; nr = math.sqrt(1 + phi * phi)
            dd.append((dm - sg * (X - (xf + phi * (Y - YMID))) / nr) / (TF["d_sd"] * TF["frame_corr"]))
            nx, ny = sg / nr, -sg * phi / nr                                 # face normal into the aisle
            fcomp = math.degrees(math.atan2(-nx, -ny))
            aa.append(wrap(wrap(fcomp - C) - am) / (TF["alpha_sd_deg"] * TF["frame_corr"]) if dm >= TF["min_d_for_alpha"] else 0.0)
        r += [np.array(dd), np.array(aa)]
    # tray edges (height): nearest design edge to the current estimate
    if len(EDGES):
        ze = np.interp(EDGES[:, 0], pt, z) + EDGES[:, 1]
        r.append((ze - EDGE_T) / TF["edge_sd"])
    # priors
    r.append((v[idx["X0"]] - SPAWN) / VC["start_sd"])
    r.append(np.array([wrap(v[idx["cy"]][0] - cy_k0[0]) / VC["start_yaw_sd_deg"]]))
    r.append(np.diff(v[idx["cy"]]) / q_cy)
    for key in ("dx", "dy"):
        dk = np.concatenate([[0.0], v[idx[key]]])
        if VC.get("drift_model", "velocity") == "velocity":
            r.append(np.diff(dk, 2) / q_a); r.append(np.array([(dk[1] - dk[0]) / VC["knot_s"] / VC["vel0_sd"]]))
        else:
            r.append(np.diff(dk) / q_d)
    r.append((v[idx["z0"]] - VC["body_height_at_rest"]) / Z0_SD)
    dzk = np.concatenate([[0.0], v[idx["dz"]]])                       # height drift: slowly changing rate, starts at 0 on the ground
    r.append(np.diff(dzk, 2) / (VC.get("q_vel_z", 0.0005) * VC["knot_s"] ** 1.5)); r.append(np.array([(dzk[1] - dzk[0]) / VC["knot_s"] / 0.01]))
    for fn in faces_used:
        b, phi = v[idx["face_" + fn]]
        r.append(np.array([b / MP["face_sd"], math.degrees(phi) / MP["face_angle_sd_deg"]]))
    for k in tags_used:
        s, ty_, tz_ = v[idx["tag_" + k]]; d = DESIGN["tags"][k]
        r.append(np.array([(s - MP["tag_standoff"]) / MP["tag_standoff_sd"], (ty_ - d["y"]) / MP["tag_sd_along"], (tz_ - d["z"]) / MP["tag_sd_z"]]))
    return np.concatenate([np.atleast_1d(a) for a in r])


def jac(v, r0):
    J = np.zeros((len(r0), len(v)))
    for j in range(len(v)):
        e = 1e-5 if not (idx["cy"].start <= j < idx["cy"].stop) else 1e-4
        vp = v.copy(); vp[j] += e; J[:, j] = (residuals(vp) - r0) / e
    return J


def weights(r, it):
    """Huber inside the gate; measurements beyond gate sd get weight 0 from the 2nd iteration (they are not what the map
    says they are: another surface, a person, a mis-read tag). Priors (the tail of r) are never gated."""
    nm = 3 * len(TO) + 2 * sum(a is not None for a in ASSOC) + len(EDGES)
    if it < SV["ls_iterations"]:           # plain least squares first (only gross outliers out): Huber from a far start
        w = np.ones(len(r))                # down-weights exactly the measurements that would pull the map into place
        if it > 0:
            w[:nm][np.abs(r[:nm]) > SV["gate_coarse"]] = 0.0
        return w
    w = np.where(np.abs(r) <= SV["huber"], 1.0, SV["huber"] / np.maximum(np.abs(r), 1e-9))
    w[:nm][np.abs(r[:nm]) > SV["gate"]] = 0.0
    return w


v = x0.copy()
for it in range(SV["iterations"]):
    x, y, z, comp = unpack(v)
    ASSOC = associate(x, y, comp)
    r0 = residuals(v)
    w = weights(r0, it)
    J = jac(v, r0); Wj = J * w[:, None]
    H = J.T @ Wj; g = J.T @ (w * r0)
    step = np.linalg.solve(H + 1e-9 * np.eye(len(v)), -g)
    v = v + step
    cost = float(np.sum(w * r0 ** 2))
    print("[survey] it %d cost %.1f |step| %.4f (faces %d, ToF fits used %d, gated out %d)" % (it, cost, float(np.max(np.abs(step))), len(faces_used),
          sum(a is not None for a in ASSOC), int((w == 0).sum())))
    if np.max(np.abs(step)) < 1e-5:
        break
r0 = residuals(v); w = weights(r0, 99)
J = jac(v, r0); COV = np.linalg.inv(J.T @ (J * w[:, None]))
SD = np.sqrt(np.diag(COV))
nT = len(TO); nS = sum(a is not None for a in ASSOC)
for name, sl in (("tag right", slice(0, nT)), ("tag down", slice(nT, 2 * nT)), ("tag fwd", slice(2 * nT, 3 * nT)),
                 ("ToF d", slice(3 * nT, 3 * nT + nS)), ("ToF alpha", slice(3 * nT + nS, 3 * nT + 2 * nS)),
                 ("tray edge", slice(3 * nT + 2 * nS, 3 * nT + 2 * nS + len(EDGES)))):
    rr = r0[sl]
    if len(rr):
        print("[survey] residual %-9s n %4d  mean %+.3f  rms %.3f  (in sd units, measured - predicted)" % (name, len(rr), rr.mean(), np.sqrt(np.mean(rr ** 2))))
o = 3 * nT + 2 * nS + len(EDGES); Kd = K - 2 if VC.get("drift_model", "velocity") == "velocity" else K - 1
for name, size in (("prior X0", 2), ("prior cY0", 1), ("prior cY walk", K - 1), ("prior drift x", Kd + (1 if Kd == K - 2 else 0)),
                   ("prior drift y", Kd + (1 if Kd == K - 2 else 0)), ("prior z0", 1), ("prior drift z", K - 1), ("prior faces", 2 * len(faces_used)), ("prior tags", 3 * len(tags_used))):
    rr = r0[o:o + size]; o += size
    print("[survey] %-14s n %4d  cost %.1f  max |%.2f|" % (name, len(rr), float(np.sum(rr ** 2)), float(np.abs(rr).max()) if len(rr) else 0.0))
x, y, z, comp = unpack(v)

# ------------------------------------------------------------------ corrected map
out = {"run": os.path.basename(os.path.normpath(A.run)), "config": os.path.basename(A.config), "design": os.path.basename(A.design),
       "frame": "rack design frame (QVIX model x, y, z)", "family": DESIGN["family"], "size_m": DESIGN["size_m"], "tags": {}, "faces": {}, "walls": {}}
TAGCOV, FACECOV = {}, {}                    # full precision; rounded only when written (see finish_out)
for k, d in DESIGN["tags"].items():
    if k in tags_used:
        tx_, ty_, tz_ = tag_xyz(v, k); sl = idx["tag_" + k]; fn_ = d["face"]
        ii = list(range(idx["face_" + fn_].start, idx["face_" + fn_].stop)) + list(range(sl.start, sl.stop))   # [b, phi, s, y, z]
        TAGCOV[k] = SG.tag_cov(COV[np.ix_(ii, ii)], ty_, YMID, FACES[fn_]["sgn"], v[idx["face_" + fn_]][1])
        out["tags"][k] = dict(d, x=tx_, y=ty_, z=tz_, observed=True, n_obs=TK.count(k))
    else:
        out["tags"][k] = dict(d, observed=False)
for fn in faces_used:
    b, phi = v[idx["face_" + fn]]; sl = idx["face_" + fn]; FACECOV[fn] = COV[sl, sl].copy()
    out["faces"][fn] = {"x_design": FACES[fn]["x"], "offset": b, "slope": phi, "y_ref": YMID}
out["uncertainty"] = {
    "tags": "cov / sd = J COV J^T of the solved parameters [face offset, face slope, stand-off, y, z] (linearised at the solution, "
            "final robust weights), turned with the anchor turn (R C R^T); a fixed shift leaves it unchanged",
    "faces": "cov of (offset at y_ref, slope dx/dy) through the same anchor map; sd = [offset m, turn deg]",
    "anchor": "CONDITIONAL on the anchor: the wall-based turn and shift are applied as fixed numbers; their own uncertainty and "
              "their correlation with the map (same flight, same ToF) are NOT included",
    "not_for": "not a stand-alone safety or map-acceptance number (codex survey-uncertainty 2026-10-31)"}

# observability (codex survey-validation-independence): what the measurements alone say about the map parameters. The map
# priors are the last rows of r (faces, then tags); without them (trajectory priors kept) the information on the map
# parameters (Schur complement) is compared with the priors' in prior-sd units: an eigenvalue < 1 is a combination the
# data pin down less well than the design prior does.
_mp = []
for fn in faces_used:
    _mp += [("face_%s.offset" % fn, idx["face_" + fn].start, MP["face_sd"]),
            ("face_%s.slope" % fn, idx["face_" + fn].start + 1, math.radians(MP["face_angle_sd_deg"]))]
for k in tags_used:
    _mp += [("tag_%s.standoff" % k, idx["tag_" + k].start, MP["tag_standoff_sd"]), ("tag_%s.y" % k, idx["tag_" + k].start + 1, MP["tag_sd_along"]),
            ("tag_%s.z" % k, idx["tag_" + k].start + 2, MP["tag_sd_z"])]
_nmap = 2 * len(faces_used) + 3 * len(tags_used)
_Jd, _wd = J[:len(r0) - _nmap], w[:len(r0) - _nmap]
_Hd = _Jd.T @ (_Jd * _wd[:, None])
_mi = np.array([m_[1] for m_ in _mp], int); _oi = np.setdiff1d(np.arange(len(v)), _mi)
try:
    _Imap = _Hd[np.ix_(_mi, _mi)] - _Hd[np.ix_(_mi, _oi)] @ np.linalg.solve(_Hd[np.ix_(_oi, _oi)], _Hd[np.ix_(_oi, _mi)])
    _D = np.diag([m_[2] for m_ in _mp]); _In = _D @ _Imap @ _D; _In = 0.5 * (_In + _In.T)
    _ev, _evec = np.linalg.eigh(_In)
    with np.errstate(all="ignore"):
        _sd_data = np.sqrt(np.diag(np.linalg.pinv(_Imap)))
    _sd_data[np.diag(_Imap) <= 0] = np.inf
except np.linalg.LinAlgError as e:
    _ev = None; print("[survey] observability not computed:", e)
if _ev is not None:
    weak = []
    for j in np.argsort(_ev)[:3]:
        top = np.argsort(-np.abs(_evec[:, j]))[:3]
        weak.append({"eig": float(_ev[j]), "params": [[_mp[t][0], round(float(_evec[t, j]), 3)] for t in top]})
    per = {m_[0]: {"sd_with_priors": float(SD[m_[1]]), "sd_data_only": float(_sd_data[i]) if np.isfinite(_sd_data[i]) else None,
                   "sd_prior": m_[2]} for i, m_ in enumerate(_mp)}
    # the wall anchor then fixes the map's common x shift (every face offset together; the tags follow their faces) and common
    # y shift (every tag y); those two directions are left out for "eligible" (they are counted in n_eig_below_1)
    _g = np.zeros((len(_mp), 2))
    for i, m_ in enumerate(_mp):
        if m_[0].endswith(".offset"):
            _g[i, 0] = 1.0 / m_[2]
        elif m_[0].endswith(".y"):
            _g[i, 1] = 1.0 / m_[2]
    _g = _g[:, np.abs(_g).sum(0) > 0]
    _Q = np.linalg.svd(np.eye(len(_mp)) - _g @ np.linalg.pinv(_g))[0][:, :len(_mp) - _g.shape[1]]
    _ev2 = np.linalg.eigvalsh(_Q.T @ _In @ _Q)
    n_weak = int((_ev2 < 1.0).sum())
    out["observability"] = {
        "rule": "eigenvalues of the data-only information on the map parameters in prior-sd units (trajectory priors kept, map "
                "priors left out); eig < 1: the data constrain that combination less than the prior; eligible = no eig < 1 once "
                "the common x / y shift (fixed by the wall anchor) is left out",
        "eig_min": float(_ev.min()), "n_eig_below_1": int((_ev < 1.0).sum()), "weakest": weak, "params": per,
        "eig_min_without_shift": float(_ev2.min()), "n_eig_below_1_without_shift": n_weak, "eligible": n_weak == 0,
        "note": "a rule of this script, not a proof of observability; the design priors and the wall anchor still fix the frame"}
    print("[survey] observability: min eig %.3g (prior-sd units), %d below 1; without the common shift min %.3g, %d below 1 -> "
          "eligible %s; weakest %s" % (_ev.min(), int((_ev < 1.0).sum()), _ev2.min(), n_weak, n_weak == 0, weak[0]["params"]))
with open(os.path.join(OUT, "traj.csv"), "w") as f:
    f.write("t,x,y,z,comp\n")
    for row in zip(pt, x, y, z, comp % 360.0):
        f.write("%.3f,%.4f,%.4f,%.4f,%.3f\n" % row)

# ------------------------------------------------------------------ stack the ToF frames
pts = []
raw = os.path.join(A.run, "tof_raw")
meta = json.load(open(os.path.join(raw, "meta.json"))) if os.path.isdir(raw) else None
if meta:
    nxt = -1e9                          # sensor -> body -> map in survey_geom (the replay exporter uses the same functions)
    for fn_ in sorted(glob.glob(os.path.join(raw, "chunk_*.npz"))):
        Z = np.load(fn_)
        for t, dep in zip(Z["t"], Z["depth"]):
            if t < nxt or not (t0 < t < t1):
                continue
            nxt = t + 1.0 / TF["frames_hz"]
            B = SG.frame_points(dep, TF, TF["pixel_stride"])                   # body FLU (mount rotation + position)
            pts.append(SG.body_to_map(B, np.interp(t, pt, x), np.interp(t, pt, y), np.interp(t, pt, z), float(np.interp(t, pt, comp))))
PTS = np.concatenate(pts) if pts else np.zeros((0, 3))
if len(PTS):
    key = np.floor(PTS / GR["cell"]).astype(np.int64)
    _, first, cnt = np.unique(key, axis=0, return_index=True, return_counts=True)
    PTS, CNT = PTS[first], cnt
    np.savez_compressed(os.path.join(OUT, "points.npz"), xyz=PTS.astype(np.float32), count=CNT.astype(np.int32))
print("[survey] ToF points %d voxels" % len(PTS))


# line fits from the points (independent of the plane fits): faces and walls
def robust_line(u, w_):
    """w_ = a + b u, trimmed least squares"""
    keep = np.ones(len(u), bool)
    for _ in range(4):
        A_ = np.vstack([np.ones(keep.sum()), u[keep]]).T
        (a, b), *_ = np.linalg.lstsq(A_, w_[keep], rcond=None)
        res = w_ - (a + b * u); s = 1.4826 * np.median(np.abs(res[keep])) + 1e-4
        keep = np.abs(res) < 3 * s
    return a, b, keep.sum(), float(np.std(res[keep]))


zin = (PTS[:, 2] > GR["z_min"]) & (PTS[:, 2] < GR["z_max"]) if len(PTS) else np.zeros(0, bool)
for fn in faces_used:
    xf, phi = face_line(v, fn); sg = FACES[fn]["sgn"]
    dist = PTS[:, 0] - (xf + phi * (PTS[:, 1] - YMID))
    # the face = front-most surface: points within the band in front of / at the solved line, inside the rack span
    m = zin & (np.abs(dist) < GR["face_band"]) & (PTS[:, 1] > RY0) & (PTS[:, 1] < RY1)
    if m.sum() > 200:
        a, b, nk, s = robust_line(PTS[m, 1] - YMID, PTS[m, 0])
        out["faces"][fn]["points"] = {"offset": a - FACES[fn]["x"], "slope": b, "n": int(nk), "rms": round(s, 4),
                                      "kind": "internal check: the same ToF frames on the same solved path (not independent)"}
for wn, (ax_, val) in WALLS.items():
    i = 0 if ax_ == "x" else 1
    m = zin & (np.abs(PTS[:, i] - val) < MP["wall_band"])
    m &= (PTS[:, 1] < RY0 - 0.05) | (PTS[:, 1] > RY1 + 0.05) if ax_ == "y" else (PTS[:, 0] < G.RACKS_X[0][0] - 0.05) | (PTS[:, 0] > G.RACKS_X[-1][1] + 0.05)
    # keep away from the corners: near a corner the band also catches the perpendicular wall, which pulled every wall into the
    # room by ~1 cm (2026-10-31: S +0.86, N -0.86 cm with the mesh walls exactly at the design 0.100 / 10.100 m)
    cm_ = MP.get("wall_corner_margin", 0.4)
    if ax_ == "x":
        m &= (PTS[:, 1] > G.ROOMS[room]["y"][0] + cm_) & (PTS[:, 1] < G.ROOMS[room]["y"][1] - cm_)
    else:
        m &= (PTS[:, 0] > G.WALL + cm_) & (PTS[:, 0] < G.X_MAX - G.WALL - cm_)
    if m.sum() > 100:
        c_ = PTS[m, i]; med = np.median(c_); k2 = np.abs(c_ - med) < 0.03; c2 = c_[k2]
        u_ = PTS[m, 1 - i][k2]                                   # along the wall
        sl = robust_line(u_ - np.mean(u_), c2)[1] if np.ptp(u_) > 1.0 else None
        out["walls"][wn] = {"axis": ax_, "design": val, "measured": round(float(np.mean(c2)), 4), "offset": round(float(np.mean(c2)) - val, 4),
                            "n": int(len(c2)), "sd": round(float(np.std(c2)), 4), "slope": None if sl is None else round(float(sl), 5),
                            "span": round(float(np.ptp(u_)), 2)}
# gauge: the solve fixes the frame only through the design priors, so a real change (a rack line moved 3 cm, sv5 2026-10-31)
# is shared out over every face, and the walls move with it (sv5: W -1.49, E -1.68 cm, CE -1.64 cm, CW +1.48 of +3 cm).
# Moving the whole map changes no measurement, so put it where the structure that does not move (the room walls) says.
anchor = MP.get("anchor", "walls")
# rotation: a map turned by eps (CCW) shows x-walls with slope dx/dy = -eps and y-walls with dy/dx = +eps; turn it back about the
# room centre (no measurement changes under a rigid turn). Only walls seen over more than wall_rot_min_span metres count.
rot = 0.0
if anchor == "walls" and MP.get("anchor_rotation", True):
    est = [(-w["slope"] if w["axis"] == "x" else w["slope"], w["span"]) for w in out["walls"].values()
           if w.get("slope") is not None and w["span"] >= MP.get("wall_rot_min_span", 2.0)]
    if est:
        rot = float(np.average([e[0] for e in est], weights=[e[1] for e in est]))   # eps (rad), longer walls weigh more
cxr, cyr = 0.5 * G.X_MAX, 0.5 * sum(G.ROOMS[room]["y"])


def move_all(f, a):
    """apply the rigid map f (a = its turn, rad) to every map item: tags (+ cov), face lines solved / points (+ cov), the path,
    the points. Lines are moved as lines (codex survey-anchor-lines): offset at y_ref and slope both follow."""
    global x, y, comp, PTS
    for k, t in out["tags"].items():
        if t.get("observed"):
            t["x"], t["y"] = f(t["x"], t["y"])
            if a:
                TAGCOV[k] = SG.turn_cov(TAGCOV[k], a)
    for fn, fc in out["faces"].items():
        xd = FACES[fn]["x"]
        FACECOV[fn] = SG.line_cov(fc["offset"], fc["slope"], FACECOV[fn], xd, fc["y_ref"], f)
        fc["offset"], fc["slope"] = SG.xform_line(fc["offset"], fc["slope"], xd, fc["y_ref"], f)
        if "points" in fc:
            fc["points"]["offset"], fc["points"]["slope"] = SG.xform_line(fc["points"]["offset"], fc["points"]["slope"], xd, fc["y_ref"], f)
        v[idx["face_" + fn]] = [fc["offset"], fc["slope"]]
    x, y = f(x, y); comp = comp - math.degrees(a)            # compass heading: a CCW turn of the map lowers it
    if len(PTS):
        PTS[:, 0], PTS[:, 1] = f(PTS[:, 0], PTS[:, 1])


if rot:
    # the map turns by -rot about the room centre (no measurement changes under a rigid turn)
    move_all(SG.turn_about(cxr, cyr, -rot), -rot)
    for w in out["walls"].values():
        if w.get("slope") is not None:
            w["slope"] = round(w["slope"] + (rot if w["axis"] == "x" else -rot), 5)
    # wall offsets after the turn (about the room centre the walls' mean positions barely move; recompute from the points)
    for wn, (ax_w, val_w) in WALLS.items():
        if wn in out["walls"]:
            i_ = 0 if ax_w == "x" else 1; w = out["walls"][wn]
            m_ = zin & (np.abs(PTS[:, i_] - val_w) < MP["wall_band"])
            cm2 = MP.get("wall_corner_margin", 0.4)
            m_ &= ((PTS[:, 1] > G.ROOMS[room]["y"][0] + cm2) & (PTS[:, 1] < G.ROOMS[room]["y"][1] - cm2)) if ax_w == "x" else \
                  ((PTS[:, 0] > G.WALL + cm2) & (PTS[:, 0] < G.X_MAX - G.WALL - cm2))
            m_ &= ((PTS[:, 0] < G.RACKS_X[0][0] - 0.05) | (PTS[:, 0] > G.RACKS_X[-1][1] + 0.05)) if ax_w == "x" else \
                  ((PTS[:, 1] < RY0 - 0.05) | (PTS[:, 1] > RY1 + 0.05))
            if m_.sum() > 100:
                c_ = PTS[m_, i_]; med = np.median(c_); c2 = c_[np.abs(c_ - med) < 0.03]
                w["measured"] = round(float(np.mean(c2)), 4); w["offset"] = round(float(np.mean(c2)) - val_w, 4)
ax_ = [w["offset"] for k, w in out["walls"].items() if w["axis"] == "x"]; ay_ = [w["offset"] for k, w in out["walls"].items() if w["axis"] == "y"]
sx = -float(np.mean(ax_)) if anchor == "walls" and len(ax_) == 2 else 0.0
sy = -float(np.mean(ay_)) if anchor == "walls" and len(ay_) == 2 else 0.0
out["anchor"] = {"mode": anchor, "shift_x": round(sx, 4), "shift_y": round(sy, 4), "turn_deg": round(math.degrees(rot), 4),
                 "transform": {"turn_rad": -rot, "about": [cxr, cyr], "then_shift": [sx, sy],
                               "order": "p' = about + R(turn_rad) (p - about) + then_shift (the map, path, points and lines)"},
                 "refs": sorted(out["walls"]) if anchor == "walls" else [],
                 "note": "the walls fix the map frame; their offsets after the anchor are NOT independent accuracy evidence"}
if sx or sy:
    move_all(SG.shift_by(sx, sy), 0.0)
    for w in out["walls"].values():
        d_ = sx if w["axis"] == "x" else sy
        w["measured"] = round(w["measured"] + d_, 4); w["offset"] = round(w["offset"] + d_, 4)
if sx or sy or rot:
    with open(os.path.join(OUT, "traj.csv"), "w") as f:
        f.write("t,x,y,z,comp\n")
        for row in zip(pt, x, y, z, comp % 360.0):
            f.write("%.3f,%.4f,%.4f,%.4f,%.3f\n" % row)
    if len(PTS):
        np.savez_compressed(os.path.join(OUT, "points.npz"), xyz=PTS.astype(np.float32), count=CNT.astype(np.int32))
# rounded only now (full precision through the anchor steps)
for k, t in out["tags"].items():
    if t.get("observed"):
        C3 = TAGCOV[k]
        t["x"], t["y"], t["z"] = (round(float(t[a_]), 4) for a_ in "xyz")
        t["sd"] = [round(float(math.sqrt(max(C3[i, i], 0.0))), 4) for i in range(3)]
        t["cov"] = [[float("%.4g" % C3[i, j]) for j in range(3)] for i in range(3)]
for fn, fc in out["faces"].items():
    C2 = FACECOV[fn]; k_ = fc["slope"]
    fc["turn_deg"] = round(math.degrees(math.atan(k_)), 3)
    fc["sd"] = [round(float(math.sqrt(C2[0, 0])), 4), round(math.degrees(math.sqrt(C2[1, 1])) / (1 + k_ * k_), 3)]
    fc["cov"] = [[float("%.4g" % C2[i, j]) for j in range(2)] for i in range(2)]
    fc["offset"] = round(float(fc["offset"]), 4); fc["slope"] = float("%.6g" % k_)
    if "points" in fc:
        pp = fc["points"]; pp["turn_deg"] = round(math.degrees(math.atan(pp["slope"])), 3)
        pp["offset"] = round(float(pp["offset"]), 4); pp["slope"] = float("%.6g" % pp["slope"])


def _sha(p_):
    h = hashlib.sha256()
    with open(p_, "rb") as fh:
        for blk in iter(lambda: fh.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


# what made this result (codex survey-replay-consistency): the merged config, the inputs and the code
out["config_effective"] = CFG; out["config_path"] = os.path.abspath(A.config); out["config_sha256"] = _sha(A.config); out["set"] = A.set
_inp = {n_: os.path.join(A.run, n_) for n_ in ("pose.csv", "tags_raw.jsonl", "sense.csv", "loc.jsonl", "plan.json")}
if os.path.isdir(raw):
    _inp.update({os.path.relpath(p_, A.run): p_ for p_ in [os.path.join(raw, "meta.json")] + sorted(glob.glob(os.path.join(raw, "chunk_*.npz")))})
out["inputs_sha256"] = {n_: _sha(p_) for n_, p_ in _inp.items() if os.path.exists(p_)}
out["design_sha256"] = _sha(A.design)
out["tag_quality"] = {"policy_unavailable": TQ.POLICY, "decisions": dict(TQ.COUNT)}   # how many detections each rule outcome had
out["code_sha256"] = {os.path.basename(p_): _sha(p_) for p_ in (os.path.abspath(__file__), os.path.join(HERE, "survey_geom.py"), os.path.join(HERE, "qvix_grid.py"), os.path.join(HERE, "tag_quality.py"))}
print("[survey] anchor %s: map turned %+.4f deg, shifted x %+.4f y %+.4f m" % (anchor, math.degrees(rot), sx, sy))
json.dump(out, open(os.path.join(OUT, "survey_map.json"), "w"), indent=1)
ob = [t for t in out["tags"].values() if t.get("observed")]
print("[survey] tags corrected %d / %d, faces %s, walls %s -> %s" % (len(ob), len(out["tags"]), list(out["faces"]), list(out["walls"]), OUT))

# ------------------------------------------------------------------ figures (cad venv matplotlib)
if A.no_plots or not len(PTS):
    sys.exit(0)
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
except Exception as e:  # noqa: BLE001
    print("[survey] no plots:", e); sys.exit(0)
truth_tags = None
tp = os.environ.get("SURVEY_TRUTH")                 # evaluation overlay only (survey_eval.py sets it); never used above
if tp and os.path.exists(tp):
    truth_tags = json.load(open(tp))["tags"]
fig, axp = plt.subplots(figsize=(7, 11))
for (a0, a1) in G.RACKS_X:
    axp.add_patch(Rectangle((a0, RY0), a1 - a0, RY1 - RY0, fill=False, ec="#999", lw=0.8, ls="--"))
for wn, (ax_, val) in WALLS.items():
    if ax_ == "y":
        axp.axhline(val, color="#999", lw=0.8, ls="--")
    else:
        axp.axvline(val, color="#999", lw=0.8, ls="--")
H2, xe, ye = np.histogram2d(PTS[zin, 0], PTS[zin, 1], bins=[np.arange(0, G.X_MAX + 0.02, GR["cell"]), np.arange(-0.2, 10.4, GR["cell"])])
axp.imshow(np.log1p(H2.T), origin="lower", extent=[xe[0], xe[-1], ye[0], ye[-1]], cmap="Greys", aspect="equal", vmax=np.log1p(H2.max()) * 0.6)
axp.plot(x, y, color="#1f6fd9", lw=1, label="solved path")
for k, t in out["tags"].items():
    d = DESIGN["tags"][k]
    axp.plot(d["x"], d["y"], "o", mfc="none", mec="#888", ms=4)
    if t.get("observed"):
        axp.plot(t["x"], t["y"], "+", color="#2f8f4e", ms=7)
    if truth_tags:
        axp.plot(truth_tags[k]["x"], truth_tags[k]["y"], "x", color="#d9731f", ms=5)
axp.set_xlim(0, G.X_MAX); axp.set_ylim(-0.2, 10.4); axp.set_xlabel("map x (m)"); axp.set_ylabel("map y (m)")
axp.set_title("survey top view: ToF points (grey), design (dashed, o), corrected tags (+)" + (", truth (x)" if truth_tags else ""), fontsize=8)
fig.tight_layout(); fig.savefig(os.path.join(OUT, "top.png"), dpi=130); plt.close(fig)
for fn in faces_used:
    xf, phi = face_line(v, fn)
    dist = FACES[fn]["sgn"] * (PTS[:, 0] - (xf + phi * (PTS[:, 1] - YMID)))      # + = into the aisle
    m = (dist > -0.4) & (dist < 0.08) & (PTS[:, 1] > RY0 - 0.5) & (PTS[:, 1] < RY1 + 0.5)
    fig, axf = plt.subplots(figsize=(14, 5.2))
    sc = axf.scatter(PTS[m, 1], PTS[m, 2], c=dist[m], s=1.2, cmap="viridis", vmin=-0.4, vmax=0.05)
    for k, t in out["tags"].items():
        if t["face"] != fn:
            continue
        d = DESIGN["tags"][k]
        axf.plot(d["y"], d["z"], "s", mfc="none", mec="#888", ms=6)
        if t.get("observed"):
            axf.plot(t["y"], t["z"], "+", color="#d62728", ms=9, mew=1.5)
        if truth_tags:
            axf.plot(truth_tags[k]["y"], truth_tags[k]["z"], "x", color="#ff7f0e", ms=6)
    axf.set_aspect("equal"); axf.set_xlabel("y along the face (m)"); axf.set_ylabel("z (m)")
    axf.set_title("front view %s: ToF points coloured by depth behind the face (m); tags design (square), corrected (+)%s"
                  % (fn, ", truth (x)" if truth_tags else ""), fontsize=8)
    axf.set_xlim(RY0 - 0.6, RY1 + 0.6); axf.set_ylim(-0.1, 4.3)
    fig.colorbar(sc, ax=axf, shrink=0.7, pad=0.01); fig.tight_layout(); fig.savefig(os.path.join(OUT, "front_%s.png" % fn), dpi=120); plt.close(fig)
print("[survey] figures in", OUT)
