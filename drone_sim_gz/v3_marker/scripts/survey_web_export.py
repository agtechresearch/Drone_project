#!/usr/bin/env python3
"""Export survey flights for the web replay page (v3_marker, 2026-10-31).
  QVIX_YAW_DEG=<yaw> python3 survey_web_export.py <run_dir> <world name> <truth.json> <out.json> [rot_deg]
One JSON per flight: design map, the world (module yaw, tag offsets: design / truth / corrected), the solved trajectory and the
true one (sim only), stage start times, tag detection events and the ToF points with the time each 5 cm voxel was first seen
(int16 cm x/y/z + uint16 0.1 s, base64), stacked on the SOLVED trajectory with survey_map.py's own functions (survey_geom:
mount rotation rpy_deg + position) and the config the solver USED (survey_map.json "config_effective", 2026-11-01 codex
survey-replay-consistency; an older result without it falls back to the current config file - labelled "config_source").
What is only for the page (frame rate, pixel stride, range / box clip, 5 cm voxels) is listed in "display"."""
import base64, csv, glob, json, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); V3 = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import qvix_grid as G  # noqa: E402
import survey_geom as SG  # noqa: E402

run, name, truth_p, out_p = sys.argv[1:5]; rot = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
design = json.load(open(os.path.join(V3, "gz", "tags_map.json")))["tags"]
truth = json.load(open(truth_p))
SDIR = os.path.join(run, os.environ.get("SURVEY_DIR", "survey"))     # another solve of the same run (absolute path allowed)
S = json.load(open(os.path.join(SDIR, "survey_map.json")))
if "config_effective" in S:                         # the solver's merged config (after --set)
    CFG = S["config_effective"]; CFG_SRC = "survey_map.json config_effective (sha256 of the file %s)" % S.get("config_sha256", "?")
else:
    CFG = json.load(open(os.path.join(V3, "config", "survey_starling2.json")))
    CFG_SRC = "CURRENT config/survey_starling2.json (old survey_map.json without config_effective: may differ from the solve)"
TF = CFG["tof"]
DISPLAY = {"frame_dt_s": 0.5, "pixel_stride": 4, "range_max_m": 4.0, "box_m": {"x": [-0.3, 6.1], "y": [-0.4, 10.6], "z": [-0.05, 2.4]},
           "voxel_m": 0.05, "note": "page-only simplification; the solver's points use frames_hz / pixel_stride / range of the config"}
E = json.load(open(os.path.join(SDIR, "eval.json")))
J = np.genfromtxt(os.path.join(SDIR, "traj.csv"), delimiter=",", names=True)
t0 = float(J["t"][0])

# truth trajectory (tags.csv true pose, world -> map)
R = [r for r in csv.DictReader(open(os.path.join(run, "tags.csv"))) if r["true_x"] not in ("nan", "")]
tt = np.array([float(r["t_sim"]) for r in R]); W = [G.world_to_model(float(r["true_x"]), float(r["true_y"])) for r in R]
TX = np.array([w[0] for w in W]); TY = np.array([w[1] for w in W])


def dec(t, hz):
    keep, nxt = [], -1e9
    for i, v in enumerate(t):
        if v >= nxt:
            keep.append(i); nxt = v + 1.0 / hz
    return np.array(keep)


k = dec(J["t"], 5.0)
traj = [[round(float(J["t"][i]) - t0, 2), round(float(J["x"][i]), 3), round(float(J["y"][i]), 3), round(float(J["z"][i]), 3),
         round(float(J["comp"][i]), 1)] for i in k]
kt = dec(tt, 5.0)
truth_traj = [[round(float(tt[i]) - t0, 2), round(float(TX[i]), 3), round(float(TY[i]), 3)] for i in kt if tt[i] >= t0]

# stages (wk.csv)
stages, last = [], None
for r in csv.DictReader(open(os.path.join(run, "wk.csv"))):
    if r["stage"] and r["stage"] != last:
        stages.append([round(float(r["t_sim"]) - t0, 1), r["stage"]]); last = r["stage"]

# tag detections (raw), one event per tag per 0.5 s
events, lastk = [], {}
for s in open(os.path.join(run, "tags_raw.jsonl")):
    m = json.loads(s)
    for d in m["dets"]:
        if m["t"] - lastk.get(d["id"], -1e9) >= 0.5 and m["t"] >= t0:
            lastk[d["id"]] = m["t"]; events.append([round(m["t"] - t0, 1), int(d["id"])])

# ToF points with first-seen time
raw = os.path.join(run, "tof_raw")
st = DISPLAY["pixel_stride"]; comp_u = np.degrees(np.unwrap(np.radians(J["comp"])))
seen = {}; nxt = -1e9
for fn in sorted(glob.glob(os.path.join(raw, "chunk_*.npz"))):
    Z = np.load(fn)
    for t, dep in zip(Z["t"], Z["depth"]):
        if t < nxt or not (J["t"][0] < t < J["t"][-1]):
            continue
        nxt = t + DISPLAY["frame_dt_s"]
        B = SG.frame_points(dep, TF, st, rmax=DISPLAY["range_max_m"])
        Pm = SG.body_to_map(B, np.interp(t, J["t"], J["x"]), np.interp(t, J["t"], J["y"]), np.interp(t, J["t"], J["z"]),
                            float(np.interp(t, J["t"], comp_u)))
        X, Y, Zp = Pm[:, 0], Pm[:, 1], Pm[:, 2]; bx_ = DISPLAY["box_m"]
        keep = (Zp > bx_["z"][0]) & (Zp < bx_["z"][1]) & (X > bx_["x"][0]) & (X < bx_["x"][1]) & (Y > bx_["y"][0]) & (Y < bx_["y"][1])
        for key in zip(np.floor(X[keep] / 0.05).astype(int), np.floor(Y[keep] / 0.05).astype(int), np.floor(Zp[keep] / 0.05).astype(int)):
            if key not in seen:
                seen[key] = t - t0
pts = np.array([[k_[0] * 5 + 2, k_[1] * 5 + 2, k_[2] * 5 + 2, int(v * 10)] for k_, v in seen.items()], dtype=np.int32)
pts = pts[np.argsort(pts[:, 3])]
blob = base64.b64encode(np.concatenate([pts[:, :3].astype(np.int16).ravel().view(np.uint8),
                                        pts[:, 3].astype(np.uint16).ravel().view(np.uint8)]).tobytes()).decode()

tags = {}
for k_, d in design.items():
    tr = truth["tags"][k_]; c_ = S["tags"][k_]
    tags[k_] = {"face": d["face"], "design": [d["x"], d["y"], d["z"]], "truth": [tr["x"], tr["y"], tr["z"]],
                "corr": [c_["x"], c_["y"], c_["z"]] if c_.get("observed") else None, "n": c_.get("n_obs", 0)}
cond = open(os.path.join(run, "condition.txt")).read()
lost = None
for line in open(os.path.join(run, "mission.log"), errors="ignore"):
    if "loc lost" in line and "EMERGENCY" in line:
        lost = line.strip()[:200]
out = {"name": name, "run": os.path.basename(os.path.normpath(run)), "yaw": G.MODEL_YAW_DEG, "rot": rot, "t_end": traj[-1][0],
       "design": {"racks_x": G.RACKS_X, "units_y": G.RACK_UNITS_Y["R1"], "room_y": G.ROOMS["R1"]["y"], "x_max": G.X_MAX, "wall": G.WALL,
                  "door_x": G.PARTITION_DOOR_X, "aisles": {k: list(v) for k, v in G.AISLES.items()}, "model_to_world": G.MODEL_TO_WORLD},
       "perturbation": truth["perturbation"], "tags": tags, "faces": S["faces"], "walls": S["walls"],
       "eval": {"path": E["path"]}, "traj": traj, "truth_traj": truth_traj, "stages": stages, "tag_events": events,
       "n_points": int(len(pts)), "points_b64": blob, "lost": lost, "condition": cond,
       "config_source": CFG_SRC, "display": DISPLAY, "survey_provenance": {k_: S.get(k_) for k_ in ("config_sha256", "set", "inputs_sha256", "code_sha256", "anchor")}}
json.dump(out, open(out_p, "w"), separators=(",", ":"))
print(name, "points", len(pts), "traj", len(traj), "events", len(events), "size %.0f kB" % (os.path.getsize(out_p) / 1024))
