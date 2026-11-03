#!/usr/bin/env python3
"""ToF quality model edges and range check (2026-11-01, codex tof-edge-model). No sim: synthetic depth frames only.
  1. codex repro input (224 x 172, left 0.3 m / right 1.0 m, hfov 85.1 deg, seed 7), boundary columns 85/86:
     separated edge term (noise 0, no dropout, edge prob 1) -> every boundary value inside [0.3, 1.0], none <= 0
  2. the JSON as it is -> no finite value <= 0 anywhere, no range past max_range_m
  3. before / after on the same seed and frames: the old model (logs/bak_review6_20261101/tof_sim_model.py) vs the new one,
     frontal and 30 deg oblique rack-like faces (tray rows 0.37 m / gaps to a 1.0 m back wall): valid fraction and the plane fit
     (wall_sense_fit.fit) distance / angle error against the true plane
Run on the host:  python3 tests/test_tof_model_offline.py      exit = failures"""
import importlib.util, json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from tof_sim_model import ToFModel  # noqa: E402
from wall_sense_fit import fit  # noqa: E402
CFG = os.path.join(ROOT, "config", "tof_sim_model.json")
F = []


def check(name, cond, info=""):
    print("%s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (" + info + ")") if info else ""))
    if not cond:
        F.append(name)


def load_old():
    spec = importlib.util.spec_from_file_location("tof_old", os.path.join(ROOT, "logs", "bak_review6_20261101", "tof_sim_model.py"))
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m.ToFModel


def model(cls, hfov, **over):
    m = cls(CFG, hfov); m.p.update(over); m.rng = np.random.default_rng(m.p.get("seed", 7)); return m


# 1 + 2: codex repro input
H, W = 172, 224; HF = math.radians(85.1)
img = np.full((H, W), 1.0, np.float32); img[:, :W // 2 - 26] = 0.3        # boundary between columns 85 and 86
sep = model(ToFModel, HF, noise_sd0=0.0, noise_sd_rel=0.0, amp_range_m=1e3, amp_soft=1e-6, edge_mix_prob=1.0)
o = sep.apply(img); b = o[:, 85:87]; fb = b[np.isfinite(b)]
check("separated edge term: boundary values inside [0.3, 1.0]", fb.size > 0 and fb.min() >= 0.3 - 1e-6 and fb.max() <= 1.0 + 1e-6,
      "finite %d, min %.6f, max %.6f, changed %d" % (fb.size, fb.min(), fb.max(), int(np.sum(np.abs(b - img[:, 85:87]) > 1e-6))))
check("separated edge term: none <= 0", int(np.sum(fb <= 0)) == 0, "%d" % int(np.sum(fb <= 0)))
cur = model(ToFModel, HF)
o = cur.apply(img); fin = o[np.isfinite(o)]
check("JSON as it is: no finite value <= 0 in the frame", int(np.sum(fin <= 0)) == 0,
      "finite %d, min %.4f, max %.4f; boundary cols finite %d min %.4f max %.4f" % (
          fin.size, fin.min(), fin.max(), int(np.isfinite(o[:, 85:87]).sum()), np.nanmin(np.where(np.isfinite(o[:, 85:87]), o[:, 85:87], np.nan)),
          np.nanmax(np.where(np.isfinite(o[:, 85:87]), o[:, 85:87], np.nan))))
old = model(load_old(), HF); oo = old.apply(img); ofin = oo[np.isfinite(oo)]
print("      old model, same input: finite %d, <= 0: %d, min %.4f, max %.4f" % (ofin.size, int(np.sum(ofin <= 0)), ofin.min(), ofin.max()))
far = np.full((H, W), 5.9, np.float32)
m6 = model(ToFModel, HF, amp_range_m=1e3, amp_soft=1e-6, noise_sd0=0.3, edge_mix_prob=0.0)
o6 = m6.apply(far); rn = m6._rays(H, W)[2]
check("no finite range past max_range_m after the noise", int(np.sum(np.isfinite(o6) & (o6 * rn > 6.0))) == 0)


# 3: before / after on rack-like faces
def face(d_body, alpha_deg, hfov, sensor_x=0.075):
    """depth along the optical axis of a plane at body distance d_body turned by alpha (camera yaw vs facing it squarely);
    tray rows 8 cm high every 12 cm, the 4 cm gaps show a back wall 1.0 m behind the face"""
    fx = (W / 2.0) / math.tan(hfov / 2.0)
    u, v = np.meshgrid(np.arange(W), np.arange(H)); ry = -(u - (W - 1) / 2.0) / fx; rz = -(v - (H - 1) / 2.0) / fx
    a = math.radians(alpha_deg); n = (math.cos(a), math.sin(a))     # plane normal in the camera (forward, left)
    ds = d_body - sensor_x * math.cos(a)
    t = ds / (n[0] + n[1] * ry); z = t * rz
    gap = (np.mod(z + 1.0, 0.12) > 0.08)
    tb = (ds + 1.0) / (n[0] + n[1] * ry)
    return np.where(gap, tb, t).astype(np.float32), ds


rows = []
for name, d_body, al in (("frontal 0.37 m", 0.37, 0.0), ("oblique 30 deg 0.45 m", 0.45, 30.0)):
    img2, ds = face(d_body, al, 1.8588)
    for tag, cls in (("old", load_old()), ("new", ToFModel)):
        m = model(cls, 1.8588); vals, de, ae, conf = [], [], [], []
        for k in range(40):
            o = m.apply(img2); vals.append(float(np.mean(np.isfinite(o) & (o > 0))))
            r, _ = fit(o)
            if r:
                de.append(r["d"] - d_body); ae.append(r["alpha"] + al); conf.append(r["conf"])   # wall_sense alpha = camera yaw vs the face = -(plane turn)
        rows.append({"case": name, "model": tag, "valid_mean": round(float(np.mean(vals)), 4), "fits": len(de),
                     "d_err_mean_m": round(float(np.mean(de)), 5) if de else None, "d_err_sd_m": round(float(np.std(de)), 5) if de else None,
                     "alpha_err_mean_deg": round(float(np.mean(ae)), 3) if ae else None, "alpha_err_sd_deg": round(float(np.std(ae)), 3) if ae else None,
                     "conf_mean": round(float(np.mean(conf)), 3) if conf else None})
for r in rows:
    print("      %-22s %s: valid %.4f, fits %d/40, d err %+.5f sd %.5f m, alpha err %+.3f sd %.3f deg, conf %.3f" % (
        r["case"], r["model"], r["valid_mean"], r["fits"], r["d_err_mean_m"], r["d_err_sd_m"], r["alpha_err_mean_deg"],
        r["alpha_err_sd_deg"], r["conf_mean"]))
for nm in ("frontal 0.37 m", "oblique 30 deg 0.45 m"):
    n_ = [r for r in rows if r["case"] == nm and r["model"] == "new"][0]
    check("%s: new model still fits every frame within 1 cm / 1 deg mean" % nm,
          n_["fits"] == 40 and abs(n_["d_err_mean_m"]) < 0.01 and abs(n_["alpha_err_mean_deg"]) < 1.0)
json.dump(rows, open(os.path.join(ROOT, "logs", "1930_tof_edge_before_after.json"), "w"), indent=1)
print("%d failure(s)" % len(F)); sys.exit(len(F))
