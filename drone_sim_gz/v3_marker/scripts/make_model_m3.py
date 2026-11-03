#!/usr/bin/env python3
"""Corrected copy "m3" of the wall-keep model starling2_tof (2026-11-02; user: real vehicle with battery weighs 297.5 g).
  python3 make_model_m3.py            -> v3_marker/gz/variants/m3/starling2_tof/ (model.sdf, model.config, meshes/) + VARIANT.json
Same rules as m2 (make_model_m2.py): the CURRENT model v3_marker/gz/models/starling2_tof is only read, the variant keeps the same
model name and is used only by an instance started with WK_MODEL_DIR=<v3_marker/gz/variants/m3>. m2 is left as it is.
Changes, nothing else:
  1. mass: links sum to the measured 0.2975 kg -> base_link 0.2975 - 4 x 0.002 (rotors) = 0.2895. Inertia kept (box estimate,
     +4.4 % mass vs 0.285 not propagated - stated in VARIANT.json).
  2. IMU position as m2: real EKF2_IMU_POS_X/Y/Z = 0.0043 / 0.0073 / -0.016 m (body FRD) -> gz FLU 0.0043 -0.0073 0.016.
Motor constant NOT changed (1.78e-6): with THR_MDL_FAC 0.9 and omega = 150 + 850 u, a hover thrust of 0.356 / 0.357 (real ulog
hover_thrust_estimate, worklog 2026-10-27) at 0.2975 kg needs 1.7837e-6 / 1.7796e-6, i.e. the current value already gives the real
hover point for the measured mass. Expected SITL hover thrust: installed 0.34985 (0.293 kg, 1932ab) x 0.2975/0.293 = 0.3552."""
import hashlib, json, os, re, shutil, sys

V3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(V3, "gz", "models", "starling2_tof")
OUT_ROOT = os.path.join(V3, "gz", "variants", "m3")
OUT = os.path.join(OUT_ROOT, "starling2_tof")
TOTAL_MASS = 0.2975
BASE_MASS = 0.2895
IMU_POSE = "0.0043 -0.0073 0.016 0 0 0"


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


src = open(os.path.join(SRC, "model.sdf")).read()
masses = [float(m) for m in re.findall(r"<mass>([^<]+)</mass>", src)]
base = re.search(r'<link name="base_link">\s*<inertial>\s*<mass>([^<]+)</mass>', src)
if not base or len(masses) != 5 or abs(float(base.group(1)) - 0.285) > 1e-9:
    sys.exit("make_model_m3: unexpected source model (base mass %s, masses %s) - nothing written" % (base and base.group(1), masses))
if abs(BASE_MASS + sum(masses) - masses[masses.index(float(base.group(1)))] - TOTAL_MASS) > 1e-9:
    sys.exit("make_model_m3: rotor masses %s do not add up to %.4f kg with base %.4f - nothing written" % (masses, TOTAL_MASS, BASE_MASS))
imu = re.search(r'(<sensor name="imu_sensor" type="imu">)(\s*)(?!<pose>)', src)
if not imu:
    sys.exit("make_model_m3: imu_sensor not found or already has a pose - nothing written")
kf = sorted(set(re.findall(r"<motorConstant>([^<]+)</motorConstant>", src)))
out = src[:base.start(1)] + "%g" % BASE_MASS + src[base.end(1):]
out = out.replace(imu.group(0), imu.group(1) + imu.group(2) + "<pose>%s</pose>%s" % (IMU_POSE, imu.group(2)), 1)
masses2 = [float(m) for m in re.findall(r"<mass>([^<]+)</mass>", out)]
if os.path.isdir(OUT):
    shutil.rmtree(OUT)                     # the variant dir is generated output only (never the current model)
shutil.copytree(SRC, OUT, ignore=shutil.ignore_patterns("model.sdf"))
open(os.path.join(OUT, "model.sdf"), "w").write(out)
info = {"variant": "m3", "source": os.path.relpath(SRC, V3), "source_sdf_sha256": sha(os.path.join(SRC, "model.sdf")),
        "sdf_sha256": sha(os.path.join(OUT, "model.sdf")), "mass_measured_kg": TOTAL_MASS,
        "mass_source": "user 2026-11-02: real vehicle with battery 297.5 g",
        "mass_sum_before": round(sum(masses), 6), "mass_sum_after": round(sum(masses2), 6), "base_link_mass": BASE_MASS,
        "imu_pose_flu": IMU_POSE, "motor_constant": kf,
        "motor_constant_check": "real ulog hover 0.356/0.357 at 0.2975 kg -> 1.7837e-6/1.7796e-6 (THR_MDL_FAC 0.9, omega=150+850u)",
        "expected_sitl_hover_thrust": 0.3552,
        "not_changed": ["inertia (box estimate)", "motor constants", "sensors other than the IMU pose", "airframe 4202"],
        "use": "WK_MODEL_DIR=%s (run_wk_instance.sh / wk_fly.sh)" % OUT_ROOT}
json.dump(info, open(os.path.join(OUT_ROOT, "VARIANT.json"), "w"), indent=1)
print(json.dumps(info, indent=1))
