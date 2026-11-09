#!/usr/bin/env python3
"""Corrected copy "m2" of the wall-keep model starling2_tof (2026-11-01, codex sim-total-mass-consistency / sim-imu-offset-consistency;
user: keep the current state as it is, make a new one to test, restorable, no effect on the rest).
  python3 make_model_m2.py            -> v3_marker/gz/variants/m2/starling2_tof/ (model.sdf, model.config, meshes/) + VARIANT.json
The CURRENT model (v3_marker/gz/models/starling2_tof, installed in the PX4 tree as starling2_tof) is only read, never written. The
variant keeps the SAME model name, so a SITL instance uses it only when run_wk_instance.sh gets WK_MODEL_DIR=<this variants dir>
(put first on that instance's GZ_SIM_RESOURCE_PATH; PX4 spawns "starling2_tof/model.sdf" by resource path, GZBridge.cpp:75).
Changes, nothing else:
  1. mass: links summed 0.285 (base_link) + 4 x 0.002 (rotors) = 0.293 kg, but the motor constant was scaled for m = 0.285
     (make_gz_model.py: m*g/4 = 0.699 N at the hover command) -> base_link 0.277 so the vehicle totals 0.285 kg. Inertia kept (box
     estimate, 2.8 % mass change not propagated - stated in VARIANT.json).
  2. IMU position: imu_sensor had no pose (= base_link origin, taken as the centre of mass) while the airframe sets the real
     EKF2_IMU_POS_X/Y/Z = 0.0043 / 0.0073 / -0.016 m (body FRD, handoff_2026-10-27) -> the sensor is placed there: gz FLU
     x = 0.0043, y = -0.0073, z = +0.016. EKF2_EV_POS stays 0 (the odometry publisher reports base_link origin = CoM)."""
import hashlib, json, os, re, shutil, sys

V3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(V3, "gz", "models", "starling2_tof")
OUT_ROOT = os.path.join(V3, "gz", "variants", "m2")
OUT = os.path.join(OUT_ROOT, "starling2_tof")
BASE_MASS = 0.277
IMU_POSE = "0.0043 -0.0073 0.016 0 0 0"


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


src = open(os.path.join(SRC, "model.sdf")).read()
masses = [float(m) for m in re.findall(r"<mass>([^<]+)</mass>", src)]
base = re.search(r'<link name="base_link">\s*<inertial>\s*<mass>([^<]+)</mass>', src)
if not base or len(masses) != 5 or abs(float(base.group(1)) - 0.285) > 1e-9:
    sys.exit("make_model_m2: unexpected source model (base mass %s, masses %s) - nothing written" % (base and base.group(1), masses))
imu = re.search(r'(<sensor name="imu_sensor" type="imu">)(\s*)(?!<pose>)', src)
if not imu:
    sys.exit("make_model_m2: imu_sensor not found or already has a pose - nothing written")
out = src[:base.start(1)] + "%g" % BASE_MASS + src[base.end(1):]
out = out.replace(imu.group(0), imu.group(1) + imu.group(2) + "<pose>%s</pose>%s" % (IMU_POSE, imu.group(2)), 1)
masses2 = [float(m) for m in re.findall(r"<mass>([^<]+)</mass>", out)]
if os.path.isdir(OUT):
    shutil.rmtree(OUT)                     # the variant dir is generated output only (never the current model)
shutil.copytree(SRC, OUT, ignore=shutil.ignore_patterns("model.sdf"))
open(os.path.join(OUT, "model.sdf"), "w").write(out)
info = {"variant": "m2", "source": os.path.relpath(SRC, V3), "source_sdf_sha256": sha(os.path.join(SRC, "model.sdf")),
        "sdf_sha256": sha(os.path.join(OUT, "model.sdf")), "mass_sum_before": round(sum(masses), 6),
        "mass_sum_after": round(sum(masses2), 6), "base_link_mass": BASE_MASS, "imu_pose_flu": IMU_POSE,
        "not_changed": ["inertia (box estimate)", "motor constants", "sensors other than the IMU pose", "airframe 4202"],
        "use": "WK_MODEL_DIR=%s (run_wk_instance.sh / wk_fly.sh)" % OUT_ROOT}
json.dump(info, open(os.path.join(OUT_ROOT, "VARIANT.json"), "w"), indent=1)
print(json.dumps(info, indent=1))
