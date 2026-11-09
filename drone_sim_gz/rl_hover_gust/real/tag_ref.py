#!/usr/bin/env python3
"""Marker reference: vehicle BODY pose in the AprilTag frame from voxl-tag-detector records (2026-11-03, plan
plans/2026-11-03_real_marker_validation_plan.md section 1). Python 3.6, standard library only (runs on the PC and on the VOXL).
  tag_ref.py <tag_detections.jsonl> --extrinsics <extrinsics.conf> --tag-size 0.100 [--cam tracking_front] [--id 0]
             [--out <ref.csv>]
Input: mpa_parse.py output, one JSON per record {"t": s, "dets": [{"id", "t_cam", "R", "size_m", "cam"}]}.
  t_cam = T_tag_wrt_cam (tag centre in camera coordinates, camera x right / y down / z forward), R = R_tag_to_cam
  (tag axes -> camera axes). Tag frame (ModalAI / AprilTag): origin at the centre, x right and y down as the printed tag is
  seen by the viewer, z into the tag (away from the viewer).
Transforms: T_A_B maps B coordinates into A (p_A = R p_B + t). From the detection, T_cam_tag = (R, t_cam * s), with
  s = true tag size / size_m in the record (the detector assumes size_m from tag_locations.conf; the scale applies only to the
  camera translation, never to the extrinsics). Extrinsics: T_body_cam = T_body_imu @ T_imu_cam from extrinsics.conf, where
  RPY_parent_to_child gives R_parent_child = Rx(roll) Ry(pitch) Rz(yaw) (checked: tracking_front [0, 90, 90] maps camera z
  (forward) to body x, camera x to body y, camera y to body z). Then T_tag_body = inv(T_cam_tag) @ inv(T_body_cam).
Output per detection (CSV): t, cam, id, x_m (body origin right of the tag centre as seen facing the tag; vehicle facing a
  north wall -> PX4 NED east), y_m (below the tag centre), z_m (negative = in front of the tag), dist_m = -z_m,
  height_above_tag_m = -y_m, yaw / pitch / roll (deg, ZYX Euler of the body relative to squarely facing the tag: yaw + = nose
  to the right, pitch + = nose up, roll + = right side down), scale. Detections of other cameras / ids are counted and skipped.
NOT YET CHECKED AGAINST A REAL VOXL RECORDING (2026-11-03): the axis conventions above are the documented ones; the bench
test (R1) confirms the signs with known displacements."""
import argparse, json, math, re, sys


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def rx(a):
    c, s = math.cos(a), math.sin(a); return [[1, 0, 0], [0, c, -s], [0, s, c]]


def ry(a):
    c, s = math.cos(a), math.sin(a); return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rz(a):
    c, s = math.cos(a), math.sin(a); return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def rpy_to_R(rpy_deg):
    r, p, y = [math.radians(v) for v in rpy_deg]
    return mat_mul(mat_mul(rx(r), ry(p)), rz(y))


class Tf(object):
    """p_A = R p_B + t"""
    def __init__(self, R, t):
        self.R = [list(map(float, row)) for row in R]; self.t = list(map(float, t))

    def __matmul__(self, o):
        return Tf(mat_mul(self.R, o.R), [a + b for a, b in zip(mat_vec(self.R, o.t), self.t)])

    def inv(self):
        Rt = transpose(self.R); return Tf(Rt, [-v for v in mat_vec(Rt, self.t)])

    def apply(self, p):
        return [a + b for a, b in zip(mat_vec(self.R, p), self.t)]


def load_extrinsics(path):
    txt = open(path).read()
    txt = re.sub(r"/\*.*?\*/", "", txt, flags=re.S)                   # the ModalAI file starts with a comment block
    d = json.loads(txt); E = {}
    for e in d["extrinsics"]:
        E[(e["parent"], e["child"])] = Tf(rpy_to_R(e["RPY_parent_to_child"]), e["T_child_wrt_parent"])
    return E


def body_from_cam(E, cam):
    """T_body_cam via imu_apps (body -> imu_apps -> camera)"""
    if ("body", cam) in E:
        return E[("body", cam)]
    return E[("body", "imu_apps")] @ E[("imu_apps", cam)]


def tag_from_body(T_body_cam, R_tag_to_cam, t_cam, scale):
    T_cam_tag = Tf(R_tag_to_cam, [v * scale for v in t_cam])
    return T_cam_tag.inv() @ T_body_cam.inv()


FACE = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]          # body axes (columns) in tag coordinates for a vehicle squarely facing the tag


def pose_fields(T):
    """body origin and orientation in the tag frame -> reported quantities. Orientation = ZYX Euler angles of the body relative
    to 'squarely facing the tag' (FACE): Rb = FACE^T R_tag_body = Rz(yaw) Ry(pitch) Rx(roll), body FRD axes; yaw positive
    = nose turned to the right, pitch positive = nose up, roll positive = right side down."""
    x, y, z = T.t
    Rb = mat_mul(transpose(FACE), T.R)
    yaw = math.degrees(math.atan2(Rb[1][0], Rb[0][0]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -Rb[2][0]))))
    roll = math.degrees(math.atan2(Rb[2][1], Rb[2][2]))
    return {"x_m": x, "y_m": y, "z_m": z, "dist_m": -z, "height_above_tag_m": -y, "yaw_deg": yaw, "pitch_deg": pitch, "roll_deg": roll}


def main(a):
    E = load_extrinsics(a.extrinsics); Tbc = {}
    n = {"records": 0, "used": 0, "other_cam": 0, "other_id": 0}; others = {}
    out = open(a.out, "w") if a.out else sys.stdout
    out.write("t,cam,id,x_m,y_m,z_m,dist_m,height_above_tag_m,yaw_deg,roll_deg,pitch_deg,scale\n")
    for line in open(a.jsonl):
        if not line.strip():
            continue
        rec = json.loads(line); n["records"] += 1
        for d in rec["dets"]:
            cam = d.get("cam") or a.cam
            if cam != a.cam:
                n["other_cam"] += 1; others[cam] = others.get(cam, 0) + 1; continue
            if d["id"] != a.id:
                n["other_id"] += 1; continue
            if cam not in Tbc:
                Tbc[cam] = body_from_cam(E, cam)
            scale = a.tag_size / float(d.get("size_m") or a.tag_size)
            f = pose_fields(tag_from_body(Tbc[cam], d["R"], d["t_cam"], scale))
            out.write("%.6f,%s,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.4f\n" % (rec["t"], cam, d["id"], f["x_m"], f["y_m"], f["z_m"],
                      f["dist_m"], f["height_above_tag_m"], f["yaw_deg"], f["roll_deg"], f["pitch_deg"], scale))
            n["used"] += 1
    if a.out:
        out.close()
    sys.stderr.write("tag_ref: %s, other cameras %s\n" % (n, others or "-"))
    return 0 if n["used"] else 3


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl"); ap.add_argument("--extrinsics", required=True); ap.add_argument("--tag-size", type=float, required=True)
    ap.add_argument("--cam", default="tracking_front"); ap.add_argument("--id", type=int, default=0); ap.add_argument("--out")
    sys.exit(main(ap.parse_args()))
