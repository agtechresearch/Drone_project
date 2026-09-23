#!/usr/bin/env python3
"""marker_drift 단위테스트. 기체 없이 실행 가능.

  cd starling2_autonomy
  .venv/Scripts/python analysis/test_marker_drift.py      (Windows)
  .venv/bin/python     analysis/test_marker_drift.py      (Linux/mac)

합성 데이터로 검증한다:
  - 좌표계 규약(T -> M, 카메라 yaw 부호)
  - PnP 왕복 (투영 -> solvePnP -> 카메라 포즈 복원)
  - 렌더링한 AprilTag 이미지 -> pupil_apriltags 검출 -> 포즈 복원 (실제 검출기 경로)
  - NED -> M 정합 (heading / procrustes) 과 주입한 드리프트 복원
  - v14 CSV 로더, 표본 수 공식, 기본 배치, CLI analyze 전체 경로
"""

import math
import os
import sys
import tempfile
import traceback

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from marker_drift import drift, geometry, logs  # noqa: E402
from marker_drift.detect import DET_FIELDS, make_detector, pose_rows_for_frame  # noqa: E402
from marker_drift.intrinsics import Intrinsics  # noqa: E402
from marker_drift.markers import Layout, default_lab_layout  # noqa: E402

RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond), detail))
    print("[{}] {} {}".format("PASS" if cond else "FAIL", name, detail))


# ---------------------------------------------------------------- 공통 합성 도구

def cam_facing_wall(yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0):
    """벽(+y)을 정면으로 보는 카메라의 R_M_C 에 작은 자세를 더한다.
    yaw 양수 = +x(오른쪽) 쪽으로 돌아감 (geometry.camera_yaw_in_M 규약)."""
    R0 = geometry.R_M_T_UPRIGHT.copy()  # 카메라 x 오른쪽, y 아래, z 전방(+y) : 태그와 같은 축 배치
    R = geometry.rot_z(math.radians(-yaw_deg)) @ geometry.rot_x(math.radians(pitch_deg)) @ geometry.rot_y(math.radians(roll_deg)) @ R0
    return R


def hires_like():
    """hires 사양(120.4 x 93.5 deg)에 맞춘 가상 카메라 1280x800. fx ~ 369, fy ~ 377."""
    from marker_drift.coverage import intrinsics_from_fov
    return intrinsics_from_fov(120.4, 93.5, 1280, 800)


def project_tag(R_M_C, p_M_C, layout, tag_id, intr):
    """태그 코너의 픽셀 좌표(AprilTag 순서)와 카메라 프레임 깊이."""
    R_M_T, p_M_T = layout.pose(tag_id)
    return geometry.project_tag_corners(R_M_C, p_M_C, R_M_T, p_M_T, layout.size(tag_id), intr.K, intr.dist)


def canonical_tag_texture(tag_id, px_per_module=25):
    """aruco 로 만든 tag36h11 이미지를 AprilTag 규약 방향으로 돌리고 흰 여백 1모듈을 두른 텍스처.
    반환: (texture, 검은 테두리 외곽 코너 픽셀 [좌하, 우하, 우상, 좌상])"""
    import cv2
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    modules = 8  # 36h11: 6x6 데이터 + 테두리 1 = 8 모듈
    tag = cv2.aruco.generateImageMarker(d, tag_id, modules * px_per_module)
    tag = cv2.rotate(tag, cv2.ROTATE_180)
    pad = px_per_module
    tex = cv2.copyMakeBorder(tag, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    s = modules * px_per_module
    corners = np.array([[pad, pad + s], [pad + s, pad + s], [pad + s, pad], [pad, pad]], float)
    return tex, corners


def render_scene(R_M_C, p_M_C, layout, intr, W, H, tag_ids=None):
    import cv2
    img = np.full((H, W), 235, np.uint8)
    # 배경에 약한 격자 질감
    img[::40, :] = 200
    img[:, ::40] = 200
    for tid in (tag_ids or layout.ids()):
        px, depth = project_tag(R_M_C, p_M_C, layout, tid, intr)
        if (depth <= 0.05).any():
            continue
        if px[:, 0].min() < -50 or px[:, 0].max() > W + 50 or px[:, 1].min() < -50 or px[:, 1].max() > H + 50:
            continue
        tex, tex_corners = canonical_tag_texture(tid)
        Hm = cv2.getPerspectiveTransform(tex_corners.astype(np.float32), px.astype(np.float32))
        cv2.warpPerspective(tex, Hm, (W, H), dst=img, borderMode=cv2.BORDER_TRANSPARENT)
    return img


# ---------------------------------------------------------------- 테스트

def test_conventions():
    R = geometry.R_M_T_UPRIGHT
    check("T.x -> M.x", np.allclose(R @ [1, 0, 0], [1, 0, 0]))
    check("T.y(아래) -> -M.z", np.allclose(R @ [0, 1, 0], [0, 0, -1]))
    check("T.z(태그 안쪽) -> +M.y", np.allclose(R @ [0, 0, 1], [0, 1, 0]))
    check("R_M_T 는 회전행렬", np.allclose(R @ R.T, np.eye(3)) and abs(np.linalg.det(R) - 1) < 1e-12)

    obj = geometry.tag_object_points(0.2)
    check("코너 순서 [좌하, 우하, 우상, 좌상] (y 아래 양수)",
          np.allclose(obj, [[-0.1, 0.1, 0], [0.1, 0.1, 0], [0.1, -0.1, 0], [-0.1, -0.1, 0]]))

    check("정면 카메라 yaw = 0", abs(geometry.camera_yaw_in_M(cam_facing_wall(0))) < 1e-9)
    check("오른쪽(+x)으로 5도 돈 카메라 yaw = +5", abs(geometry.camera_yaw_in_M(cam_facing_wall(5.0)) - 5.0) < 1e-9)
    check("왼쪽으로 7도 돈 카메라 yaw = -7", abs(geometry.camera_yaw_in_M(cam_facing_wall(-7.0)) + 7.0) < 1e-9)
    pitch, roll = geometry.camera_pitch_roll_in_M(cam_facing_wall(0, 3.0, 0))
    check("pitch 부호(위를 보면 양수)", abs(pitch - 3.0) < 1e-6, "pitch={:.3f}".format(pitch))

    enu = geometry.ned_to_enu([[1, 2, -3]])
    check("NED (1,2,-3) -> ENU (2,1,3)", np.allclose(enu, [[2, 1, 3]]))
    v = geometry.compass_rotate([[0, 1]], 90)  # 북 -> 동
    check("compass_rotate(북, +90) = 동", np.allclose(v, [[1, 0]], atol=1e-12))
    check("heading_from_vector(1,0) = 90", abs(geometry.heading_from_vector(1, 0) - 90) < 1e-9)
    check("wrap_deg(190) = -170", abs(geometry.wrap_deg(190) + 170) < 1e-12)
    check("circular_mean([-179, 179]) ~ 180", abs(abs(geometry.circular_mean_deg([-179, 179])) - 180) < 1e-6)


def test_pnp_roundtrip():
    intr = hires_like()
    layout = Layout.default()
    cases = [
        ((0.30, -0.50, 0.60), (4.0, 1.5, -1.0), 0),
        ((2.70, -0.55, 0.62), (-3.0, -2.0, 0.5), 3),
        ((5.40, -0.48, 2.02), (1.0, 0.0, 0.0), 31),
        ((1.00, -1.20, 0.60), (0.0, 0.0, 0.0), 1),
    ]
    for pos, att, tid in cases:
        R_true = cam_facing_wall(*att)
        p_true = np.array(pos)
        px, _ = project_tag(R_true, p_true, layout, tid, intr)
        R_C_T, t_C_T, err = geometry.solve_tag_pose(px, layout.size(tid), intr.K, intr.dist)
        R_M_T, p_M_T = layout.pose(tid)
        R_est, p_est = geometry.camera_pose_from_tag(R_C_T, t_C_T, R_M_T, p_M_T)
        pos_err = np.linalg.norm(p_est - p_true)
        yaw_err = abs(geometry.camera_yaw_in_M(R_est) - geometry.camera_yaw_in_M(R_true))
        check("PnP 왕복 tag {} pos {}".format(tid, pos), pos_err < 2e-3 and yaw_err < 0.05 and err < 0.5,
              "pos_err={:.4f}m yaw_err={:.3f}deg reproj={:.3f}px".format(pos_err, yaw_err, err))


def test_rendered_detection():
    import cv2
    intr = hires_like()
    layout = Layout.default()

    tex, tc = canonical_tag_texture(0)
    det = make_detector()
    r = det.detect(tex)
    ok = len(r) == 1 and r[0].tag_id == 0
    c = r[0].corners if ok else None
    order_ok = ok and np.allclose(c, tc, atol=1.5)
    check("텍스처 코너 순서 = AprilTag 규약 [좌하,우하,우상,좌상]", order_ok,
          "" if order_ok else "corners={}".format(None if c is None else c.round(1).tolist()))

    scenes = [
        ((0.30, -0.50, 0.60), (4.0, 1.5, -1.0)),
        ((2.50, -0.50, 0.60), (0.0, 0.0, 0.0)),
        ((5.25, -0.55, 2.00), (-2.0, 0.5, 0.0)),
    ]
    for pos, att in scenes:
        R_true = cam_facing_wall(*att)
        p_true = np.array(pos)
        img = render_scene(R_true, p_true, layout, intr, 1280, 800)
        dets = det.detect(img)
        rows = pose_rows_for_frame(dets, 0, 0.0, intr, layout)
        rows = [r for r in rows if r["tag_in_layout"] == 1]
        if not rows:
            check("렌더 검출 {}".format(pos), False, "검출 0")
            continue
        P = np.array([[r["cam_x"], r["cam_y"], r["cam_z"]] for r in rows])
        yaws = np.array([r["cam_yaw_deg"] for r in rows])
        pos_err = np.linalg.norm(P.mean(axis=0) - p_true)
        yaw_err = abs(geometry.wrap_deg(geometry.circular_mean_deg(yaws) - geometry.camera_yaw_in_M(R_true)))
        spread = float(np.max(np.linalg.norm(P - P.mean(axis=0), axis=1))) if len(P) > 1 else 0.0
        check("렌더 검출 -> 태그별 포즈 평균 {} ({}태그, 7 cm)".format(pos, len(rows)), pos_err < 0.03 and yaw_err < 3.0,
              "pos_err={:.4f}m yaw_err={:.2f}deg spread={:.4f}m ids={}".format(
                  pos_err, yaw_err, spread, [r["tag_id"] for r in rows]))
        if len(rows) >= 2:
            j = rows[0]
            jp = np.array([j["joint_x"], j["joint_y"], j["joint_z"]])
            j_pos_err = np.linalg.norm(jp - p_true)
            j_yaw_err = abs(geometry.wrap_deg(j["joint_yaw_deg"] - geometry.camera_yaw_in_M(R_true)))
            check("렌더 검출 -> 한 몸 PnP {} ({}태그)".format(pos, j["joint_n_tags"]), j_pos_err < 0.01 and j_yaw_err < 0.5,
                  "pos_err={:.4f}m yaw_err={:.2f}deg reproj={:.2f}px".format(j_pos_err, j_yaw_err, j["joint_reproj_px"]))


def test_joint_vs_single_small_tags():
    """7 cm 태그, 정면 근처 yaw 에서 태그 1개 PnP 의 yaw 오차와 한 몸 PnP 의 yaw 오차 비교."""
    intr = hires_like()
    layout = Layout.default()
    det = make_detector()
    single_errs, joint_errs = [], []
    for yaw in (-4.0, -1.5, 0.0, 1.5, 4.0):
        R_true = cam_facing_wall(yaw, 0.5, -0.5)
        p_true = np.array([0.5, -0.5, 0.6])  # 0.5 m 간격이면 태그 0·1·2 가 보임
        img = render_scene(R_true, p_true, layout, intr, 1280, 800)
        rows = [r for r in pose_rows_for_frame(det.detect(img), 0, 0.0, intr, layout) if r["tag_in_layout"] == 1]
        if len(rows) < 2:
            check("joint vs single: 두 태그 검출 (yaw {})".format(yaw), False, "n={}".format(len(rows)))
            continue
        for r in rows:
            single_errs.append(abs(geometry.wrap_deg(r["cam_yaw_deg"] - yaw)))
        joint_errs.append(abs(geometry.wrap_deg(rows[0]["joint_yaw_deg"] - yaw)))
    check("한 몸 PnP yaw 오차 최대 < 0.5 deg", joint_errs and max(joint_errs) < 0.5,
          "joint max={:.2f} mean={:.2f} | single max={:.2f} mean={:.2f} (deg)".format(
              max(joint_errs) if joint_errs else float("nan"), np.mean(joint_errs) if joint_errs else float("nan"),
              max(single_errs) if single_errs else float("nan"), np.mean(single_errs) if single_errs else float("nan")))
    check("한 몸 PnP 가 태그별 PnP 보다 yaw 가 정확", joint_errs and single_errs and np.mean(joint_errs) <= np.mean(single_errs))


def synth_flight(delta_true=-90.0, trans_true=(0.8, -1.3, 0.05), drift_per_m=(0.01, -0.02, 0.005),
                 noise_m=0.0, noise_yaw=0.0, fps=20.0, seed=0):
    """합성 비행: 호버 10 s -> 우측 5.5 m (18 s) -> 도착 호버 5 s. 카메라 M 진실 + VIO(L, NED) 로그."""
    rng = np.random.default_rng(seed)
    t = np.arange(0, 33.0, 1.0 / fps)
    x = np.piecewise(t, [t < 10, (t >= 10) & (t < 28), t >= 28], [0.0, lambda u: (u - 10) / 18 * 5.5, 5.5])
    truth = np.column_stack([x, np.full_like(t, -0.5), np.full_like(t, 0.6)])
    yaw_M = np.zeros_like(t)
    dist = x - x[0]
    e = np.column_stack([dist * drift_per_m[0], dist * drift_per_m[1], dist * drift_per_m[2]])
    vio_M = truth + e
    # M -> L(NED)
    tr = np.asarray(trans_true)
    enu_xy = geometry.compass_rotate(vio_M[:, :2] - tr[:2], -delta_true)
    enu_z = vio_M[:, 2] - tr[2]
    ned = np.column_stack([enu_xy[:, 1], enu_xy[:, 0], -enu_z])
    yaw_ned = geometry.wrap_deg(yaw_M - delta_true)
    cmd_M = truth.copy()  # 명령 = 진실 경로 (경로 이탈 0)
    cmd_xy = geometry.compass_rotate(cmd_M[:, :2] - tr[:2], -delta_true)
    cmd_ned = np.column_stack([cmd_xy[:, 1], cmd_xy[:, 0], -(cmd_M[:, 2] - tr[2])])

    import pandas as pd
    frames = pd.DataFrame({
        "frame": np.arange(len(t)), "t": t, "n_tags": 1,
        "x": truth[:, 0] + rng.normal(0, noise_m, len(t)),
        "y": truth[:, 1] + rng.normal(0, noise_m, len(t)),
        "z": truth[:, 2] + rng.normal(0, noise_m, len(t)),
        "yaw_deg": yaw_M + rng.normal(0, noise_yaw, len(t)),
        "spread_m": 0.0, "reproj_px": 0.3, "tags": "0",
    })
    log = pd.DataFrame({"t": t + 100.0, "n": ned[:, 0], "e": ned[:, 1], "d": ned[:, 2], "yaw_deg": yaw_ned,
                        "cmd_n": cmd_ned[:, 0], "cmd_e": cmd_ned[:, 1], "cmd_d": cmd_ned[:, 2],
                        "stage": np.where(t < 10, "hover_start", np.where(t < 28, "right_5p5", "hover_end"))})
    return frames, log, e


def test_alignment_and_metrics():
    frames, log, e_true = synth_flight()
    ts = drift.build_timeseries(frames, log, t_offset=100.0)
    check("로그 보간 후 NaN 없음", ts[["n", "e", "d", "yaw_ned_deg"]].notna().all().all())

    al = drift.align_L_to_M(ts, (0.0, 10.0), method="heading")
    check("heading 정합 delta = -90", abs(geometry.wrap_deg(al.delta_deg + 90)) < 1e-6, "delta={:.4f}".format(al.delta_deg))
    check("heading 정합 trans", np.allclose(al.trans, [0.8, -1.3, 0.05], atol=1e-6), "trans={}".format(al.trans.round(4)))
    ts_a = drift.apply_alignment(ts, al)
    E_true = e_true[-1]
    m = drift.compute_metrics(ts_a, (0.0, 10.0), (28.0, 33.0), move_window=(10.0, 28.0))
    check("최종 드리프트 E_x 복원", abs(m["E_x"] - E_true[0]) < 1e-6, "E_x={:.4f} true={:.4f}".format(m["E_x"], E_true[0]))
    check("최종 드리프트 E_y 복원", abs(m["E_y"] - E_true[1]) < 1e-6, "E_y={:.4f} true={:.4f}".format(m["E_y"], E_true[1]))
    check("최종 드리프트 E_z 복원", abs(m["E_z"] - E_true[2]) < 1e-6, "E_z={:.4f} true={:.4f}".format(m["E_z"], E_true[2]))
    check("드리프트율 = |E_h| / 5.5", abs(m["rate_h_per_m"] - math.hypot(*E_true[:2]) / 5.5) < 1e-6)
    check("경로 이탈 0 (명령 = 진실)", m["dev_rms_h_m"] < 1e-6, "dev_rms={:.2e}".format(m["dev_rms_h_m"]))
    check("호버 드리프트 0", m["hover_std_x_m"] < 1e-9 and abs(m["hover_slope_x_m_per_s"]) < 1e-9)
    check("호버 잡음 0 -> 허용오차 하한 1도", m["align_tol_deg_suggested"] == 1.0)

    frames0, log0, _ = synth_flight(drift_per_m=(0.0, 0.0, 0.0))
    ts0 = drift.build_timeseries(frames0, log0, t_offset=100.0)
    al2 = drift.align_L_to_M(ts0, (10.0, 28.0), method="procrustes")
    check("procrustes 정합 delta = -90 (드리프트 없는 이동 구간)", abs(geometry.wrap_deg(al2.delta_deg + 90)) < 1e-6, "delta={:.4f}".format(al2.delta_deg))
    al3 = drift.align_L_to_M(ts, (10.0, 28.0), method="procrustes")
    exp = -90 + math.degrees(math.atan2(-0.02, 1.01))
    check("procrustes 는 드리프트 회전 성분을 흡수 (문서화된 한계)", abs(geometry.wrap_deg(al3.delta_deg - exp)) < 0.05,
          "delta={:.3f} expected={:.3f}".format(al3.delta_deg, exp))

    try:
        drift.align_L_to_M(ts, (0.0, 10.0), method="procrustes")
        check("procrustes 는 정지 구간에서 거부", False)
    except ValueError:
        check("procrustes 는 정지 구간에서 거부", True)

    # 잡음이 있는 경우: 잡음 추정과 허용오차
    frames_n, log_n, _ = synth_flight(noise_m=0.004, noise_yaw=0.5, seed=1)
    ts_n = drift.apply_alignment(drift.build_timeseries(frames_n, log_n, 100.0),
                                 drift.align_L_to_M(drift.build_timeseries(frames_n, log_n, 100.0), (0.0, 10.0)))
    m_n = drift.compute_metrics(ts_n, (0.0, 10.0), (28.0, 33.0))
    check("프레임 간 잡음 추정 ~ 4 mm", 0.0025 < m_n["noise_x_m"] < 0.0055, "noise_x={:.4f}".format(m_n["noise_x_m"]))
    check("yaw 잡음 추정 ~ 0.5 deg", 0.3 < m_n["noise_yaw_deg"] < 0.7, "noise_yaw={:.3f}".format(m_n["noise_yaw_deg"]))
    check("허용오차 제안 = 3 sigma", abs(m_n["align_tol_deg_suggested"] - 3 * m_n["noise_yaw_deg"]) < 1e-9)
    check("잡음 있어도 E_h 오차 < 1 cm", abs(m_n["E_h"] - math.hypot(*e_true[-1][:2])) < 0.01,
          "E_h={:.4f} true={:.4f}".format(m_n["E_h"], math.hypot(*e_true[-1][:2])))


def test_per_frame_pose():
    import pandas as pd
    det = pd.DataFrame([
        {"frame": 0, "t": 0.0, "tag_id": 0, "cam_x": 0.00, "cam_y": -0.5, "cam_z": 0.6, "cam_yaw_deg": 1.0, "reproj_px": 0.2, "tag_in_layout": 1},
        {"frame": 0, "t": 0.0, "tag_id": 1, "cam_x": 0.02, "cam_y": -0.5, "cam_z": 0.6, "cam_yaw_deg": -1.0, "reproj_px": 0.4, "tag_in_layout": 1},
        {"frame": 1, "t": 0.05, "tag_id": 99, "cam_x": "", "cam_y": "", "cam_z": "", "cam_yaw_deg": "", "reproj_px": "", "tag_in_layout": 0},
        {"frame": 2, "t": 0.10, "tag_id": 1, "cam_x": 0.05, "cam_y": -0.5, "cam_z": 0.6, "cam_yaw_deg": 0.5, "reproj_px": 0.3, "tag_in_layout": 1},
    ])
    f = drift.per_frame_pose(det)
    check("프레임 집계 행 수 (배치 외 태그 제외)", len(f) == 2 and f["frame"].tolist() == [0, 2])
    check("두 태그 평균 위치", abs(f.iloc[0]["x"] - 0.01) < 1e-12 and f.iloc[0]["n_tags"] == 2)
    check("태그 간 편차 spread = 2 cm", abs(f.iloc[0]["spread_m"] - 0.02) < 1e-9)
    check("원형 평균 yaw = 0", abs(f.iloc[0]["yaw_deg"]) < 1e-9)


def test_v14_loader():
    import pandas as pd
    p = "voxl_px4_vehicle_local_position_"
    rows = []
    for i in range(5):
        rows.append({
            "unix_time": 1.7e9 + i, "wallclock": "00:00:0{}".format(i), "mono_time": 1000.0 + i,
            "stage": "hover" if i < 2 else "right", "phase": "MOVE",
            "target_rel_n": 0.0, "target_rel_e": 0.1 * i, "target_rel_u": 0.6, "target_yaw_deg": 90.0,
            p + "ts_ms": (1000.0 + i) * 1000.0, p + "x": 5.0, p + "y": 3.0 + 0.1 * i, p + "z": -0.6,
            p + "rel_x": 0.0, p + "rel_y": 0.1 * i, p + "rel_z": 0.0, p + "yaw_deg": 90.0,
            "mav_rel_n": 0.0, "mav_rel_e": 0.1 * i, "mav_rel_u": 0.6,
        })
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "v14.csv")
        pd.DataFrame(rows).to_csv(path, index=False)
        df = logs.load_v14_csv(path)
    check("v14 로더 행 수", len(df) == 5)
    check("v14 t = ts_ms/1000", abs(df["t"].iloc[0] - 1000.0) < 1e-9)
    check("v14 위치 = rel 파이프값", abs(df["e"].iloc[3] - 0.3) < 1e-9 and abs(df["d"].iloc[0]) < 1e-9)
    check("v14 명령 d = -target_rel_u", abs(df["cmd_d"].iloc[0] + 0.6) < 1e-9)
    w = logs.stage_windows(df, "hover")
    check("stage 창 추출", len(w) == 1 and abs(w[0][0] - 1000.0) < 1e-9, "windows={}".format(w))


def test_layout_and_stats():
    lay = Layout(default_lab_layout())
    check("기본 배치 24개 마커 (0.5 m 간격)", len(lay) == 24)
    check("하단열 ID 0~11, 상단열 20~31", lay.ids() == list(range(12)) + list(range(20, 32)))
    check("마커 11 은 x=5.5, z=0.6", lay.markers[11]["x"] == 5.5 and lay.markers[11]["z"] == 0.6)
    check("마커 31 은 x=5.5, z=2.0", lay.markers[31]["x"] == 5.5 and lay.markers[31]["z"] == 2.0)
    lay1 = Layout(default_lab_layout(spacing_m=1.0))
    check("1 m 간격이면 7개/열, 상단열 10~16", lay1.ids() == [0, 1, 2, 3, 4, 5, 6, 10, 11, 12, 13, 14, 15, 16])
    yml = os.path.join(HERE, "marker_drift", "layouts", "lab_default.yaml")
    lay2 = Layout.load(yml)
    check("layouts/lab_default.yaml 과 기본 배치 일치",
          all(lay2.markers[k]["x"] == lay.markers[k]["x"] and lay2.markers[k]["z"] == lay.markers[k]["z"] for k in lay.ids()))

    n = drift.sample_size_two_group(sd=0.03, delta=0.03)
    check("표본 수: sd = delta -> 16/그룹", n == 16, "n={}".format(n))
    n2 = drift.sample_size_two_group(sd=0.015, delta=0.03)
    check("표본 수: sd = delta/2 -> 4/그룹", n2 == 4, "n={}".format(n2))


def test_cli_analyze_end_to_end():
    import pandas as pd
    from marker_drift.__main__ import main
    frames, log, e_true = synth_flight(noise_m=0.003, noise_yaw=0.3, seed=2)
    with tempfile.TemporaryDirectory() as td:
        det_path = os.path.join(td, "det.csv")
        det_rows = []
        for _, r in frames.iterrows():
            det_rows.append({"frame": int(r["frame"]), "t": r["t"], "tag_id": 0, "hamming": 0, "decision_margin": 50,
                             **{"c{}{}".format(i, a): 0.0 for i in range(4) for a in "xy"},
                             "reproj_px": 0.3, "tag_dist_m": 0.5, "cam_x": r["x"], "cam_y": r["y"], "cam_z": r["z"],
                             "cam_yaw_deg": r["yaw_deg"], "cam_pitch_deg": 0.0, "cam_roll_deg": 0.0, "tag_in_layout": 1})
        pd.DataFrame(det_rows, columns=DET_FIELDS).to_csv(det_path, index=False)
        log_path = os.path.join(td, "log.csv")
        log.to_csv(log_path, index=False)
        out_dir = os.path.join(td, "out")
        try:
            main(["analyze", "--detections", det_path, "--log", log_path, "--log-format", "generic",
                  "--t-offset", "100", "--hover", "0", "10", "--move", "10", "28", "--end", "28", "33",
                  "--name", "synthA1", "--condition", "A", "--out-dir", out_dir])
            ok = True
        except SystemExit as ex:
            ok = ex.code in (None, 0)
        except Exception:
            traceback.print_exc()
            ok = False
        check("CLI analyze 실행", ok)
        summ = os.path.join(out_dir, "synthA1_summary.csv")
        if os.path.exists(summ):
            s = pd.read_csv(summ).iloc[0]
            check("CLI 요약 E_h ~ 진실", abs(s["E_h"] - math.hypot(*e_true[-1][:2])) < 0.01, "E_h={:.4f}".format(s["E_h"]))
            check("CLI 산출물(시계열·그림) 존재",
                  os.path.exists(os.path.join(out_dir, "synthA1_timeseries.csv")) and os.path.exists(os.path.join(out_dir, "synthA1.png")))
            # compare 는 조건 2개 이상 필요: 같은 요약을 B 로 복제해 경로만 검증
            s2 = pd.read_csv(summ)
            s2["condition"] = "B"
            s2["E_h"] += 0.02
            b_path = os.path.join(out_dir, "synthB1_summary.csv")
            s2.to_csv(b_path, index=False)
            s3 = pd.read_csv(summ); s3["E_h"] += 0.001; a2 = os.path.join(out_dir, "synthA2_summary.csv"); s3.to_csv(a2, index=False)
            s4 = s2.copy(); s4["E_h"] += 0.001; b2 = os.path.join(out_dir, "synthB2_summary.csv"); s4.to_csv(b2, index=False)
            try:
                main(["compare", summ, a2, b_path, b2, "--out-dir", os.path.join(out_dir, "cmp")])
                ok2 = os.path.exists(os.path.join(out_dir, "cmp", "compare_table.csv"))
            except Exception:
                traceback.print_exc()
                ok2 = False
            check("CLI compare 실행", ok2)
        else:
            check("CLI 요약 파일 생성", False)


def main():
    tests = [test_conventions, test_pnp_roundtrip, test_rendered_detection, test_joint_vs_single_small_tags, test_alignment_and_metrics,
             test_per_frame_pose, test_v14_loader, test_layout_and_stats, test_cli_analyze_end_to_end]
    for t in tests:
        print("\n== {} ==".format(t.__name__))
        try:
            t()
        except Exception:
            traceback.print_exc()
            check(t.__name__ + " (exception)", False)
    n_pass = sum(1 for _, ok, _ in RESULTS if ok)
    print("\n{}/{} passed".format(n_pass, len(RESULTS)))
    return 0 if n_pass == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
