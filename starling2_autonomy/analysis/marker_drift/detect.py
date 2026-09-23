"""영상 -> 태그 검출 + 카메라 포즈 CSV.

입력
  --video FILE          동영상. 프레임 시각 = t0 + idx / fps (fps 는 파일 메타 또는 --fps).
  --frames-dir DIR      이미지 폴더(이름순). 시각은 --timestamps 로.
  --timestamps CSV      프레임 인덱스(또는 파일명) -> 시각 매핑. --ts-col, --ts-scale, --ts-index-col.
  --intrinsics FILE     intrinsics.py 형식.
  --layout FILE         마커 배치(없으면 기본 배치).

출력 CSV (한 행 = 한 프레임의 한 태그)
  frame, t, tag_id, hamming, decision_margin,
  c0x, c0y, ..., c3x, c3y            코너 픽셀(AprilTag 순서)
  reproj_px                          PnP 재투영 RMS
  tag_dist_m                         카메라-태그 거리
  cam_x, cam_y, cam_z                카메라 위치 (M 프레임)
  cam_yaw_deg, cam_pitch_deg, cam_roll_deg
  tag_in_layout                      배치에 없는 ID 면 0 (포즈 계산 안 함)

프레임 단위 집계(여러 태그 평균)는 drift.py 가 한다.
"""

import csv
import glob
import os

import numpy as np

from marker_drift import geometry
from marker_drift.intrinsics import Intrinsics
from marker_drift.markers import Layout

DET_FIELDS = ["frame", "t", "tag_id", "hamming", "decision_margin",
              "c0x", "c0y", "c1x", "c1y", "c2x", "c2y", "c3x", "c3y",
              "reproj_px", "tag_dist_m", "cam_x", "cam_y", "cam_z",
              "cam_yaw_deg", "cam_pitch_deg", "cam_roll_deg", "tag_in_layout"]


def make_detector(family="tag36h11", quad_decimate=1.0, nthreads=2):
    from pupil_apriltags import Detector
    return Detector(families=family, nthreads=nthreads, quad_decimate=quad_decimate,
                    quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)


def detect_gray(detector, gray):
    """회색조 이미지 -> pupil_apriltags Detection 리스트."""
    return detector.detect(gray)


def pose_rows_for_frame(detections, frame_idx, t, intr, layout, min_margin=20.0, max_hamming=0):
    """한 프레임의 검출 목록 -> CSV 행 리스트."""
    rows = []
    for det in detections:
        if det.hamming > max_hamming or det.decision_margin < min_margin:
            continue
        corners = np.asarray(det.corners, float).reshape(4, 2)
        row = {
            "frame": frame_idx, "t": t, "tag_id": int(det.tag_id),
            "hamming": int(det.hamming), "decision_margin": float(det.decision_margin),
        }
        for i in range(4):
            row["c{}x".format(i)] = corners[i, 0]
            row["c{}y".format(i)] = corners[i, 1]
        if det.tag_id in layout:
            R_C_T, t_C_T, err = geometry.solve_tag_pose(corners, layout.size(det.tag_id), intr.K, intr.dist, intr.fisheye)
            R_M_T, p_M_T = layout.pose(det.tag_id)
            R_M_C, p_M_C = geometry.camera_pose_from_tag(R_C_T, t_C_T, R_M_T, p_M_T)
            pitch, roll = geometry.camera_pitch_roll_in_M(R_M_C)
            row.update({
                "reproj_px": err, "tag_dist_m": float(np.linalg.norm(t_C_T)),
                "cam_x": p_M_C[0], "cam_y": p_M_C[1], "cam_z": p_M_C[2],
                "cam_yaw_deg": geometry.camera_yaw_in_M(R_M_C),
                "cam_pitch_deg": pitch, "cam_roll_deg": roll, "tag_in_layout": 1,
            })
        else:
            row.update({k: "" for k in ("reproj_px", "tag_dist_m", "cam_x", "cam_y", "cam_z",
                                        "cam_yaw_deg", "cam_pitch_deg", "cam_roll_deg")})
            row["tag_in_layout"] = 0
        rows.append(row)
    return rows


def load_timestamps(path, index_col=None, ts_col="timestamp_ns", ts_scale=1e-9):
    """타임스탬프 CSV -> {index 또는 파일명: 초}. index_col 이 None 이면 행 순서를 인덱스로 본다."""
    import pandas as pd
    df = pd.read_csv(path)
    ts = pd.to_numeric(df[ts_col], errors="coerce") * float(ts_scale)
    if index_col and index_col in df.columns:
        keys = df[index_col].tolist()
    else:
        keys = list(range(len(df)))
    return {k: float(v) for k, v in zip(keys, ts)}


def iter_video(path, max_frames=None, stride=1):
    import cv2
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise IOError("영상을 열 수 없음: {}".format(path))
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    idx = 0
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % stride == 0:
            yield idx, fps, frame
            n += 1
            if max_frames and n >= max_frames:
                break
        idx += 1
    cap.release()


def iter_frames_dir(path, max_frames=None, stride=1):
    import cv2
    files = sorted(glob.glob(os.path.join(path, "*")))
    files = [f for f in files if f.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".pgm", ".tif", ".tiff"))]
    n = 0
    for idx, f in enumerate(files):
        if idx % stride:
            continue
        img = cv2.imread(f, cv2.IMREAD_UNCHANGED)
        if img is None:
            continue
        yield idx, os.path.basename(f), img
        n += 1
        if max_frames and n >= max_frames:
            break


def run(args):
    import cv2
    intr = Intrinsics.load(args.intrinsics, fisheye=(True if args.fisheye else None))
    layout = Layout.load(args.layout) if args.layout else Layout.default()
    detector = make_detector(layout.tag_family, quad_decimate=args.quad_decimate)
    ts_map = load_timestamps(args.timestamps, args.ts_index_col, args.ts_col, args.ts_scale) if args.timestamps else None

    out = open(args.out, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(out, fieldnames=DET_FIELDS)
    writer.writeheader()

    n_frames = 0
    n_det = 0
    if args.video:
        source = iter_video(args.video, args.max_frames, args.stride)
        for idx, fps_meta, frame in source:
            fps = args.fps or fps_meta or 30.0
            if ts_map is not None:
                t = ts_map.get(idx, float("nan"))
            else:
                t = args.t0 + idx / fps
            gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            dets = detect_gray(detector, gray)
            rows = pose_rows_for_frame(dets, idx, t, intr, layout, args.min_margin)
            writer.writerows(rows)
            n_frames += 1
            n_det += len(rows)
            if args.show:
                _draw(frame, dets)
                cv2.imshow("marker_drift detect", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
    else:
        for idx, name, img in iter_frames_dir(args.frames_dir, args.max_frames, args.stride):
            if ts_map is not None:
                t = ts_map.get(name, ts_map.get(idx, float("nan")))
            else:
                t = args.t0 + idx / (args.fps or 30.0)
            gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if gray.dtype != np.uint8:
                gray = cv2.convertScaleAbs(gray, alpha=255.0 / max(1, int(gray.max())))
            dets = detect_gray(detector, gray)
            rows = pose_rows_for_frame(dets, idx, t, intr, layout, args.min_margin)
            writer.writerows(rows)
            n_frames += 1
            n_det += len(rows)
    out.close()
    if args.show:
        cv2.destroyAllWindows()
    print("frames={} detections={} -> {}".format(n_frames, n_det, args.out))


def _draw(frame, dets):
    import cv2
    for d in dets:
        c = np.asarray(d.corners).astype(int)
        for i in range(4):
            cv2.line(frame, tuple(c[i]), tuple(c[(i + 1) % 4]), (0, 255, 0), 2)
        cv2.putText(frame, str(d.tag_id), tuple(c[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)


def add_args(p):
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--video")
    src.add_argument("--frames-dir")
    p.add_argument("--intrinsics", required=True)
    p.add_argument("--layout", default=None, help="마커 배치 YAML (기본: layouts/lab_default.yaml 과 같은 기본 배치)")
    p.add_argument("--out", required=True)
    p.add_argument("--fps", type=float, default=None)
    p.add_argument("--t0", type=float, default=0.0, help="첫 프레임 시각(초). 타임스탬프 파일이 없을 때")
    p.add_argument("--timestamps", default=None)
    p.add_argument("--ts-col", default="timestamp_ns")
    p.add_argument("--ts-scale", type=float, default=1e-9)
    p.add_argument("--ts-index-col", default=None)
    p.add_argument("--fisheye", action="store_true", help="intrinsics 를 fisheye 모델로 강제")
    p.add_argument("--quad-decimate", type=float, default=1.0)
    p.add_argument("--min-margin", type=float, default=20.0)
    p.add_argument("--stride", type=int, default=1)
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument("--show", action="store_true")
    p.set_defaults(func=run)
