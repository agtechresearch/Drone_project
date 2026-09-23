"""체커보드 캘리브레이션 -> intrinsics YAML.

  python -m marker_drift calibrate --video cal.mp4 --board 9x6 --square 0.025 --out hires.yaml [--fisheye] [--stride 10]

--board 는 내부 코너 수(가로x세로). VOXL 기본 보드는 6x5 (65.5 mm 격자) 이지만 hires 는 공장 캘리브레이션이
없을 수 있어 별도 보드로 찍는다. 프레임은 --stride 로 건너뛰며 코너가 잡힌 것만 쓴다(최대 --max-views).
"""

import numpy as np

from marker_drift.detect import iter_frames_dir, iter_video
from marker_drift.intrinsics import Intrinsics


def run(args):
    import cv2
    cols, rows = [int(v) for v in args.board.lower().split("x")]
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(args.square)

    obj_pts, img_pts = [], []
    size = None
    source = iter_video(args.video, None, args.stride) if args.video else iter_frames_dir(args.frames_dir, None, args.stride)
    for item in source:
        img = item[2]
        gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows),
                                                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        obj_pts.append(objp)
        img_pts.append(corners)
        if len(obj_pts) >= args.max_views:
            break
    if len(obj_pts) < 8:
        raise SystemExit("코너가 잡힌 프레임이 {}개. 최소 8개 필요".format(len(obj_pts)))

    if args.fisheye:
        K = np.eye(3)
        D = np.zeros(4)
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        rms, K, D, _, _ = cv2.fisheye.calibrate(
            [o.reshape(-1, 1, 3) for o in obj_pts], [i.reshape(-1, 1, 2) for i in img_pts], size, K, D,
            flags=flags, criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
        intr = Intrinsics(K, D.reshape(-1), size[0], size[1], "fisheye")
    else:
        rms, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
        intr = Intrinsics(K, D.reshape(-1), size[0], size[1], "pinhole")
    intr.save(args.out)
    print("views={} rms={:.3f}px size={} -> {}".format(len(obj_pts), rms, size, args.out))
    print("K=\n{}".format(np.array2string(intr.K, precision=2)))


def add_args(p):
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--video")
    src.add_argument("--frames-dir")
    p.add_argument("--board", default="9x6", help="내부 코너 수 가로x세로")
    p.add_argument("--square", type=float, required=True, help="격자 한 칸 크기 [m]")
    p.add_argument("--out", required=True)
    p.add_argument("--fisheye", action="store_true")
    p.add_argument("--stride", type=int, default=10)
    p.add_argument("--max-views", type=int, default=60)
    p.set_defaults(func=run)
