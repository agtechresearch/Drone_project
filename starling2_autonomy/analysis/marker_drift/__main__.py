"""CLI.  analysis/ 폴더에서:  python -m marker_drift <command> ...

  detect     영상 -> 검출·카메라 포즈 CSV
  analyze    검출 CSV + 기체 로그 -> 시계열 CSV, 요약 CSV, 그림
  noise      정지 촬영 검출 CSV -> 프레임 간 잡음, 정렬 허용오차 제안 (기체 로그 불필요)
  compare    비행 요약 CSV 여러 개 -> 조건 비교 표·그림
  calibrate  체커보드 -> intrinsics YAML
  coverage   경로상 화각 안 태그 수 점검 (마커 크기·간격·이격 결정용, 카메라 파일 없이도 가능)
  layout     기본 마커 배치 YAML 생성
"""

import argparse
import json
import os
import sys

import pandas as pd


def cmd_analyze(args):
    from marker_drift import drift, logs, report
    from marker_drift.markers import Layout

    det = pd.read_csv(args.detections)
    frames = drift.per_frame_pose(det)
    if frames.empty:
        raise SystemExit("배치에 있는 태그의 검출이 없음")
    total_frames = int(det["frame"].nunique()) if "frame" in det.columns else None

    if args.log_format == "v14":
        log = logs.load_v14_csv(args.log, pipe=args.pipe, time_source=args.time_source)
    elif args.log_format == "voxl-logger":
        log = logs.load_voxl_logger_pose_csv(args.log)
    else:
        log = logs.load_generic_csv(args.log, z_up=args.z_up)

    ts = drift.build_timeseries(frames, log, t_offset=args.t_offset)
    ts = ts.dropna(subset=["n", "e", "d"])
    if ts.empty:
        raise SystemExit("영상 시각과 로그 시각이 겹치지 않음. --t-offset 을 확인 (로그 t - 영상 t)")

    t_first, t_last = float(ts["t"].iloc[0]), float(ts["t"].iloc[-1])
    hover = tuple(args.hover) if args.hover else (t_first, t_first + args.hover_sec)
    end = tuple(args.end) if args.end else (t_last - args.end_sec, t_last)
    move = tuple(args.move) if args.move else None

    al = drift.align_L_to_M(ts, hover, method=args.align)
    ts = drift.apply_alignment(ts, al)
    m = drift.compute_metrics(ts, hover, end, path_length_m=args.path_length, move_window=move,
                              total_frames=total_frames)
    m.update({"align_" + k: v for k, v in al.to_dict().items()})
    m["flight"] = args.name or os.path.splitext(os.path.basename(args.detections))[0]
    m["condition"] = args.condition or ""

    os.makedirs(args.out_dir, exist_ok=True)
    base = os.path.join(args.out_dir, m["flight"])
    ts.to_csv(base + "_timeseries.csv", index=False)
    pd.DataFrame([m]).to_csv(base + "_summary.csv", index=False)
    layout = Layout.load(args.layout) if args.layout else Layout.default()
    report.plot_flight(ts, m, base + ".png", layout=layout, title=m["flight"])
    print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.items()}, ensure_ascii=False, indent=1))
    print("-> {}_timeseries.csv, {}_summary.csv, {}.png".format(base, base, base))


def cmd_noise(args):
    from marker_drift import drift
    det = pd.read_csv(args.detections)
    frames = drift.per_frame_pose(det)
    if frames.empty:
        raise SystemExit("검출 없음")
    w = (float(frames["t"].iloc[0]), float(frames["t"].iloc[-1]))
    n = drift.frame_noise(frames, w)
    n["align_tol_deg_suggested"] = drift.alignment_tolerance_deg(n["noise_yaw_deg"], args.k, args.floor)
    n["std_x_m"] = float(frames["x"].std())
    n["std_y_m"] = float(frames["y"].std())
    n["std_z_m"] = float(frames["z"].std())
    n["std_yaw_deg"] = float(frames["yaw_deg"].std())
    n["mean_reproj_px"] = float(frames["reproj_px"].mean())
    print(json.dumps({k: (round(v, 5) if isinstance(v, float) else v) for k, v in n.items()}, indent=1))


def cmd_compare(args):
    from marker_drift import report
    df = report.load_summaries(args.summaries)
    if "condition" not in df.columns or df["condition"].isna().all():
        raise SystemExit("summary 에 condition 컬럼이 없음. analyze 때 --condition 을 주세요")
    table = report.compare(df, args.out_dir, baseline=args.baseline, delta_m=args.delta, alpha=args.alpha, power=args.power)
    pd.set_option("display.width", 200)
    print(table.to_string(index=False))


def cmd_layout(args):
    from marker_drift.markers import Layout, default_lab_layout
    Layout(default_lab_layout(size_m=args.size)).save(args.out)
    print("-> {}".format(args.out))


def main(argv=None):
    p = argparse.ArgumentParser(prog="marker_drift", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    from marker_drift import calibrate, coverage, detect
    detect.add_args(sub.add_parser("detect", help="영상 -> 검출 CSV"))
    calibrate.add_args(sub.add_parser("calibrate", help="체커보드 캘리브레이션"))
    coverage.add_args(sub.add_parser("coverage", help="마커 커버리지 점검 (크기·간격·이격 결정용)"))

    a = sub.add_parser("analyze", help="검출 CSV + 로그 -> 드리프트 지표")
    a.add_argument("--detections", required=True)
    a.add_argument("--log", required=True)
    a.add_argument("--log-format", choices=["v14", "voxl-logger", "generic"], default="v14",
                   help="v14: 비행 스크립트 CSV / voxl-logger: run/mpa/px4_vehicle_local_position/data.csv / generic: t,n,e,d,yaw_deg")
    a.add_argument("--pipe", default="px4_vehicle_local_position")
    a.add_argument("--time-source", choices=["pipe", "mono", "unix"], default="pipe")
    a.add_argument("--z-up", action="store_true", help="generic 로그의 d 가 위 양수일 때")
    a.add_argument("--t-offset", type=float, default=0.0, help="로그 t - 영상 t (초)")
    a.add_argument("--align", choices=["heading", "procrustes"], default="heading")
    a.add_argument("--hover", type=float, nargs=2, metavar=("T0", "T1"), help="정합·호버 창 (영상 시계)")
    a.add_argument("--hover-sec", type=float, default=10.0, help="--hover 생략 시 첫 검출부터 이 길이")
    a.add_argument("--move", type=float, nargs=2, metavar=("T0", "T1"))
    a.add_argument("--end", type=float, nargs=2, metavar=("T0", "T1"))
    a.add_argument("--end-sec", type=float, default=5.0, help="--end 생략 시 마지막 검출까지 이 길이")
    a.add_argument("--path-length", type=float, default=None, help="기본: 태그 x 범위")
    a.add_argument("--layout", default=None)
    a.add_argument("--name", default=None)
    a.add_argument("--condition", default=None, help="A/B/C 등")
    a.add_argument("--out-dir", default="out")
    a.set_defaults(func=cmd_analyze)

    n = sub.add_parser("noise", help="정지 촬영 -> 잡음·허용오차")
    n.add_argument("--detections", required=True)
    n.add_argument("--k", type=float, default=3.0)
    n.add_argument("--floor", type=float, default=1.0)
    n.set_defaults(func=cmd_noise)

    c = sub.add_parser("compare", help="요약 CSV 비교")
    c.add_argument("summaries", nargs="+")
    c.add_argument("--baseline", default="A")
    c.add_argument("--delta", type=float, default=0.03, help="검출하려는 최소 차이 [m]")
    c.add_argument("--alpha", type=float, default=0.05)
    c.add_argument("--power", type=float, default=0.8)
    c.add_argument("--out-dir", default="out/compare")
    c.set_defaults(func=cmd_compare)

    l = sub.add_parser("layout", help="기본 배치 YAML 생성")
    l.add_argument("--out", default="layout.yaml")
    l.add_argument("--size", type=float, default=0.07)
    l.set_defaults(func=cmd_layout)

    args = p.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    sys.exit(main())
