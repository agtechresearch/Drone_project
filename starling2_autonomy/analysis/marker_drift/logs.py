"""기체 로그 로더. 결과는 공통 스키마의 pandas DataFrame:

  t          초. 기본은 VOXL 단조시계(MPA 타임스탬프와 같은 시계). 영상 타임스탬프와 직접 비교 가능.
  n, e, d    로컬 NED 위치 [m]. v14 CSV 는 파이프 원시값에서 초기값을 뺀 rel 값을 쓴다.
  yaw_deg    헤딩(NED, 시계방향 양수). 없으면 NaN.
  cmd_n, cmd_e, cmd_d   명령 setpoint (같은 프레임). 없으면 NaN.
  stage, phase          미션 단계 문자열(있으면).

지원
  load_v14_csv     flight/path_flight_phase1_v14.py 의 --csv 출력
  load_generic_csv 임의 CSV. 컬럼 이름을 인자로 매핑.
"""

import numpy as np
import pandas as pd

V14_DEFAULT_PIPE = "px4_vehicle_local_position"


def _num(s):
    return pd.to_numeric(s, errors="coerce")


def load_v14_csv(path, pipe=V14_DEFAULT_PIPE, time_source="pipe"):
    """v14 CSV -> 공통 스키마.

    time_source
      "pipe"  voxl_<pipe>_ts_ms / 1000  (VOXL 단조시계, 권장. 파이프 표본이 갱신될 때만 바뀌므로 중복 t 는 제거)
      "mono"  mono_time (스크립트가 기체에서 돌았다면 같은 시계)
      "unix"  unix_time
    """
    df = pd.read_csv(path)
    p = "voxl_{}_".format(pipe)
    out = pd.DataFrame()

    if time_source == "pipe" and p + "ts_ms" in df.columns:
        out["t"] = _num(df[p + "ts_ms"]) / 1000.0
    elif time_source in ("pipe", "mono") and "mono_time" in df.columns:
        out["t"] = _num(df["mono_time"])
    else:
        out["t"] = _num(df["unix_time"])

    if p + "rel_x" in df.columns:
        out["n"] = _num(df[p + "rel_x"])
        out["e"] = _num(df[p + "rel_y"])
        out["d"] = _num(df[p + "rel_z"])
    else:  # 파이프 컬럼이 없으면 MAVSDK 값으로 대체
        out["n"] = _num(df["mav_rel_n"])
        out["e"] = _num(df["mav_rel_e"])
        out["d"] = -_num(df["mav_rel_u"])

    out["yaw_deg"] = _num(df[p + "yaw_deg"]) if p + "yaw_deg" in df.columns else np.nan

    if "target_rel_n" in df.columns:
        out["cmd_n"] = _num(df["target_rel_n"])
        out["cmd_e"] = _num(df["target_rel_e"])
        out["cmd_d"] = -_num(df["target_rel_u"])
    else:
        out["cmd_n"] = out["cmd_e"] = out["cmd_d"] = np.nan

    out["stage"] = df["stage"] if "stage" in df.columns else ""
    out["phase"] = df["phase"] if "phase" in df.columns else ""

    # MAVSDK 값도 진단용으로 보존
    for src, dst in (("mav_rel_n", "mav_n"), ("mav_rel_e", "mav_e"), ("mav_rel_u", "mav_u")):
        if src in df.columns:
            out[dst] = _num(df[src])

    out = out.dropna(subset=["t", "n", "e", "d"])
    out = out.drop_duplicates(subset=["t"], keep="last").sort_values("t").reset_index(drop=True)
    return out


def load_generic_csv(path, t="t", n="n", e="e", d="d", yaw="yaw_deg",
                     cmd_n=None, cmd_e=None, cmd_d=None, t_scale=1.0, z_up=False):
    """임의 CSV -> 공통 스키마. z_up=True 면 d 컬럼이 '위' 양수라고 보고 부호를 뒤집는다."""
    df = pd.read_csv(path)
    out = pd.DataFrame()
    out["t"] = _num(df[t]) * float(t_scale)
    out["n"] = _num(df[n])
    out["e"] = _num(df[e])
    out["d"] = -_num(df[d]) if z_up else _num(df[d])
    out["yaw_deg"] = _num(df[yaw]) if yaw and yaw in df.columns else np.nan
    # 명령 컬럼: 인자로 안 주면 cmd_n/cmd_e/cmd_d 이름을 자동 인식
    cmd_n = cmd_n or ("cmd_n" if "cmd_n" in df.columns else None)
    cmd_e = cmd_e or ("cmd_e" if "cmd_e" in df.columns else None)
    cmd_d = cmd_d or ("cmd_d" if "cmd_d" in df.columns else None)
    for src, dst in ((cmd_n, "cmd_n"), (cmd_e, "cmd_e"), (cmd_d, "cmd_d")):
        out[dst] = _num(df[src]) if src and src in df.columns else np.nan
    if z_up and cmd_d and cmd_d in df.columns:
        out["cmd_d"] = -out["cmd_d"]
    out["stage"] = df["stage"] if "stage" in df.columns else ""
    out["phase"] = df["phase"] if "phase" in df.columns else ""
    out = out.dropna(subset=["t", "n", "e", "d"])
    return out.drop_duplicates(subset=["t"], keep="last").sort_values("t").reset_index(drop=True)


def load_voxl_logger_pose_csv(path, t_scale=1e-9):
    """voxl-logger 의 pose_6dof 채널(`-f px4_vehicle_local_position`) data.csv -> 공통 스키마.

    컬럼(기체 실측 2026-09-23, voxl-logger 0.6.1):
      i,timestamp(ns),T_ch_wrt_par_x(m),T_ch_wrt_par_y(m),T_ch_wrt_par_z(m),roll(rad),pitch(rad),yaw(rad),
      vel_ch_wrt_par_x(m/s),...,angular_vel_z(rad/s)
    px4_vehicle_local_position 은 NED 이므로 x,y,z -> n,e,d 그대로. 이륙 전에는 x,y 가 nan 이다(행 제거).
    timestamp(ns) 는 CLOCK_MONOTONIC. 카메라 data.csv 의 timestamp(ns) 와 같은 시계라 t_offset 없이 맞는다.
    """
    # voxl-logger 는 행 끝에 쉼표를 하나 더 찍는다(헤더보다 필드가 1개 많음). index_col=False 가 없으면
    # pandas 가 첫 컬럼을 인덱스로 삼아 모든 컬럼이 한 칸씩 밀린다.
    df = pd.read_csv(path, index_col=False)
    out = pd.DataFrame()
    out["t"] = _num(df["timestamp(ns)"]) * float(t_scale)
    out["n"] = _num(df["T_ch_wrt_par_x(m)"])
    out["e"] = _num(df["T_ch_wrt_par_y(m)"])
    out["d"] = _num(df["T_ch_wrt_par_z(m)"])
    out["yaw_deg"] = np.degrees(_num(df["yaw(rad)"]))
    out["cmd_n"] = out["cmd_e"] = out["cmd_d"] = np.nan   # 명령은 v14 CSV 에만 있다
    out["stage"] = ""
    out["phase"] = ""
    out = out[out["t"] > 0].dropna(subset=["t", "n", "e", "d"])
    return out.drop_duplicates(subset=["t"], keep="last").sort_values("t").reset_index(drop=True)


def load_voxl_logger_ov_csv(path, t_scale=1e-9):
    """voxl-logger 의 vio 채널(`-v ov`) data.csv -> 자체 지표용 DataFrame.

    컬럼(기체 실측): i,timestamp(ns),T_imu_wrt_vio_x/y/z(m),roll/pitch/yaw(rad),vel_*,angular_vel_*,gravity_vector_*,
      T_cam_wrt_imu_*,imu_to_cam_*,features,quality,state,error_code
    반환: t, vio_x, vio_y, vio_z (VIO 자체 프레임, 중력 정렬 아님), vio_yaw_deg, features, quality, state, error_code
    첫 행은 timestamp -1e9 (미초기화) 로 나오므로 t <= 0 행은 버린다.
    """
    df = pd.read_csv(path, index_col=False)
    out = pd.DataFrame()
    out["t"] = _num(df["timestamp(ns)"]) * float(t_scale)
    out["vio_x"] = _num(df["T_imu_wrt_vio_x(m)"])
    out["vio_y"] = _num(df["T_imu_wrt_vio_y(m)"])
    out["vio_z"] = _num(df["T_imu_wrt_vio_z(m)"])
    out["vio_yaw_deg"] = np.degrees(_num(df["yaw(rad)"]))
    for c in ("features", "quality", "state", "error_code"):
        out[c] = _num(df[c]) if c in df.columns else np.nan
    out = out[out["t"] > 0]
    return out.drop_duplicates(subset=["t"], keep="last").sort_values("t").reset_index(drop=True)


def stage_windows(df, name_contains):
    """stage 컬럼에서 name_contains 를 포함하는 구간들의 (t_start, t_end) 리스트."""
    if "stage" not in df.columns:
        return []
    mask = df["stage"].astype(str).str.contains(name_contains, case=False, regex=False)
    if not mask.any():
        return []
    windows = []
    t = df["t"].to_numpy()
    m = mask.to_numpy()
    start = None
    for i in range(len(m)):
        if m[i] and start is None:
            start = t[i]
        if start is not None and (not m[i] or i == len(m) - 1):
            end = t[i] if not m[i] else t[i]
            windows.append((float(start), float(end)))
            start = None
    return windows
