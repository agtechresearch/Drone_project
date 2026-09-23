"""검출 CSV + 기체 로그 -> 드리프트 시계열과 지표.

단계
  1. per_frame_pose      태그별 카메라 포즈를 프레임 단위로 집계(평균 위치, 원형 평균 yaw, 태그 간 편차)
  2. resample_log        기체 로그를 프레임 시각에 보간 (t_offset 적용)
  3. align_L_to_M        기체 로컬 NED -> M 정합. 정렬 완료 후 호버 구간에서 yaw 오프셋(헤딩 차) + 평행이동.
  4. compute_metrics     e(t), E, 드리프트율, 경로 이탈 d(t), 호버 드리프트, 프레임 간 잡음, 검출률

정의는 docs/09 §2.
"""

import math

import numpy as np
import pandas as pd

from marker_drift import geometry


# ---------------------------------------------------------------- 1. 프레임 집계

def per_frame_pose(det_df):
    """검출 CSV(DataFrame) -> 프레임별 카메라 포즈.

    반환 컬럼: frame, t, n_tags, x, y, z, yaw_deg, spread_m(태그별 위치 추정 최대 편차), reproj_px, tags, pose_src

    포즈 선택: 프레임에 배치 태그가 2개 이상이고 joint_* 컬럼이 있으면 한 몸 PnP 결과(pose_src="joint"),
    아니면 태그별 포즈의 평균/원형평균(pose_src="single"). spread_m 은 항상 태그별 포즈에서 계산해
    마커 좌표 실측 오차의 진단값으로 남긴다.
    """
    d = det_df.copy()
    d = d[d["tag_in_layout"] == 1] if "tag_in_layout" in d.columns else d
    for c in ("cam_x", "cam_y", "cam_z", "cam_yaw_deg", "reproj_px", "t"):
        d[c] = pd.to_numeric(d[c], errors="coerce")
    has_joint = all(c in d.columns for c in ("joint_n_tags", "joint_x", "joint_y", "joint_z", "joint_yaw_deg"))
    if has_joint:
        for c in ("joint_n_tags", "joint_x", "joint_y", "joint_z", "joint_yaw_deg", "joint_reproj_px"):
            d[c] = pd.to_numeric(d[c], errors="coerce")
    d = d.dropna(subset=["cam_x", "cam_y", "cam_z", "t"])
    rows = []
    for frame, g in d.groupby("frame", sort=True):
        P = g[["cam_x", "cam_y", "cam_z"]].to_numpy()
        mean = P.mean(axis=0)
        spread = float(np.max(np.linalg.norm(P - mean, axis=1)) * 2.0) if len(P) > 1 else 0.0
        row = {"frame": int(frame), "t": float(g["t"].iloc[0]), "n_tags": int(len(g)), "spread_m": spread,
               "tags": " ".join(str(int(i)) for i in g["tag_id"].tolist())}
        j = g.iloc[0]
        if has_joint and len(g) >= 2 and not np.isnan(j["joint_x"]):
            row.update({"x": j["joint_x"], "y": j["joint_y"], "z": j["joint_z"], "yaw_deg": j["joint_yaw_deg"],
                        "reproj_px": float(j["joint_reproj_px"]), "pose_src": "joint"})
        else:
            row.update({"x": mean[0], "y": mean[1], "z": mean[2],
                        "yaw_deg": geometry.circular_mean_deg(g["cam_yaw_deg"].to_numpy()),
                        "reproj_px": float(g["reproj_px"].mean()) if "reproj_px" in g else np.nan, "pose_src": "single"})
        rows.append(row)
    return pd.DataFrame(rows)


# ---------------------------------------------------------------- 2. 로그 보간

EDGE_TOL_S = 0.05  # 로그 범위 밖이라도 이 시간 안이면 끝값으로 본다(부동소수·표본 주기 여유)


def _interp(t_new, t, v):
    t_new = np.asarray(t_new, float)
    t = np.asarray(t, float)
    v = np.asarray(v, float)
    ok = ~np.isnan(v)
    if ok.sum() < 2:
        return np.full_like(t_new, np.nan)
    t, v = t[ok], v[ok]
    out = np.interp(np.clip(t_new, t[0], t[-1]), t, v)
    out[(t_new < t[0] - EDGE_TOL_S) | (t_new > t[-1] + EDGE_TOL_S)] = np.nan
    return out


def _interp_angle(t_new, t, a_deg):
    a = np.asarray(a_deg, float)
    ok = ~np.isnan(a)
    if ok.sum() < 2:
        return np.full_like(np.asarray(t_new, float), np.nan)
    un = np.degrees(np.unwrap(np.radians(a[ok])))
    return geometry.wrap_deg(_interp(t_new, np.asarray(t, float)[ok], un))


def resample_log(log_df, t_frames, t_offset=0.0):
    """기체 로그를 프레임 시각으로 보간. t_offset: 로그 시계 = 영상 시계 + t_offset 이 되도록 더해주는 값."""
    t_log = log_df["t"].to_numpy(float) - float(t_offset)
    out = pd.DataFrame({"t": np.asarray(t_frames, float)})
    for c in ("n", "e", "d", "cmd_n", "cmd_e", "cmd_d", "mav_n", "mav_e", "mav_u"):
        if c in log_df.columns:
            out[c] = _interp(out["t"], t_log, log_df[c])
    out["yaw_deg"] = _interp_angle(out["t"], t_log, log_df["yaw_deg"]) if "yaw_deg" in log_df.columns else np.nan
    if "stage" in log_df.columns:
        idx = np.searchsorted(t_log, out["t"].to_numpy(), side="right") - 1
        idx = np.clip(idx, 0, len(t_log) - 1)
        out["stage"] = log_df["stage"].to_numpy()[idx]
    return out


# ---------------------------------------------------------------- 3. 프레임 정합

class Alignment:
    """L(NED) -> M 변환: ENU 로 바꾼 뒤 컴퍼스 회전 delta_deg, 평행이동 trans."""

    def __init__(self, delta_deg, trans, method, window):
        self.delta_deg = float(delta_deg)
        self.trans = np.asarray(trans, float).reshape(3)
        self.method = method
        self.window = window

    def apply(self, ned_xyz):
        enu = geometry.ned_to_enu(ned_xyz)
        xy = geometry.compass_rotate(enu[:, :2], self.delta_deg)
        return np.column_stack([xy, enu[:, 2]]) + self.trans

    def apply_yaw(self, yaw_ned_deg):
        return geometry.wrap_deg(np.asarray(yaw_ned_deg, float) + self.delta_deg)

    def to_dict(self):
        return {"method": self.method, "delta_deg": self.delta_deg,
                "trans_x": self.trans[0], "trans_y": self.trans[1], "trans_z": self.trans[2],
                "window_start": self.window[0], "window_end": self.window[1]}


def align_L_to_M(ts, window, method="heading"):
    """ts: 프레임 시계열(x,y,z,yaw_deg = 태그 기준 카메라, n,e,d,yaw_ned = 기체 로그 보간). window = (t0, t1).

    method
      "heading"    yaw 오프셋 = 원형평균(태그 yaw - 기체 yaw). 정지 호버에서도 동작. 기본.
      "procrustes" 수평 위치 궤적의 2D 회전을 최소자승으로 추정. 창 안에서 충분히 움직여야 하고,
                   창 안에 드리프트가 있으면 그 회전 성분까지 흡수해 드리프트를 과소평가한다.
                   heading 이 불가능할 때(기체 yaw 로그가 없을 때)만 쓴다.
    """
    m = (ts["t"] >= window[0]) & (ts["t"] <= window[1])
    m &= ts[["x", "y", "z", "n", "e", "d"]].notna().all(axis=1)
    if m.sum() < 3:
        raise ValueError("정합 창 안에 유효 표본이 3개 미만")
    w = ts[m]
    tag_xyz = w[["x", "y", "z"]].to_numpy(float)
    enu = geometry.ned_to_enu(w[["n", "e", "d"]].to_numpy(float))

    if method == "heading":
        if w["yaw_deg"].isna().all() or w["yaw_ned_deg"].isna().all():
            raise ValueError("heading 정합에는 태그 yaw 와 기체 yaw 가 모두 필요")
        diff = geometry.wrap_deg(w["yaw_deg"].to_numpy(float) - w["yaw_ned_deg"].to_numpy(float))
        delta = geometry.circular_mean_deg(diff)
    elif method == "procrustes":
        A = enu[:, :2] - enu[:, :2].mean(axis=0)
        B = tag_xyz[:, :2] - tag_xyz[:, :2].mean(axis=0)
        if np.linalg.norm(A) < 0.05:
            raise ValueError("procrustes 정합에는 창 안에서 5 cm 이상 움직여야 한다")
        # 컴퍼스 회전 행렬 [[c, s], [-s, c]] 를 A -> B 로 맞추는 각 delta
        # B ≈ A @ M^T,  M = [[c, s], [-s, c]]  ->  c ∝ sum(a·b), s ∝ sum(a_x b_y ... ) 부호 정리:
        # b_x = c a_x + s a_y,  b_y = -s a_x + c a_y
        c = float((A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1]).sum())
        s = float((A[:, 1] * B[:, 0] - A[:, 0] * B[:, 1]).sum())
        delta = math.degrees(math.atan2(s, c))
    else:
        raise ValueError("unknown method: {}".format(method))

    rot = np.column_stack([geometry.compass_rotate(enu[:, :2], delta), enu[:, 2]])
    trans = tag_xyz.mean(axis=0) - rot.mean(axis=0)
    return Alignment(delta, trans, method, window)


# ---------------------------------------------------------------- 4. 지표

def build_timeseries(frame_df, log_df, t_offset=0.0):
    """프레임 포즈 + 로그 -> 하나의 시계열. 기체 컬럼: n,e,d,yaw_ned_deg,cmd_*"""
    rs = resample_log(log_df, frame_df["t"].to_numpy(float), t_offset)
    ts = frame_df.reset_index(drop=True).copy()
    for c in rs.columns:
        if c == "t":
            continue
        ts["yaw_ned_deg" if c == "yaw_deg" else c] = rs[c].to_numpy()
    return ts


def apply_alignment(ts, al):
    ts = ts.copy()
    P = al.apply(ts[["n", "e", "d"]].to_numpy(float))
    ts["vio_x"], ts["vio_y"], ts["vio_z"] = P[:, 0], P[:, 1], P[:, 2]
    ts["vio_yaw_deg"] = al.apply_yaw(ts["yaw_ned_deg"].to_numpy(float))
    ts["e_x"] = ts["vio_x"] - ts["x"]
    ts["e_y"] = ts["vio_y"] - ts["y"]
    ts["e_z"] = ts["vio_z"] - ts["z"]
    ts["e_yaw_deg"] = geometry.wrap_deg(ts["vio_yaw_deg"] - ts["yaw_deg"])
    ts["e_h"] = np.hypot(ts["e_x"], ts["e_y"])
    if ts[["cmd_n", "cmd_e", "cmd_d"]].notna().any().any():
        C = al.apply(ts[["cmd_n", "cmd_e", "cmd_d"]].to_numpy(float))
        ts["cmd_x"], ts["cmd_y"], ts["cmd_z"] = C[:, 0], C[:, 1], C[:, 2]
        ts["d_x"] = ts["x"] - ts["cmd_x"]
        ts["d_y"] = ts["y"] - ts["cmd_y"]
        ts["d_z"] = ts["z"] - ts["cmd_z"]
        ts["d_h"] = np.hypot(ts["d_x"], ts["d_y"])
    # 진행 거리(태그 기준 x 의 시작점 대비)
    x0 = ts["x"].iloc[: max(1, min(10, len(ts)))].mean()
    ts["dist_m"] = ts["x"] - x0
    return ts


def _win(ts, window):
    return ts[(ts["t"] >= window[0]) & (ts["t"] <= window[1])]


def _slope_per_s(t, v):
    t = np.asarray(t, float)
    v = np.asarray(v, float)
    ok = ~np.isnan(v)
    if ok.sum() < 3:
        return float("nan")
    return float(np.polyfit(t[ok], v[ok], 1)[0])


def frame_noise(ts, window):
    """정지 구간의 프레임 간 잡음: 연속 프레임 차이의 표준편차 / sqrt(2) (한 프레임 측정의 표준편차 추정)."""
    w = _win(ts, window)
    out = {}
    for c, key in (("x", "noise_x_m"), ("y", "noise_y_m"), ("z", "noise_z_m")):
        v = w[c].to_numpy(float)
        out[key] = float(np.nanstd(np.diff(v)) / math.sqrt(2)) if len(v) > 2 else float("nan")
    yaw = w["yaw_deg"].to_numpy(float)
    out["noise_yaw_deg"] = float(np.nanstd(geometry.wrap_deg(np.diff(yaw))) / math.sqrt(2)) if len(yaw) > 2 else float("nan")
    out["noise_frames"] = int(len(w))
    return out


def alignment_tolerance_deg(noise_yaw_deg, k=3.0, floor_deg=1.0):
    """docs/09 §5.3: 허용오차 = max(floor, k * sigma_yaw)."""
    if noise_yaw_deg is None or np.isnan(noise_yaw_deg):
        return float("nan")
    return float(max(floor_deg, k * noise_yaw_deg))


def compute_metrics(ts, hover_window, end_window, path_length_m=None, move_window=None,
                    total_frames=None):
    """정합된 시계열 -> 한 비행의 요약 사전."""
    m = {}
    hov = _win(ts, hover_window)
    end = _win(ts, end_window)

    # 최종 드리프트 E (도착 호버 평균)
    for c in ("e_x", "e_y", "e_z", "e_yaw_deg", "e_h"):
        m["E_" + c[2:]] = float(end[c].mean()) if len(end) else float("nan")
    # 경로 길이: 인자 > 태그 x 범위
    if path_length_m is None:
        path_length_m = float(np.nanmax(ts["x"]) - np.nanmin(ts["x"]))
    m["path_length_m"] = float(path_length_m)
    t_move0 = move_window[0] if move_window else hover_window[1]
    t_move1 = move_window[1] if move_window else end_window[0]
    m["move_duration_s"] = float(t_move1 - t_move0)
    if path_length_m > 0:
        m["rate_h_per_m"] = m["E_h"] / path_length_m
    if m["move_duration_s"] > 0:
        m["rate_h_per_s"] = m["E_h"] / m["move_duration_s"]

    # 경로 이탈 (이동 구간)
    if "d_h" in ts.columns:
        mv = ts[(ts["t"] >= t_move0) & (ts["t"] <= t_move1)]
        m["dev_mean_h_m"] = float(mv["d_h"].mean()) if len(mv) else float("nan")
        m["dev_rms_h_m"] = float(np.sqrt(np.nanmean(mv["d_h"] ** 2))) if len(mv) else float("nan")
        m["dev_max_h_m"] = float(mv["d_h"].max()) if len(mv) else float("nan")
        m["dev_rms_z_m"] = float(np.sqrt(np.nanmean(mv["d_z"] ** 2))) if len(mv) else float("nan")

    # 호버 드리프트 (태그 기준 실제 위치의 흔들림·추세)
    for c in ("x", "y", "z"):
        m["hover_std_{}_m".format(c)] = float(hov[c].std()) if len(hov) > 1 else float("nan")
        m["hover_slope_{}_m_per_s".format(c)] = _slope_per_s(hov["t"], hov[c])
    m["hover_std_yaw_deg"] = float(np.degrees(np.std(np.radians(hov["yaw_deg"])))) if len(hov) > 1 else float("nan")

    # 프레임 간 잡음 + 허용오차 제안
    m.update(frame_noise(ts, hover_window))
    m["align_tol_deg_suggested"] = alignment_tolerance_deg(m["noise_yaw_deg"])

    # 검출 품질
    m["frames_with_pose"] = int(len(ts))
    if total_frames:
        m["detect_rate"] = float(len(ts) / total_frames)
    m["multi_tag_frames"] = int((ts["n_tags"] > 1).sum())
    m["multi_tag_spread_mean_m"] = float(ts.loc[ts["n_tags"] > 1, "spread_m"].mean()) if m["multi_tag_frames"] else float("nan")
    m["reproj_px_mean"] = float(ts["reproj_px"].mean())
    m["duration_s"] = float(ts["t"].iloc[-1] - ts["t"].iloc[0]) if len(ts) else float("nan")
    return m


def sample_size_two_group(sd, delta, alpha=0.05, power=0.8):
    """두 표본 t검정 근사: 그룹당 n = 2 (z_{1-a/2} + z_{power})^2 (sd/delta)^2.  alpha .05, power .8 -> 15.7 (sd/delta)^2"""
    from scipy.stats import norm
    if delta <= 0 or sd <= 0:
        return float("nan")
    z = norm.ppf(1 - alpha / 2) + norm.ppf(power)
    return float(math.ceil(2 * (z * sd / delta) ** 2))
