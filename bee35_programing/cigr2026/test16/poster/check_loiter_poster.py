#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v14 로이터 비행 로그 분석 (cm 기반)

v14 변경사항:
  - 임계가 cm 단위 (좌우는 가용 좌우의 70%, 동적)
  - 거리 임계: 25~55cm 비대칭
  - 새 로그 컬럼: marker_px, err_x_cm, distance_cm, lateral_thr_px
  - 마커 30mm, focal_length 950.3 px

분석 포인트:
  1. 정렬 성공률 (cm 기준 + 농업 임계)
  2. 거리 추정 (앞뒤) 분포
  3. 좌우 cm 분포 + 동적 임계 만족율
  4. PID 응답성 (err → 명령 → 자세)
  5. 운영 박스 (25~55cm × 좌우 가용) 안에 머무는 비율

사용법:
  python check_loiter_v14.py logs/v14/loiter_v14_realtime_*.csv
  python check_loiter_v14.py             # 가장 최근 자동
  python check_loiter_v14.py --baseline  # baseline_v14 로그
"""

import sys
import os
import glob
import argparse
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.patches import Rectangle
from datetime import datetime as _dt


def setup_korean_font():
    candidates = [
        'NanumGothic', 'Nanum Gothic', 'NanumBarunGothic',
        'Noto Sans CJK KR', 'Noto Sans KR', 'Source Han Sans KR',
        'AppleGothic', 'Apple SD Gothic Neo',
        'Malgun Gothic', 'UnDotum', 'Baekmuk Gulim',
    ]
    available = {f.name for f in fm.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams['font.family'] = name
            plt.rcParams['axes.unicode_minus'] = False
            print(f"[FONT] 한글 폰트 적용: {name}")
            return True
    print("[FONT] ⚠️  한글 폰트 없음 — 라벨이 □로 표시될 수 있어요.")
    plt.rcParams['axes.unicode_minus'] = False
    return False

setup_korean_font()

OUT_DIR = "figures"

# ─────────────────────────────────────────────
# v14 환경 상수 (코드 alighn_l_v14.py와 동일해야 함)
# ─────────────────────────────────────────────
MARKER_SIZE_MM     = 30.0
FOCAL_LENGTH_PX    = 950.3

# 정렬 범위 (비전 운영)
TARGET_DISTANCE_CM = 40.0
DISTANCE_MIN_CM    = 25.0
DISTANCE_MAX_CM    = 55.0

LATERAL_RATIO      = 0.7

CAM_W, CAM_H       = 640, 480

# 농업 운용 임계 (분석용)
AGRICULTURAL_LATERAL_CM = 40.0

# ─────────────────────────────────────────────
# 안전 범위 (충돌 방지)
# 드론 전장 22cm (카메라부터 후방까지)
# 재배대 간격 100cm
# 앞쪽 마진 10cm, 뒤쪽 마진 10cm
# ─────────────────────────────────────────────
DRONE_LENGTH_CM   = 22.0
ROW_SPACING_CM    = 100.0
SAFETY_MARGIN_CM  = 10.0
SAFETY_MIN_CM     = SAFETY_MARGIN_CM                                     # 10cm
SAFETY_MAX_CM     = ROW_SPACING_CM - DRONE_LENGTH_CM - SAFETY_MARGIN_CM  # 68cm


# ─────────────────────────────────────────────
# 유틸 함수
# ─────────────────────────────────────────────
def marker_px_to_distance_cm(marker_px):
    if isinstance(marker_px, (int, float)):
        if marker_px <= 0:
            return np.nan
        return (FOCAL_LENGTH_PX * MARKER_SIZE_MM) / marker_px / 10.0
    # Series
    result = (FOCAL_LENGTH_PX * MARKER_SIZE_MM) / marker_px / 10.0
    return result.where(marker_px > 0, np.nan)


def err_x_px_to_cm(err_x_px, marker_px):
    if isinstance(marker_px, (int, float)):
        if marker_px <= 0:
            return 0.0
        return err_x_px * (MARKER_SIZE_MM / marker_px) / 10.0
    result = err_x_px * (MARKER_SIZE_MM / marker_px) / 10.0
    return result.where(marker_px > 0, 0.0)


def lateral_threshold_px(marker_px):
    available_px = (CAM_W - marker_px) / 2
    return available_px * LATERAL_RATIO


def find_latest(pattern):
    files = sorted(glob.glob(pattern, recursive=True))
    return files[-1] if files else None


def load_log(path, measurement_only=True, measurement_duration=60.0):
    """
    로그 로드.
    measurement_only=True: 첫 정렬 도달 이후 measurement_duration(60s)만 반환
    measurement_only=False: 전체 비행 로그
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"파일 없음: {path}")
    if os.path.getsize(path) == 0:
        raise ValueError(f"⚠️  빈 파일: {path}")
    df = pd.read_csv(path)
    if df.empty:
        raise ValueError(f"⚠️  데이터 없음: {path}")

    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df["elapsed_s"] = df["elapsed_s"] - df["elapsed_s"].iloc[0]
    
    # v14 로그 컬럼 확인 — 없으면 자동 계산
    if 'marker_px' not in df.columns:
        print("[WARN] marker_px 컬럼 없음 — v13 이하 로그? err_pitch + 70으로 추정")
        df['marker_px'] = df['err_pitch_px'] + 70
    if 'err_x_cm' not in df.columns:
        df['err_x_cm'] = err_x_px_to_cm(df['err_x_px'], df['marker_px'])
    if 'distance_cm' not in df.columns:
        df['distance_cm'] = marker_px_to_distance_cm(df['marker_px'])
    if 'lateral_thr_px' not in df.columns:
        df['lateral_thr_px'] = lateral_threshold_px(df['marker_px'])
    
    # 첫 정렬 후 60초만 (비행 측정과 동일)
    if measurement_only:
        aligned_rows = df[df['aligned'] == 1]
        if len(aligned_rows) == 0:
            print("[WARN] 정렬 도달 없음 — 전체 비행 분석")
        else:
            first_align_time = aligned_rows.iloc[0]['elapsed_s']
            end_time = first_align_time + measurement_duration
            mask = (df['elapsed_s'] >= first_align_time) & (df['elapsed_s'] < end_time)
            df_measure = df[mask].copy()
            # elapsed_s를 측정 시작 기준으로 재설정
            df_measure['elapsed_s'] = df_measure['elapsed_s'] - first_align_time
            print(f"[MEASURE] 첫 정렬: {first_align_time:.2f}s → {measurement_duration:.0f}s 분석")
            print(f"          전체 {len(df)} → 측정 {len(df_measure)} 샘플")
            return df_measure
    
    return df


# ─────────────────────────────────────────────
# 분석
# ─────────────────────────────────────────────
def analyze(df, label):
    print(f"\n=== {label} ===")
    print(f"  비행 시간: {df['elapsed_s'].max():.1f}s ({len(df)} 샘플)")

    detected = (df["err_x_px"] != 0) | (df["err_pitch_px"] != 0)
    valid = df[detected].copy()
    detect_pct = len(valid) / len(df) * 100
    print(f"  마커 감지: {len(valid)} ({detect_pct:.1f}%)")
    
    if len(valid) < 10:
        print("  ⚠️  유효 샘플 부족")
        return None

    # 정렬 통계
    aligned_pct_total = valid['aligned'].sum() / len(df) * 100
    aligned_pct_det   = valid['aligned'].mean() * 100
    print(f"\n  [정렬 성공률]")
    print(f"    전체 대비:   {aligned_pct_total:5.1f}%")
    print(f"    감지 중 대비: {aligned_pct_det:5.1f}%")

    # 거리 통계 (앞뒤)
    print(f"\n  [거리 (앞뒤) — 목표 {TARGET_DISTANCE_CM:.0f}cm, 허용 {DISTANCE_MIN_CM:.0f}~{DISTANCE_MAX_CM:.0f}cm]")
    dist_mean = valid['distance_cm'].mean()
    dist_std  = valid['distance_cm'].std()
    dist_in_range = ((valid['distance_cm'] >= DISTANCE_MIN_CM) & 
                     (valid['distance_cm'] <= DISTANCE_MAX_CM)).sum() / len(valid) * 100
    print(f"    평균: {dist_mean:.1f} ± {dist_std:.1f} cm")
    print(f"    범위: {valid['distance_cm'].min():.1f} ~ {valid['distance_cm'].max():.1f} cm")
    print(f"    정렬 범위 만족 ({DISTANCE_MIN_CM:.0f}~{DISTANCE_MAX_CM:.0f}cm): {dist_in_range:.1f}%")
    
    # 안전 범위 (충돌 방지)
    safe_pct = ((valid['distance_cm'] >= SAFETY_MIN_CM) & 
                (valid['distance_cm'] <= SAFETY_MAX_CM)).sum() / len(valid) * 100
    front_collision_pct = (valid['distance_cm'] < SAFETY_MIN_CM).sum() / len(valid) * 100
    rear_collision_pct  = (valid['distance_cm'] > SAFETY_MAX_CM).sum() / len(valid) * 100
    print(f"    안전 범위 만족 ({SAFETY_MIN_CM:.0f}~{SAFETY_MAX_CM:.0f}cm): {safe_pct:.1f}%  ⭐")
    if front_collision_pct > 0:
        print(f"      ⚠️  앞 충돌 위험 (<{SAFETY_MIN_CM:.0f}cm): {front_collision_pct:.2f}%")
    if rear_collision_pct > 0:
        print(f"      ⚠️  뒤 충돌 위험 (>{SAFETY_MAX_CM:.0f}cm): {rear_collision_pct:.2f}%")
    
    # 좌우 통계 (cm)
    print(f"\n  [좌우 (err_x) — 동적 임계 (가용의 {LATERAL_RATIO*100:.0f}%)]")
    lat_mean = valid['err_x_cm'].mean()
    lat_std  = valid['err_x_cm'].std()
    # 동적 임계 만족율
    lateral_thr_cm = err_x_px_to_cm(valid['lateral_thr_px'], valid['marker_px'])
    lat_in_dynamic = (valid['err_x_cm'].abs() <= lateral_thr_cm).sum() / len(valid) * 100
    # 농업 임계 만족율
    lat_in_agri = (valid['err_x_cm'].abs() <= AGRICULTURAL_LATERAL_CM).sum() / len(valid) * 100
    print(f"    평균: {lat_mean:+.2f} ± {lat_std:.2f} cm")
    print(f"    범위: {valid['err_x_cm'].min():+.1f} ~ {valid['err_x_cm'].max():+.1f} cm")
    print(f"    동적 임계 만족: {lat_in_dynamic:.1f}% (임계 평균 ±{lateral_thr_cm.mean():.1f}cm)")
    print(f"    농업 임계(±{AGRICULTURAL_LATERAL_CM:.0f}cm) 만족: {lat_in_agri:.1f}%")
    
    # 종합 (거리 + 좌우 둘 다 OK)
    both_ok = ((valid['distance_cm'] >= DISTANCE_MIN_CM) & 
               (valid['distance_cm'] <= DISTANCE_MAX_CM) & 
               (valid['err_x_cm'].abs() <= lateral_thr_cm)).sum() / len(valid) * 100
    print(f"\n  [종합 (사후 계산)]")
    print(f"    거리 + 좌우 동시 만족: {both_ok:.1f}%")
    print(f"    비행 중 aligned=1:     {aligned_pct_det:.1f}%")
    diff = abs(aligned_pct_det - both_ok)
    if diff > 5:
        print(f"    ⚠️  차이 {diff:.1f}%p — 코드 상수가 분석 코드와 다를 수도")

    # PID 응답성
    cmd_roll  = valid["target_roll_us"]  - 1500
    cmd_pitch = valid["target_pitch_us"] - 1500
    err_x     = valid["err_x_px"]
    err_p     = valid["err_pitch_px"]
    att_roll  = valid["actual_roll_deg"]
    att_pitch = valid["actual_pitch_deg"]

    def safe_corr(a, b):
        if a.std() < 1e-6 or b.std() < 1e-6:
            return float("nan")
        return float(a.corr(b))

    r = {
        "label": label,
        "n_total": len(df),
        "n_valid": len(valid),
        "detect_pct": detect_pct,
        "aligned_pct_total": aligned_pct_total,
        "aligned_pct_det": aligned_pct_det,
        "dist_mean": dist_mean, "dist_std": dist_std,
        "dist_in_range": dist_in_range,
        "safe_pct": safe_pct,
        "front_collision_pct": front_collision_pct,
        "rear_collision_pct": rear_collision_pct,
        "lat_mean": lat_mean, "lat_std": lat_std,
        "lat_in_dynamic": lat_in_dynamic,
        "lat_in_agri": lat_in_agri,
        "both_ok": both_ok,
        "errx_cmdroll":     safe_corr(err_x, cmd_roll),
        "errp_cmdpitch":    safe_corr(err_p, cmd_pitch),
        "cmdroll_attroll":  safe_corr(cmd_roll, att_roll),
        "cmdpitch_attpitch":safe_corr(cmd_pitch, att_pitch),
        "cmdroll_attpitch": safe_corr(cmd_roll, att_pitch),
        "cmdpitch_attroll": safe_corr(cmd_pitch, att_roll),
        "cmd_roll_max":     float(cmd_roll.abs().max()),
        "cmd_pitch_max":    float(cmd_pitch.abs().max()),
        "att_roll_std":     float(att_roll.std()),
        "att_pitch_std":    float(att_pitch.std()),
    }

    print(f"\n  [PID 응답성]")
    print(f"    r(err_x,   cmd_roll)  = {r['errx_cmdroll']:+.3f}")
    print(f"    r(err_p,   cmd_pitch) = {r['errp_cmdpitch']:+.3f}")
    print(f"    r(cmd_roll,att_roll)  = {r['cmdroll_attroll']:+.3f}")
    print(f"    r(cmd_p,   att_pitch) = {r['cmdpitch_attpitch']:+.3f}")
    
    # 채널 스왑 검사
    main_r = abs(r['cmdroll_attroll']) + abs(r['cmdpitch_attpitch'])
    cross_r = abs(r['cmdroll_attpitch']) + abs(r['cmdpitch_attroll'])
    if cross_r > main_r:
        print(f"    ❌ 교차 상관({cross_r:.2f}) > 메인({main_r:.2f}) — 채널 스왑 의심!")
    
    # 자세 안정성
    print(f"\n  [자세 안정성]")
    print(f"    roll  std: {r['att_roll_std']:.2f}° (cmd max: ±{r['cmd_roll_max']:.0f}μs)")
    print(f"    pitch std: {r['att_pitch_std']:.2f}° (cmd max: ±{r['cmd_pitch_max']:.0f}μs)")
    if r['cmd_roll_max'] >= 95:
        print(f"    ⚠️  cmd_roll 포화 가능 (MAX_CORRECTION 100)")
    if r['cmd_pitch_max'] >= 95:
        print(f"    ⚠️  cmd_pitch 포화 가능")

    return r


def interpret(r):
    lines = []

    def tag_pct(v, good=70, ok=50):
        if v >= good: return "우수 ✓"
        if v >= ok: return "양호"
        if v >= 30: return "보통"
        return "미흡 ❌"
    
    lines.append("【1】정렬 성능")
    lines.append(f"  비행 중 aligned: {r['aligned_pct_det']:.1f}% (감지 중)")
    lines.append(f"    → {tag_pct(r['aligned_pct_det'])}")
    lines.append(f"  정렬 범위 만족 ({DISTANCE_MIN_CM:.0f}~{DISTANCE_MAX_CM:.0f}cm): {r['dist_in_range']:.1f}%")
    lines.append(f"  좌우 (동적):     {r['lat_in_dynamic']:.1f}%")
    lines.append(f"  좌우 (농업 ±40cm): {r['lat_in_agri']:.1f}%")
    
    lines.append("")
    lines.append(f"【1-2】안전 범위 (충돌 방지: {SAFETY_MIN_CM:.0f}~{SAFETY_MAX_CM:.0f}cm)")
    lines.append(f"  안전 만족: {r['safe_pct']:.1f}%")
    if r['front_collision_pct'] > 0:
        lines.append(f"  ⚠️  앞 충돌 위험 (<{SAFETY_MIN_CM:.0f}cm): {r['front_collision_pct']:.2f}%")
    if r['rear_collision_pct'] > 0:
        lines.append(f"  ⚠️  뒤 충돌 위험 (>{SAFETY_MAX_CM:.0f}cm): {r['rear_collision_pct']:.2f}%")
    if r['front_collision_pct'] == 0 and r['rear_collision_pct'] == 0:
        lines.append(f"  ✓ 충돌 위험 없음")
    
    lines.append("")
    lines.append("【2】위치 분포")
    lines.append(f"  거리 평균: {r['dist_mean']:.1f} ± {r['dist_std']:.1f} cm (목표 {TARGET_DISTANCE_CM:.0f}cm)")
    if abs(r['dist_mean'] - TARGET_DISTANCE_CM) > 5:
        lines.append(f"    ⚠️  목표에서 {abs(r['dist_mean']-TARGET_DISTANCE_CM):.1f}cm 편향")
    lines.append(f"  좌우 평균: {r['lat_mean']:+.1f} ± {r['lat_std']:.1f} cm")
    if abs(r['lat_mean']) > 3:
        side = "오른쪽" if r['lat_mean'] > 0 else "왼쪽"
        lines.append(f"    ⚠️  {side}으로 {abs(r['lat_mean']):.1f}cm 편향")
    
    lines.append("")
    lines.append("【3】PID 응답성")
    def tag_corr(v):
        if v != v: return "측정 불가"
        if v > 0.5: return "정상 ✓"
        if v > 0.2: return "약함"
        if v > -0.2: return "거의 없음 ⚠️"
        return "반대 ❌"
    lines.append(f"  r(err_x,   cmd_roll)  = {r['errx_cmdroll']:+.3f}  → {tag_corr(r['errx_cmdroll'])}")
    lines.append(f"  r(err_p,   cmd_pitch) = {r['errp_cmdpitch']:+.3f}  → {tag_corr(r['errp_cmdpitch'])}")
    lines.append(f"  r(cmd_roll,att_roll)  = {r['cmdroll_attroll']:+.3f}  → {tag_corr(r['cmdroll_attroll'])}")
    
    lines.append("")
    lines.append("【4】자세 안정성")
    lines.append(f"  roll std: {r['att_roll_std']:.2f}°  /  pitch std: {r['att_pitch_std']:.2f}°")
    if r['att_roll_std'] < 1.0 and r['att_pitch_std'] < 1.0:
        lines.append(f"    → 매우 안정 ✓")
    elif r['att_roll_std'] < 2.0:
        lines.append(f"    → 안정")
    else:
        lines.append(f"    → 진동 발생 ⚠️")
    
    lines.append("")
    lines.append("【5】감지 성능")
    lines.append(f"  감지율: {r['detect_pct']:.1f}%")
    if r['detect_pct'] < 80:
        lines.append(f"    ⚠️  감지 실패 많음 — 검출 파라미터 또는 조명 확인")
    
    return "\n".join(lines)


# ─────────────────────────────────────────────
# 그래프
# ─────────────────────────────────────────────
def plot_timeseries_cm(df, out_path, title="v14"):
    """시계열 — 거리, 좌우(cm), 명령, 자세."""
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)

    valid = df[(df["err_x_px"] != 0) | (df["err_pitch_px"] != 0)].copy()
    lateral_thr_cm = err_x_px_to_cm(valid['lateral_thr_px'], valid['marker_px'])

    # 1. 거리 (앞뒤)
    # 안전 한계 (빨간색)
    axes[0].axhline(SAFETY_MIN_CM, color='red', lw=1.5, ls='-', alpha=0.7,
                    label=f'안전 한계 ({SAFETY_MIN_CM:.0f}/{SAFETY_MAX_CM:.0f}cm)')
    axes[0].axhline(SAFETY_MAX_CM, color='red', lw=1.5, ls='-', alpha=0.7)
    # 정렬 범위 (녹색)
    axes[0].axhspan(DISTANCE_MIN_CM, DISTANCE_MAX_CM, alpha=0.15, color='green',
                    label=f"정렬 ({DISTANCE_MIN_CM:.0f}~{DISTANCE_MAX_CM:.0f}cm)")
    axes[0].axhline(TARGET_DISTANCE_CM, color='green', lw=1, ls='--', label=f"목표 {TARGET_DISTANCE_CM:.0f}cm")
    axes[0].plot(valid["elapsed_s"], valid["distance_cm"], color='C2', lw=0.8)
    axes[0].set_title(f"{title} — 거리 (앞뒤)")
    axes[0].set_ylabel("거리 (cm)")
    axes[0].set_ylim(5, 80)
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].grid(alpha=0.3)

    # 2. 좌우 (cm) + 동적 임계
    axes[1].fill_between(valid["elapsed_s"], -lateral_thr_cm, lateral_thr_cm,
                          alpha=0.15, color='blue', label=f"동적 임계 (가용의 {LATERAL_RATIO*100:.0f}%)")
    axes[1].axhline(0, color='gray', lw=0.5)
    axes[1].plot(valid["elapsed_s"], valid["err_x_cm"], color='C0', lw=0.8)
    axes[1].set_title(f"{title} — 좌우 (err_x)")
    axes[1].set_ylabel("err_x (cm)")
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(alpha=0.3)

    # 3. 명령
    cmd_roll  = df["target_roll_us"]  - 1500
    cmd_pitch = df["target_pitch_us"] - 1500
    axes[2].plot(df["elapsed_s"], cmd_roll,  label="cmd_roll",  lw=0.7)
    axes[2].plot(df["elapsed_s"], cmd_pitch, label="cmd_pitch", lw=0.7, alpha=0.7)
    axes[2].axhline(0, color='gray', lw=0.5)
    axes[2].set_title(f"{title} — PID 명령 (PWM offset)")
    axes[2].set_ylabel("μs")
    axes[2].legend(loc='upper right', fontsize=8)
    axes[2].grid(alpha=0.3)

    # 4. 실제 자세
    axes[3].plot(df["elapsed_s"], df["actual_roll_deg"],  label="roll",  lw=0.7)
    axes[3].plot(df["elapsed_s"], df["actual_pitch_deg"], label="pitch", lw=0.7, alpha=0.7)
    axes[3].axhline(0, color='gray', lw=0.5)
    axes[3].set_title(f"{title} — 실제 자세")
    axes[3].set_xlabel("time (s)")
    axes[3].set_ylabel("deg")
    axes[3].legend(loc='upper right', fontsize=8)
    axes[3].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    print(f"[FIG] {out_path}")


def plot_operational_box(df, out_path, title="v14"):
    """좌우 cm vs 거리 cm 산점도 — 운영 박스 안에 머무는지."""
    fig, ax = plt.subplots(figsize=(9, 8))

    valid = df[(df["err_x_px"] != 0) | (df["err_pitch_px"] != 0)].copy()
    
    # 시간에 따른 색상
    sc = ax.scatter(valid["err_x_cm"], valid["distance_cm"],
                    c=valid["elapsed_s"], cmap='viridis',
                    s=8, alpha=0.6)
    plt.colorbar(sc, ax=ax, label='time (s)')

    # 목표 위치
    ax.scatter([0], [TARGET_DISTANCE_CM], marker='+', s=300, c='red', 
               linewidths=2, label='목표 (0cm, 40cm)', zorder=5)

    # 안전 범위 (충돌 한계, 빨간 영역)
    ax.axhspan(0, SAFETY_MIN_CM, alpha=0.20, color='red', 
               label=f'충돌 위험 (<{SAFETY_MIN_CM:.0f}cm 또는 >{SAFETY_MAX_CM:.0f}cm)')
    ax.axhspan(SAFETY_MAX_CM, 100, alpha=0.20, color='red')
    ax.axhline(SAFETY_MIN_CM, color='red', lw=1.5, ls='-', alpha=0.8)
    ax.axhline(SAFETY_MAX_CM, color='red', lw=1.5, ls='-', alpha=0.8)
    
    # 거리 허용 범위 (수평 라인)
    ax.axhline(DISTANCE_MIN_CM, color='green', lw=1, ls='--', alpha=0.7,
               label=f'정렬 범위 ({DISTANCE_MIN_CM:.0f}~{DISTANCE_MAX_CM:.0f}cm)')
    ax.axhline(DISTANCE_MAX_CM, color='green', lw=1, ls='--', alpha=0.7)
    
    # 동적 좌우 임계는 거리에 따라 다른 모양 — 곡선
    # 임계 = (640 - marker_px) / 2 × LATERAL_RATIO × cm_per_px
    # marker_px = f × 30 / distance_mm
    # 임계 (cm) = (640 - f×30/distance_mm) / 2 × LATERAL_RATIO × distance_mm / f / 10
    distances = np.linspace(15, 70, 100)
    distance_mm = distances * 10
    marker_pxs = FOCAL_LENGTH_PX * MARKER_SIZE_MM / distance_mm
    available_pxs = (CAM_W - marker_pxs) / 2
    threshold_cms = available_pxs * LATERAL_RATIO * distance_mm / FOCAL_LENGTH_PX / 10
    
    ax.plot(threshold_cms, distances, color='blue', lw=1, ls=':', alpha=0.7,
            label=f'좌우 임계 (가용의 {LATERAL_RATIO*100:.0f}%)')
    ax.plot(-threshold_cms, distances, color='blue', lw=1, ls=':', alpha=0.7)
    
    # 농업 임계
    ax.axvline(AGRICULTURAL_LATERAL_CM, color='red', lw=1, ls='-', alpha=0.5,
               label=f'농업 임계 (±{AGRICULTURAL_LATERAL_CM:.0f}cm)')
    ax.axvline(-AGRICULTURAL_LATERAL_CM, color='red', lw=1, ls='-', alpha=0.5)

    ax.axvline(0, color='gray', lw=0.5)
    ax.set_xlabel("좌우 err_x (cm)")
    ax.set_ylabel("거리 (cm)")
    ax.set_title(f"{title} — 운영 위치 분포\n[목표: (0, {TARGET_DISTANCE_CM:.0f}), 운영 박스: ±dynamic × {DISTANCE_MIN_CM:.0f}~{DISTANCE_MAX_CM:.0f}cm]")
    ax.legend(loc='lower right', fontsize=9)
    ax.grid(alpha=0.3)
    ax.set_xlim(-50, 50)
    ax.set_ylim(5, 80)  # 안전 한계 표시 가능하게
    ax.invert_yaxis()  # 가까운 거리가 위로

    plt.tight_layout()
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    print(f"[FIG] {out_path}")


def plot_distance_distribution(df, out_path, title="v14"):
    """거리 + 좌우 cm 히스토그램."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    valid = df[(df["err_x_px"] != 0) | (df["err_pitch_px"] != 0)].copy()

    # 거리 히스토그램
    axes[0].axvline(SAFETY_MIN_CM, color='red', lw=1.5, ls='-', alpha=0.7,
                    label=f'안전 한계 ({SAFETY_MIN_CM:.0f}/{SAFETY_MAX_CM:.0f}cm)')
    axes[0].axvline(SAFETY_MAX_CM, color='red', lw=1.5, ls='-', alpha=0.7)
    axes[0].axvspan(DISTANCE_MIN_CM, DISTANCE_MAX_CM, alpha=0.15, color='green',
                    label=f'정렬 ({DISTANCE_MIN_CM:.0f}~{DISTANCE_MAX_CM:.0f}cm)')
    axes[0].axvline(TARGET_DISTANCE_CM, color='green', lw=1.5, ls='--', label=f'목표 {TARGET_DISTANCE_CM:.0f}cm')
    axes[0].hist(valid["distance_cm"], bins=40, color='C2', alpha=0.7, edgecolor='black')
    axes[0].set_title(f"{title} — 거리 분포")
    axes[0].set_xlabel("거리 (cm)")
    axes[0].set_ylabel("프레임 수")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    # 좌우 히스토그램
    axes[1].axvspan(-AGRICULTURAL_LATERAL_CM, AGRICULTURAL_LATERAL_CM, alpha=0.10, color='orange',
                    label=f'농업 임계 (±{AGRICULTURAL_LATERAL_CM:.0f}cm)')
    axes[1].axvline(0, color='gray', lw=0.5)
    axes[1].hist(valid["err_x_cm"], bins=40, color='C0', alpha=0.7, edgecolor='black')
    axes[1].set_title(f"{title} — 좌우 분포")
    axes[1].set_xlabel("err_x (cm)")
    axes[1].set_ylabel("프레임 수")
    axes[1].legend(fontsize=9)
    axes[1].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    print(f"[FIG] {out_path}")


def plot_command_response(df, out_path, title="v14"):
    """명령 vs 자세 산점도 — 부호 매핑."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    valid = df[(df["err_x_px"] != 0) | (df["err_pitch_px"] != 0)]
    cmd_roll  = valid["target_roll_us"]  - 1500
    cmd_pitch = valid["target_pitch_us"] - 1500

    axes[0].scatter(cmd_roll, valid["actual_roll_deg"], s=4, alpha=0.4, label="cmd_roll vs att_roll")
    axes[0].scatter(cmd_roll, valid["actual_pitch_deg"], s=4, alpha=0.3, c='r', label="cross (att_pitch)")
    axes[0].axhline(0, color='gray', lw=0.5); axes[0].axvline(0, color='gray', lw=0.5)
    axes[0].set_title(f"{title} — Roll 명령 응답")
    axes[0].set_xlabel("cmd_roll (μs)")
    axes[0].set_ylabel("attitude (deg)")
    axes[0].legend(fontsize=8); axes[0].grid(alpha=0.3)

    axes[1].scatter(cmd_pitch, valid["actual_pitch_deg"], s=4, alpha=0.4, label="cmd_pitch vs att_pitch")
    axes[1].scatter(cmd_pitch, valid["actual_roll_deg"], s=4, alpha=0.3, c='r', label="cross (att_roll)")
    axes[1].axhline(0, color='gray', lw=0.5); axes[1].axvline(0, color='gray', lw=0.5)
    axes[1].set_title(f"{title} — Pitch 명령 응답")
    axes[1].set_xlabel("cmd_pitch (μs)")
    axes[1].set_ylabel("attitude (deg)")
    axes[1].legend(fontsize=8); axes[1].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    print(f"[FIG] {out_path}")


def plot_time_aligned(df, out_path, title="v14"):
    """시간대별 정렬률 — 10초 구간."""
    fig, ax = plt.subplots(figsize=(11, 4))

    df = df.copy()
    df['time_bin'] = (df['elapsed_s'] // 10) * 10
    grouped = df.groupby('time_bin').agg(
        aligned_pct=('aligned', lambda x: x.mean() * 100),
        n=('aligned', 'count')
    ).reset_index()
    
    colors = ['green' if v >= 70 else 'orange' if v >= 50 else 'red' 
              for v in grouped['aligned_pct']]
    
    ax.bar(grouped['time_bin'], grouped['aligned_pct'], width=8,
           color=colors, alpha=0.7, edgecolor='black')
    ax.axhline(70, color='green', lw=1, ls='--', alpha=0.5, label='우수 (70%)')
    ax.axhline(50, color='orange', lw=1, ls='--', alpha=0.5, label='양호 (50%)')
    
    for tb, p in zip(grouped['time_bin'], grouped['aligned_pct']):
        ax.text(tb, p + 2, f'{p:.0f}%', ha='center', fontsize=9)
    
    ax.set_xlabel("time (s, 10s intervals)")
    ax.set_ylabel("정렬률 (%)")
    ax.set_title(f"{title} — 시간대별 정렬률")
    ax.set_ylim(0, 110)
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    print(f"[FIG] {out_path}")


def write_report(r, log_path, out_path, generated_pngs):
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("=" * 60 + "\n")
        f.write(f"  v14 비행 로그 분석 리포트 (cm 기반)\n")
        f.write("=" * 60 + "\n")
        f.write(f"분석 시각  : {_dt.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"로그 파일  : {log_path}\n")
        f.write(f"환경       : 마커 {MARKER_SIZE_MM:.0f}mm, focal {FOCAL_LENGTH_PX:.1f}px\n")
        f.write(f"            정렬 {DISTANCE_MIN_CM:.0f}~{DISTANCE_MAX_CM:.0f}cm (목표 {TARGET_DISTANCE_CM:.0f}cm)\n")
        f.write(f"            안전 {SAFETY_MIN_CM:.0f}~{SAFETY_MAX_CM:.0f}cm (드론 {DRONE_LENGTH_CM:.0f}cm + 마진 {SAFETY_MARGIN_CM:.0f}cm)\n")
        f.write(f"            좌우 임계: 가용의 {LATERAL_RATIO*100:.0f}% (동적)\n")
        f.write(f"전체 샘플  : {r['n_total']}  /  마커 감지: {r['n_valid']} ({r['detect_pct']:.1f}%)\n")
        f.write("\n")

        f.write("─" * 60 + "\n")
        f.write("  핵심 결과\n")
        f.write("─" * 60 + "\n")
        f.write(f"정렬 성공률 (감지 중):    {r['aligned_pct_det']:5.1f}%\n")
        f.write(f"정렬 성공률 (전체):       {r['aligned_pct_total']:5.1f}%\n")
        f.write(f"정렬 범위 만족 ({DISTANCE_MIN_CM:.0f}~{DISTANCE_MAX_CM:.0f}cm): {r['dist_in_range']:5.1f}%\n")
        f.write(f"안전 범위 만족 ({SAFETY_MIN_CM:.0f}~{SAFETY_MAX_CM:.0f}cm): {r['safe_pct']:5.1f}%  ⭐\n")
        if r['front_collision_pct'] > 0:
            f.write(f"  ⚠️  앞 충돌 위험 (<{SAFETY_MIN_CM:.0f}cm): {r['front_collision_pct']:.2f}%\n")
        if r['rear_collision_pct'] > 0:
            f.write(f"  ⚠️  뒤 충돌 위험 (>{SAFETY_MAX_CM:.0f}cm): {r['rear_collision_pct']:.2f}%\n")
        f.write(f"좌우 동적 임계 만족:      {r['lat_in_dynamic']:5.1f}%\n")
        f.write(f"좌우 농업 임계 만족:      {r['lat_in_agri']:5.1f}%\n")
        f.write(f"둘 다 만족 (사후):         {r['both_ok']:5.1f}%\n")
        f.write("\n")
        f.write(f"거리:    {r['dist_mean']:5.1f} ± {r['dist_std']:.1f} cm (목표 {TARGET_DISTANCE_CM:.0f}cm)\n")
        f.write(f"좌우:    {r['lat_mean']:+5.1f} ± {r['lat_std']:.1f} cm (목표 0cm)\n")
        f.write("\n")

        f.write("─" * 60 + "\n")
        f.write("  PID 응답성\n")
        f.write("─" * 60 + "\n")
        f.write(f"r(err_x,   cmd_roll)  = {r['errx_cmdroll']:+.3f}\n")
        f.write(f"r(err_p,   cmd_pitch) = {r['errp_cmdpitch']:+.3f}\n")
        f.write(f"r(cmd_roll,att_roll)  = {r['cmdroll_attroll']:+.3f}\n")
        f.write(f"r(cmd_p,   att_pitch) = {r['cmdpitch_attpitch']:+.3f}\n")
        f.write(f"교차 (스왑 검사):\n")
        f.write(f"  r(cmd_roll, att_pitch) = {r['cmdroll_attpitch']:+.3f}\n")
        f.write(f"  r(cmd_pitch,att_roll)  = {r['cmdpitch_attroll']:+.3f}\n")
        f.write("\n")

        f.write("─" * 60 + "\n")
        f.write("  자동 해석\n")
        f.write("─" * 60 + "\n")
        f.write(interpret(r))
        f.write("\n\n")

        f.write("─" * 60 + "\n")
        f.write("  생성된 그래프\n")
        f.write("─" * 60 + "\n")
        for p in generated_pngs:
            f.write(f"  - {p}\n")

    print(f"\n[REPORT] {out_path}")


def main():
    ap = argparse.ArgumentParser(description="v14+ 비행 로그 분석 (cm 기반)")
    ap.add_argument("log", nargs="?", help="분석할 CSV 파일")
    ap.add_argument("--baseline", action="store_true", help="baseline_v14 로그 자동 선택")
    ap.add_argument("--full", action="store_true", 
                    help="전체 비행 분석 (기본은 첫 정렬 후 60s만)")
    ap.add_argument("--duration", type=float, default=60.0,
                    help="측정 구간 길이 (기본 60s)")
    args = ap.parse_args()

    if args.log:
        log_path = args.log
    elif args.baseline:
        # logs/baseline*/ 안의 모든 realtime CSV → 최신
        all_files = glob.glob("logs/baseline*/loiter*realtime*.csv")
        if all_files:
            log_path = max(all_files, key=os.path.getmtime)
            print(f"[AUTO baseline] {log_path}")
        else:
            log_path = None
    else:
        # logs/ 안의 모든 정렬 데이터 → 최신 (mtime 기준)
        # baseline_* 폴더는 제외
        all_files = []
        all_files.extend(glob.glob("logs/v*/loiter_v*_realtime_*.csv"))
        all_files.extend(glob.glob("logs/poster/loiter_poster_realtime_*.csv"))
        all_files.extend(glob.glob("logs/loiter_*_realtime_*.csv"))
        all_files = [f for f in all_files if 'baseline' not in f.lower()]
        
        if all_files:
            log_path = max(all_files, key=os.path.getmtime)
            folder = os.path.basename(os.path.dirname(log_path))
            print(f"[AUTO {folder}] {log_path}")
        else:
            log_path = None
    
    if not log_path:
        print("[ERROR] 로그 파일을 찾을 수 없음")
        sys.exit(1)
    
    measurement_only = not args.full
    if measurement_only:
        print(f"\n🎯 분석 범위: 첫 정렬 후 {args.duration:.0f}초 (--full로 전체 분석)")
    else:
        print(f"\n🎯 분석 범위: 전체 비행")
    
    print(f"\n📐 환경:")
    print(f"   마커: {MARKER_SIZE_MM:.0f}mm  focal: {FOCAL_LENGTH_PX:.1f}px")
    print(f"   목표 거리: {TARGET_DISTANCE_CM:.0f}cm  허용: {DISTANCE_MIN_CM:.0f}~{DISTANCE_MAX_CM:.0f}cm")
    print(f"   좌우 임계: 가용의 {LATERAL_RATIO*100:.0f}% (동적)")
    print(f"   농업 임계: ±{AGRICULTURAL_LATERAL_CM:.0f}cm")

    df = load_log(log_path, measurement_only=measurement_only, 
                  measurement_duration=args.duration)
    
    # 라벨 자동 추출 (v14, v15, v16, v17, baseline 등)
    import re
    m = re.search(r'(baseline_v\d+|v\d+)', os.path.basename(log_path))
    label = m.group(1) if m else "unknown"
    result = analyze(df, label)
    
    if not result:
        sys.exit(1)
    
    # 그래프 생성
    os.makedirs(OUT_DIR, exist_ok=True)
    base = os.path.splitext(os.path.basename(log_path))[0]
    
    ts_path   = os.path.join(OUT_DIR, f"{base}_timeseries_cm.png")
    box_path  = os.path.join(OUT_DIR, f"{base}_operational_box.png")
    hist_path = os.path.join(OUT_DIR, f"{base}_distributions.png")
    cmd_path  = os.path.join(OUT_DIR, f"{base}_cmd_response.png")
    time_path = os.path.join(OUT_DIR, f"{base}_time_aligned.png")
    
    plot_timeseries_cm(df, ts_path, label)
    plot_operational_box(df, box_path, label)
    plot_distance_distribution(df, hist_path, label)
    plot_command_response(df, cmd_path, label)
    plot_time_aligned(df, time_path, label)
    
    # 자동 해석
    print("\n" + "=" * 60)
    print("  📝 자동 해석")
    print("=" * 60)
    print(interpret(result))

    # 리포트 저장
    report_path = os.path.join(OUT_DIR, f"{base}_report.txt")
    write_report(result, log_path, report_path,
                 [ts_path, box_path, hist_path, cmd_path, time_path])


if __name__ == "__main__":
    main()