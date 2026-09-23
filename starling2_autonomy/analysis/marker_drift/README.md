# marker_drift — 마커 기반 드리프트 측정 후처리

[`docs/09`](../../docs/09_marker_drift_experiment_plan.md) 실험의 로컬 PC 분석 파이프라인.
기체 없이 돈다. 단위테스트 65항목은 합성 렌더링 이미지로 실제 검출기 경로까지 검증한다.

## 설치

```bash
cd starling2_autonomy
python -m venv .venv
.venv/Scripts/python -m pip install numpy opencv-python pupil-apriltags pandas matplotlib scipy pyyaml   # Windows
# .venv/bin/python ...                                                                              # Linux/mac
.venv/Scripts/python analysis/test_marker_drift.py        # 65/65 passed 확인
```

명령은 `analysis/` 폴더에서 `python -m marker_drift <command>` 로 실행한다.

## 흐름

```
① coverage   마커 크기·간격·이격이 화각을 채우는지 점검 (설치 전, 카메라 파일 없이도 됨)
② calibrate  hires 카메라 내부 파라미터 (기체에 파일이 없을 때)
③ detect     비행 영상 -> 프레임별 태그 검출 + 카메라 M 포즈 CSV
④ noise      정지 촬영 검출 CSV -> 프레임 간 잡음 -> 정렬 허용오차 제안 (docs/09 §5.3)
⑤ analyze    검출 CSV + 기체 로그 -> 드리프트 시계열·요약·그림
⑥ compare    비행 요약 여러 개 -> 조건별 상자그림, Welch t, 표본 수 산정 (docs/09 §5.4)
```

### ① 커버리지

```bash
python -m marker_drift coverage --standoff 0.5 --altitude 0.6                 # hires 120x93.5도, 1280x800 가정
python -m marker_drift coverage --standoff 0.5 --altitude 0.6 --tag-size 0.10 # 10 cm 태그면?
python -m marker_drift coverage --intrinsics hires.yaml --layout my.yaml --standoff 0.5 --altitude 2.0 -v
```

기본 배치(7 cm, 0.5 m 간격, 이격 50 cm, 120° 화각, 1280×800) 결과: 경로 100 %에서 태그 2개 이상, 93 %에서 3개 이상,
한 변 52 px(벽이 화상면과 평행해 위치와 무관). 1 m 간격이면 2개 이상은 71 %로 떨어진다.

**태그 2개 이상이 보이는 프레임은 한 몸 PnP**(`geometry.solve_multi_tag_camera_pose`, `joint_*` 컬럼)로 푼다.
7 cm 태그 하나짜리 PnP 는 정면 근처에서 yaw 가 최대 1.8° 튀지만(평면 포즈 모호성), 두 태그를 묶으면 0.3° 이내다.
`per_frame_pose` 는 joint 가 있으면 그것을 쓰고(`pose_src="joint"`), 태그별 위치 불일치 `spread_m` 은 진단용으로 남긴다.

### ② 캘리브레이션

```bash
python -m marker_drift calibrate --video cal.mp4 --board 9x6 --square 0.025 --out hires.yaml
python -m marker_drift calibrate --frames-dir cal_frames/ --board 6x5 --square 0.0655 --out tracking_front.yaml --fisheye
```

기체의 `/data/modalai/opencv_<cam>_intrinsics.yml` (OpenCV FileStorage) 도 그대로 읽는다.

### ③ 검출

```bash
# 동영상 (프레임 시각 = t0 + idx/fps)
python -m marker_drift detect --video flight.mp4 --intrinsics hires.yaml --layout layout.yaml --out det.csv --fps 30 --t0 12345.67
# voxl-logger 이미지 폴더 + 타임스탬프 CSV
python -m marker_drift detect --frames-dir hires_small_color/ --timestamps hires_small_color.csv \
    --ts-col timestamp_ns --ts-scale 1e-9 --intrinsics hires.yaml --out det.csv
```

출력 한 행 = 한 프레임의 한 태그. `cam_x, cam_y, cam_z, cam_yaw_deg` 가 M 프레임 카메라 포즈다.
`--show` 로 검출 상자를 보면서 돌릴 수 있다.

### ④ 잡음 → 허용오차

```bash
python -m marker_drift noise --detections still_30s.csv        # 손으로 들거나 정지 호버 30 s
```

`noise_yaw_deg` (프레임 간 yaw 표준편차) 와 `align_tol_deg_suggested = max(1°, 3σ)` 를 출력한다.

### ⑤ 분석

```bash
python -m marker_drift analyze --detections det.csv --log px4_reach_sync_XXXX.csv --log-format v14 \
    --hover 12350 12360 --move 12360 12378 --end 12378 12383 --name A1 --condition A --out-dir out/
```

- `--log-format v14` 는 `flight/path_flight_phase1_v14.py --csv` 출력을 읽는다. 시각은 `voxl_<pipe>_ts_ms/1000`
  (VOXL 단조시계 = MPA 카메라 타임스탬프와 같은 시계). 영상 시각이 다른 시계면 `--t-offset` (로그 t − 영상 t).
- 정합은 `--hover` 창(정렬 완료 직후 호버)에서 **heading 방식**(태그 yaw − 기체 yaw 의 원형평균 + 평행이동)으로 한다.
  `--align procrustes` 는 기체 yaw 로그가 없을 때만 쓴다. 창 안의 드리프트 회전 성분을 흡수하므로 과소평가한다.
- 산출: `<name>_timeseries.csv` (e_x, e_y, e_z, e_yaw, e_h, d_* 등), `<name>_summary.csv` (E_*, 드리프트율,
  호버 드리프트, 잡음, 검출률, 정합 파라미터), `<name>.png` (평면 궤적 / 진행거리 대 드리프트 / yaw / 수평 오차).

### ⑥ 비교

```bash
python -m marker_drift compare out/*_summary.csv --baseline A --delta 0.03 --out-dir out/compare
```

조건별 평균·표준편차·Welch t·p 와 "조건 간 차이 Δ=3 cm 를 검출하려면 그룹당 n" 을 표로 낸다.
표본 수 근사: n = 2 (z₁₋α/₂ + z_power)² (s/Δ)² → α .05, 검정력 .8 에서 15.7 (s/Δ)².

## 좌표계 (geometry.py)

| 프레임 | 정의 |
|---|---|
| **M** | 마커 0 중심 원점. x = 마커 열 방향(진행), y = 벽 안쪽, z = 위. 마커는 y=0 평면, 기체는 y<0 |
| **T** | AprilTag 규약. x 오른쪽, y 아래, z 태그 안쪽. 벽에 똑바로 붙으면 `R_M_T = [[1,0,0],[0,0,1],[0,-1,0]]` |
| **C** | OpenCV. x 오른쪽, y 아래, z 전방 |
| **L** | 기체 로컬 NED. ENU 로 바꾼 뒤 컴퍼스 회전 + 평행이동으로 M 에 정합 |

정면 정렬 yaw = 광축을 수평면에 투영해 +y 에서 +x 쪽으로 잰 각. **0 = 벽 정면, 양수 = 오른쪽으로 돌아감.**
PnP 는 `cv2.SOLVEPNP_IPPE_SQUARE` (코너 순서 좌하·우하·우상·좌상 = AprilTag 순서).

## 기체 확인 후 손볼 곳

- `detect --timestamps`: voxl-logger 의 카메라 타임스탬프 파일 형식(컬럼명·단위)에 맞춰 `--ts-col/--ts-scale`.
- `logs.load_generic_csv`: voxl-logger 의 `ov` / `px4_vehicle_local_position` 기록을 CSV 로 바꾼 뒤 컬럼 매핑.
- hires intrinsics 파일 유무. 없으면 ② 로 만든다.
- 카메라-기체 중심 오프셋은 yaw 가 일정한 비행에서는 상수 평행이동으로 정합에 흡수된다. yaw 가 크게 변하는
  비행을 분석하려면 `drift.py` 에 lever-arm 보정을 추가한다.

## 손 촬영 검증 절차 (비행 전)

1. 태그 2~3개를 실측 간격으로 벽에 붙이고 `layout.yaml` 에 좌표를 적는다.
2. 카메라(핸드폰이든 기체 hires 든)를 캘리브레이션한다(②).
3. 정지 30 s 촬영 → ③ → ④ 로 잡음과 허용오차를 본다.
4. 벽과 50 cm 를 유지하며 천천히 옆으로 걸어가며 촬영 → ③ → `per_frame_pose` 의 `spread_m` (태그 2개가 동시에
   보일 때 두 태그에서 구한 위치의 불일치) 로 마커 좌표 실측 오차를 검증한다. 수 mm 이내면 정상.
