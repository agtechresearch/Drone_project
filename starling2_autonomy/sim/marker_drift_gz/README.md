# 마커 드리프트 실험 — SITL 선검증 (drone_sim_gz, gz Harmonic)

실기 비행 전에 `flight/marker_drift_flight.py` + `flight/s2flight/` 를 연구실 서버의 시뮬레이션에서 먼저 돌린다
(2026-11-09 결정: 프롭 탈착이 필요한 지상 arm 시험은 하지 않고, 시뮬 → 실기 짧은 호버 시험 → 본실험 순).
시뮬 기반은 리포의 `drone_sim_gz/` (PX4 v1.15.4 SITL + gz Harmonic, 컨테이너 `starling-gz` / `starling-flight`; 2026-11-10 main 에 병합됨).
**그쪽 파일은 수정하지 않는다** — 인스턴스 2~5 도구를 호출만 한다.

**검증 상태 (2026-11-10, 연구실 서버 agtechlab 인스턴스 2~4):** 정상 3회(T·A) + 오류 주입 6종 + 2열 동일 ID 월드 A/C + 시뮬 파일럿 A·B·C×3 = 20회 통과
(A2 1회 `NO_SETPOINT_SET` 실패 → 재시도 추가 후 재실행 통과). 코드 바닥 잡음 settled drift s ≈ 0.5 cm. 상세·수치는 `docs/12_marker_drift_experiment_log.md`.

## 무엇이 어떻게 대체되는가

| 실기 | 시뮬 | 구현 |
|---|---|---|
| `voxl-inspect-pose -n px4_vehicle_local_position` (도달 판정) | MAVSDK 텔레메트리 (`position_velocity_ned` + `attitude_euler`) | `s2flight.sim.sim_pose_task`, `--sim` |
| `/run/mpa/tag_detections` (voxl-tag-detector) | `sim_tag_detect.py`: `front_camera_real`(1280×800, 실기 tracking_front 화각) → pupil_apriltags → UDP JSON | `s2flight.sim.UdpTagFeed`, `--sim-tag-port` |
| voxl-logger | 없음 (진실값은 gz `gt_logger.py` 의 `gt.csv`) | `--sim` 이면 생략 |
| 현수막 + 태그 0~9 (20 cm, 0.5 m 간격, 이격 70 cm; 2026-11-10 실험장) | `drift_banner` 모델·월드 | `make_banner_world.py` |
| RC OFFBOARD 스위치 | 불필요 | — |

비행 로직(`vehicle`, `align_stage`, 지오펜스, 기록)은 실기와 **같은 코드**가 돈다. 다른 것은 입력 어댑터뿐이다 (v13 의 `--sim` 과 같은 접근).

## 서버 배치

```
~/starling_sim/                       (서버 레이아웃; drone_sim_gz/README.md "경로 주의")
├── repo_export/gz, scripts/          = drone_sim_gz/gz, scripts   (기본 월드 qvix.sdf, 모델)
├── v3_marker/                        = drone_sim_gz/v3_marker     (인스턴스 도구, 태그 이미지, starling2_tof 모델)
├── PX4-Autopilot/                    (starling-gz 안에서 ~/PX4-Autopilot)
└── marker_drift/                     ← 이 실험 (sync_to_server.sh 가 만든다)
    ├── flight/s2flight/, flight/marker_drift_flight.py
    ├── sim/                          = starling2_autonomy/sim/marker_drift_gz/  (이 폴더)
    ├── gz/                           make_banner_world.py 생성물 (models/drift_banner, worlds/drift_banner.sdf, banner_layout.json)
    └── logs/<시각>_iN/               run_sim_drift.sh 산출물
```

starling-flight 안에서는 `~/starling_sim` 이 `/home/dronesim/workspace` 로 보인다 (`docker/run_flight.sh`). 다른 레이아웃이면
`run_sim_drift.sh` 의 `SIM`, `MD`, `FLIGHT_WS` 환경변수를 맞춘다.

## 절차

```bash
# PC → 서버 (flight/ 와 sim/ 만; 기존 gz/, logs/ 는 그대로)
SIM_HOST=user@server ./sim/marker_drift_gz/sync_to_server.sh

# 서버 호스트
cd ~/starling_sim/marker_drift
python3 sim/make_banner_world.py --upper         # 태그 0~9 @ z 0.60 + 상단열(같은 ID 0~9) @ 2.0 (2026-11-10 실험장). --ids/--upper-ids 로 변경 가능
docker exec starling-gz ~/starling_sim/marker_drift/sim/install_banner_world.sh   # PX4 gz 트리에 모델·월드 복사 (빌드 없음)

# 1) 패키지 점검 비행: 이륙 → 좌회전 → 0.5 m 왕복 → 착륙 (태그 정렬 없음)
sim/run_sim_drift.sh --no-align --distance 0.5 --hover-ref 3 --hover-far 2 --hover-return 2 --condition T

# 2) 정렬 포함 본 시퀀스 (A 조건, 0.6 m, 4.5 m 왕복)
sim/run_sim_drift.sh --condition A --run 1
# 3) 조건 C (상단열 월드: make_banner_world.py --upper 후 재설치). 상단열도 ID 0~10 이라 --start/end-tag 는 그대로
sim/run_sim_drift.sh --altitude 2.0 --condition C --run 1
```

`run_sim_drift.sh` 가 하는 일: 인스턴스 N(기본 2) 정지 → `run_wk_instance.sh N drift_banner headless 0,0,0.035,0,0,1.5708`
(starling2_tof, VIO 대역 relay, 북향 스폰) → `New NED origin` 대기 → starling-flight 에 `mavsdk_server -p 5005N udp://:1454N`
→ `gt_logger.py`(진실값) + `sim_tag_detect.py`(UDP 1468N) → `marker_drift_flight.py --sim --port 5005N --sim-tag-port 1468N -y ...`
→ 정지. 산출물은 `logs/<시각>_iN/` 에 `sitl.log mission.log gt.csv tags.csv tags_raw.jsonl tag_detect.log args.txt` 와 비행 기록
`md_<조건><회차>_<시각>_{sync,events,stages,align_log}.csv, _summary.json, _console.log`.

환경변수: `INSTANCE`(2~5) `WORLD` `CAM_TOPIC`(/front_camera_real; /front_camera 는 640×480 68°) `TAG_HZ`(5) `TAG_SIZE`(0.20)
`SPAWN` `RTF` `MISSION_TIMEOUT`(600) `KEEP_SITL=1`(끝나도 인스턴스 유지) `NO_TAGS=1`(검출기 없이, `--no-align` 자동).

## 오류 주입 (`INJECT`, vio_relay.py)

`run_wk_instance.sh` 의 VIO relay 가 `/tmp/wk_inject_N.json` 을 0.5 s 마다 읽어 PX4 로 가는 VIO 에 오차를 넣는다. `INJECT='<json>'` 로 넘기면
그 파일에 써 주고 산출물 `inject.json` 에 남긴다. 키: `rot_deg`(VIO 프레임 회전 °), `drift_n`/`drift_e`(m/s, 이륙 후 등속), `step_e`/`step_at`
(m, 이륙 후 s 에 점프), `noise_pos_m`/`noise_vel_m_s`(백색잡음), `noise_seed`.

```bash
INJECT='{"drift_n":-0.02}'            sim/run_sim_drift.sh --condition D --run 1   # 이동축 −2 cm/s 드리프트 → settled drift_along ≈ −0.02·t
INJECT='{"step_e":-0.3,"step_at":25}' sim/run_sim_drift.sh --condition S --run 1   # 25 s 에 VIO 0.3 m 점프 → 실제 이격 0.7→1.0, drift_standoff −0.30
INJECT='{"noise_pos_m":0.02,"noise_vel_m_s":0.05}' sim/run_sim_drift.sh --condition N --run 1
sim/run_sim_drift.sh --condition M --run 1 --end-tag 11 --max-extra 0.5          # 끝 태그 없음 → MAX_DIST → 그 자리 착륙
python3 sim/summarize_run.py logs/<시각>_iN                                        # 폴더 요약 (정지 사유·settled·err 통계·gt 범위)
python3 sim/pilot_stats.py logs/20261110_19*_i*                                   # 여러 비행 → 조건별 mean/s, n ≈ 16(s/Δ)², Welch t (docs/09 §5.4)
```
2026-11-10 결과(6 시나리오 모두 설계대로; 횡방향 점프 0.8 m 는 지오펜스 2.5 m 안이라 통과하므로 실기 통로에선 `--max-horizontal-dev` 를 조일 것):
`worklog/2026-11-10.md` 표.

## 화면으로 보기 (TurboVNC :1)

`run_sim_drift.sh` 는 headless 로 gz 서버만 띄운다. 서버가 뜬 뒤(sitl.log 에 `run_sitl_instance` 줄) 같은 파티션에 GUI 클라이언트만 붙이면
VNC 화면(`DISPLAY=:1`, 컨테이너에 X 소켓이 마운트돼 있다)에 Gazebo 창이 뜬다. 관전용이며 비행 로직에 영향 없다. 마우스로 회전·줌,
모델 우클릭 → Follow. 서버가 내려가면 창도 닫히므로 매 실행마다 다시 붙인다.

```bash
docker exec -d -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 -e GZ_PARTITION=starling_2 starling-gz gz sim -g   # 인스턴스 2
```

## 월드 기하 (`make_banner_world.py`)

- 기체 스폰 (0, 0) 북향(gz yaw 1.5708; drone_sim_gz 규칙 — 다른 방향 스폰은 착지 시 yaw 점프를 만든다).
- 현수막 평면 x = −0.70 (기체 왼쪽), 법선 +x. 태그 k 는 y = 0.5·k (0 → 0 m, 9 → 4.5 m), 중심 z 0.65 (실측 2026-11-10; 비행 고도 0.6). 검은 사각형 0.20 m, 인쇄 0.25 m.
- 흰 판 5.0 × 0.5 m 가 태그 뒤 4 mm 에 있다 (`--banner-h`, `--banner-margin`). `--upper` 면 **같은 ID 0~9** 가 z 2.00 에 같은 배치
  (2026-11-10 실험장 변경; 링크 이름 `tag_NN_r1`, layout 키 `NN_r1`). 비행 코드는 |y_cam| > 0.7 m 검출을 버려 다른 열을 섞지 않는다.
- 비행: 이륙 → 좌회전 90° (−x 를 봄 = 현수막 정면) → 정렬 → body 우측 = +y 로 4.5 m → 복귀. `banner_layout.json` 에 좌표가 남는다.

## 판정

- `mission.log` / `_summary.json`: `sequence.completed`, `align.state == ALIGNED`, `legs[*].status == STOPPED`(태그 9 / 태그 0 중앙에서 정지),
  `legs_settled[*].drift_along`(자기 추정 − 태그 실제; 시뮬 VIO 는 진실값 relay 라 0 근처가 정상), `center.ok`.
- `md_*_drift_log.csv`: 태그 프레임마다 `truth_along / own_along_b / err_along`. 시뮬에서는 err ≈ 0 이어야 코드가 맞는 것이다.
- `tags.csv`: 정렬 구간에 `n_det ≥ 2`. `tag_detect.log` 첫 줄에 `image 1280x800 fx=276.x`.
- 진실값: `gt.csv`(gz 오도메트리, sim 시각) 와 비행 `_sync.csv`(mono_time) 는 시계가 다르다. 이벤트 시각으로 맞추거나
  `tags.csv` 의 `true_x/true_y`(검출 프레임마다 진실 위치)를 쓴다. 자동 비교 스크립트는 아직 없다.
- 시뮬 결과는 코드 경로 검증이다. VIO 드리프트 자체는 실기에서만 의미가 있다 (시뮬 VIO 는 relay 로, 주입 없으면 진실값 + 잡음).

## 주의

- `wk_fly.sh` 등 다른 작업이 같은 인스턴스를 쓰면 충돌한다. 인스턴스 0/1 은 보호 대상이라 거부된다.
- 실시간 비율(RTF) 이 1 미만이면 비행 코드의 타임아웃(벽시계 기준)이 시뮬 시간보다 빨리 지난다. `--move-timeout` 등을 늘리거나
  `RTF=1` 을 지킨다. (wk_mission 의 `--sim-clock` 같은 시뮬 시계 연동은 아직 없다.)
- `front_camera_real` 은 `starling2_tof` 모델에만 있다. `run_wk_instance.sh` 가 그 모델을 쓴다 (`run_sitl_instance.sh` 는 아님).
- PC(Windows) 에서는 `asyncio` 가 `add_reader` 를 지원하지 않아 `UdpTagFeed` 가 폴링으로 받는다. 컨테이너(Linux) 에서는 `add_reader`.
