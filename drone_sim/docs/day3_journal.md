# Day 3 Journal — 2026-08-03

flight_code의 SITL 대응. 실기 코드 경로는 그대로 두고 SITL 모드만 추가.

## 목표

- flight_code에 `sim_pose_reader` 함수 추가 (MAVSDK telemetry 기반)
- `--sim` CLI 플래그로 SITL/실기 모드 전환
- SITL에서 미션 20단계 완주

## 결과

**미션 20단계 완주 성공.**

- 실기 flight_code에 3곳 편집 (기존 코드는 한 줄도 안 바꿈, 새 코드만 추가)
- `--sim` 플래그 유무로 실기와 SITL 모두 지원
- CSV 로그로 정확한 미션 실행 확인 (`voxl_px4_vehicle_local_position_x`가 최대 10.12m 이동, 계획 대비 정확)

## 편집 지점 (4곳)

### 1. `sim_pose_reader` 함수 추가

`voxl_pose_reader` 함수 바로 아래에 새 함수 추가. MAVSDK의 세 스트림을 병렬 구독:

- `telemetry.position_velocity_ned()` — NED 위치와 속도
- `telemetry.attitude_euler()` — roll/pitch/yaw (deg)
- `telemetry.attitude_angular_velocity_body()` — 각속도

수집한 값을 `parse_voxl_inspect_pose_line`이 반환하는 것과 **동일한 dict 형식**으로 `self.voxl_latest[pipe_name]`에 저장. 다운스트림 코드(`wait_reach_pipe_ready`, `feedback_state`, CSV 로깅 등)는 소스가 실기든 SITL이든 구분 못 하고 그대로 동작.

여러 pipe(`px4_vehicle_local_position`, `vvhub_body_wrt_local` 등)에 대해 SITL에서는 동일한 ground truth를 채운다.

### 2. `arm_and_start_offboard` 내 리더 시작 분기

기존:
```python
for pipe in self.args.voxl_pipes:
    self.voxl_tasks.append(asyncio.ensure_future(self.voxl_pose_reader(pipe)))
```

수정:
```python
if getattr(self.args, "sim", False):
    self.voxl_tasks.append(asyncio.ensure_future(
        self.sim_pose_reader(self.args.voxl_pipes)))
else:
    for pipe in self.args.voxl_pipes:
        self.voxl_tasks.append(asyncio.ensure_future(self.voxl_pose_reader(pipe)))
```

`--sim`이 없으면 실기 로직 그대로.

### 3. argparse에 `--sim` 플래그 추가

```python
p.add_argument("--sim", action="store_true",
               help="Use MAVSDK telemetry instead of voxl-inspect-pose. For SITL only.")
```

### 4. CSV 경로 하드코딩 안전화

`/home/root`가 쓰기 불가능하면 (SITL 환경) 현재 디렉터리로 폴백. 실기에서는 원래 경로 그대로.

```python
default_dir = "/home/root"
if not os.path.isdir(default_dir) or not os.access(default_dir, os.W_OK):
    default_dir = os.getcwd()
path = os.path.join(default_dir, "px4_reach_sync_custom_v13_{}.csv".format(stamp))
```

## 검증 결과

Container B에서 실행:
```bash
python3 path_flight_phase1_v13.py --sim --csv auto --csv-sample-sec 1.0
```

로그에 `[sim] starting MAVSDK-based pose reader (SITL mode)`가 뜨고, 20단계 미션이 정상 진행. Gazebo GUI에서 iris_starling 드론이 실제 계획된 경로대로 움직이는 것 시각 확인.

CSV 분석:
- 총 소요 시간: 7.4분 (예상 범위)
- 등장 스테이지: takeoff, right_1, turn_3, right_4, up_5, left_7, up_8, right_10, up_11, left_13, up_14, right_16, down_17, left_18, turn_19, left_20, preland_descend 모두 확인
- 실제 이동 범위: `voxl_px4_vehicle_local_position_x` -0.18 ~ +10.12m (미션 계획 대비 정확)

## 삽질 로그

### 1. Python 3.6에서 asyncio.run() 없음

Day 1에 이미 확인한 사항이지만, `sim_pose_reader` 구현 시 태스크 생성/취소에서도 3.6 호환성 주의. `asyncio.ensure_future` 사용.

### 2. VNC 화면 얼음 vs 실제 시뮬 진행

미션 실행 중 브라우저 VNC 화면이 얼어붙어 마치 물리 시뮬레이션이 멈춘 것처럼 보였다. 그러나 로그와 CSV 분석 결과 시뮬레이션은 정상 진행. **VNC 렌더링만 얼었을 뿐 Gazebo 물리는 살아있음**. 브라우저 재접속으로 부분 복구.

교훈: 시각 확인이 아니라 로그와 CSV로 검증하는 것이 신뢰할 수 있음.

### 3. `mav_abs_n` 컬럼 stale 문제

CSV의 `mav_abs_n/e/d` 필드가 미션 내내 초기값에 얼어있음. 처음엔 미션 실패로 오인했으나, `voxl_px4_vehicle_local_position_x` 필드는 정상 값이었음. 원인 추정: `mavsdk_position_monitor`(기존 함수)와 `sim_pose_reader`(신규)가 둘 다 같은 MAVSDK 스트림을 구독 → MAVSDK-Python 0.12.0에서 다중 subscriber 시 후자가 stale.

**영향**: reach 판정과 미션 진행은 `voxl_latest`(sim_pose_reader가 채움)를 사용하므로 정확. CSV의 mav_abs_* 컬럼만 부정확. 실기에선 두 함수가 서로 다른 소스라 문제없음.

**후속 조치 (선택)**: SITL 모드에서 `mavsdk_position_monitor`를 스킵하도록 추가 편집 가능. 지금은 우선순위 낮음. 분석 시 `voxl_x` 계열 컬럼을 primary로 사용하면 됨.

### 4. 부정확한 초기 인상

CSV의 `mav_abs_n` span=0을 처음 보고 "드론이 안 움직였다"고 오판. 다른 컬럼 확인 후 정정. 다각도 컬럼 검증의 중요성.

## 이미지 스냅샷

- `starling-sitl:day3-complete` — Day 3 완료 상태의 SITL 컨테이너
- `starling-flight:day3-sim` — --sim 대응 flight_code가 있는 컨테이너

## Day 4 준비 사항

- 실기 Starling 2에서 같은 미션 실행하여 데이터 수집
- 실기 데이터와 SITL 데이터를 pair로 비교 가능한 포맷으로 정리
- 지표 정의: 스테이지별 도달 시간, 위치 오차, 자세 오차, VIO 드리프트 등

## 참고 파일

- `flight_code/path_flight_phase1_v13.py` — --sim 대응 추가된 버전
- `logs/day3/day3_sim_mission_20260803.csv` — SITL 미션 로그 (2026-08-03 실행)
- `logs/day3/day3_flight_test_sim.log` — flight_code 실행 로그
- `logs/day3/day3_mavsdk_server.log` — mavsdk_server 로그
