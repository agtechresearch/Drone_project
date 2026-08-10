# myAGV 자율주행 운용 가이드

## 하드웨어 구성

| 항목 | 내용 |
|------|------|
| 보드 | Raspberry Pi 4B (aarch64) |
| OS | Ubuntu 20.04 + ROS Noetic |
| 구동 | 메카넘 휠 4륜 (전/후/좌/우/대각 이동 가능) |
| MCU | `/dev/ttyAMA2` 115200bps (pymycobot.MyAgv 프로토콜) |
| 라이다 | YDLidar X2, `/dev/ttyAMA0` 115200bps, 프레임: `laser_frame` |
| 라이다 모터 | GPIO 20번 HIGH=ON / LOW=OFF (소프트웨어 제어) |

---

## A안 — 직접 제어 (Direct cmd_vel)

### 원리

```
move.sh forward 0.3
    │
    ▼
move_direct.py
    │  /cmd_vel 퍼블리시 (geometry_msgs/Twist)
    ▼
myagv_odometry_node  (myAGVSub.cpp)
    │  cmd_vel → speed = vel * 100 + 128 → 직렬 패킷
    ▼
/dev/ttyAMA2 → MCU → 휠 모터
    │
    │  /odom 피드백 (nav_msgs/Odometry)
    ▼
move_direct.py (이동거리 측정 → 목표 도달 시 정지)
```

- **move_base / AMCL / 지도 불필요**
- `myagv_odometry_node` 하나만 실행되면 동작
- 오도메트리로 이동거리를 측정해 closed-loop 제어
- 오도메트리 없으면 `--timed` 플래그로 시간 기반 폴백

### 속도 변환 공식

MCU 프로토콜: `byte = velocity(m/s) × 100 + 128`
- 0.15 m/s → byte 143
- 0.20 m/s → byte 148
- 최대 실용 속도: 약 0.25 m/s (그 이상은 미끄러짐)

### 시작 방법

```bash
# 터미널 1 — 오도메트리 + 라이다 시작
bash /home/er/autonomous_nav/scripts/start_direct.sh

# 터미널 2 — 이동 명령
bash /home/er/autonomous_nav/scripts/move.sh forward 0.3
```

### 명령어 목록

```bash
move.sh forward  <m>           # 전진
move.sh back     <m>           # 후진
move.sh left     <m>           # 왼쪽 횡이동 (메카넘)
move.sh right    <m>           # 오른쪽 횡이동 (메카넘)
move.sh turn     <deg>         # 회전 (양수=반시계, 음수=시계)
move.sh dx <m> dy <m>          # 대각선 이동

# 속도 조절
move.sh forward 0.3 --speed 0.10    # 느리게 (0.10 m/s)
move.sh forward 0.3 --speed 0.20    # 빠르게 (0.20 m/s)
move.sh turn 90 --aspeed 0.3        # 회전 속도 조절 (rad/s)

# 모드 플래그
move.sh forward 0.3 --timed         # 시간 기반 (오도메트리 없어도 됨)

# 기타
bash stop_all.sh                     # 비상 정지
bash save_waypoint.sh A              # 현재 위치 저장 (오도메트리 기준)
bash status.sh                       # 상태 확인
```

### 속도 기본값 변경

`/home/er/autonomous_nav/python/move_direct.py` 상단:

```python
LINEAR_SPEED  = 0.15   # m/s — 직선 이동 기본 속도
ANGULAR_SPEED = 0.4    # rad/s — 회전 기본 속도
```

권장 범위:
- `LINEAR_SPEED`: 0.08 ~ 0.25 m/s (0.15가 정확도/속도 균형 최적)
- `ANGULAR_SPEED`: 0.2 ~ 0.6 rad/s

### 한계

- 장애물 회피 없음 (앞에 뭔가 있으면 그냥 박음)
- 누적 오차: 오도메트리는 미끄러짐/바닥 상태에 따라 오차 누적
- 웨이포인트 자동 이동(`go.sh`), 순찰(`patrol.sh`) 불가

---

## B안 — Navigation Stack (move_base + AMCL)

### 원리

```
move.sh forward 0.3 --nav
    │
    ▼
move_relative.py
    │  /amcl_pose 로 현재 위치 파악
    │  목표 좌표 계산 (로봇 프레임 → 맵 프레임 변환)
    │  /move_base/goal 퍼블리시
    ▼
move_base
    ├── Global Planner (global_planner/GlobalPlanner)
    │     맵 위에서 출발→목표 최단경로 계산
    ├── Local Planner (dwa_local_planner/DWAPlannerROS)
    │     실시간 cmd_vel 생성, 장애물 회피
    └── Costmap (global + local)
          라이다 스캔으로 장애물 격자 관리
    │
    ▼
/cmd_vel → myagv_odometry_node → MCU
```

```
AMCL (Adaptive Monte Carlo Localization)
    /scan + /odom + 저장된 맵
    → 파티클 필터로 맵 위 로봇 위치 추정
    → map→odom TF 퍼블리시
```

TF 체인: `map → odom → base_footprint → laser_frame`

### 시작 방법

```bash
# 맵이 없으면 먼저 맵 제작 (아래 맵 제작 섹션 참조)

# 터미널 1 — 전체 navigation stack 시작
bash /home/er/autonomous_nav/scripts/3_start_navigation.sh

# 터미널 2 — 초기위치 설정 (필수!) — 로봇을 원점(매핑 시작점)에 같은 방향으로 둔 뒤
bash /home/er/autonomous_nav/scripts/set_initialpose.sh   # 원점(0,0,0) 발행 (RViz 없이)
#   ↳ 로봇이 다른 위치면: set_initialpose.sh <x> <y> <yaw°>
#   ↳ 이거 안 하면 AMCL이 (0,0,0) 방치 → 스캔이 맵과 안 맞아 주행 status 4 ABORTED

# 터미널 2 — 이동 명령 (AMCL 수렴까지 30초 대기)
bash /home/er/autonomous_nav/scripts/move.sh forward 0.3 --nav
bash /home/er/autonomous_nav/scripts/go.sh A              # 웨이포인트 이동
bash /home/er/autonomous_nav/scripts/patrol.sh            # 순찰
```

> **초기위치(initialpose)가 핵심.** RViz(2D Pose Estimate)는 Pi GPU/RAM 부담으로 못 쓰므로
> `set_initialpose.sh`로 CLI 발행한다. 발행 후 `/amcl_pose`가 뜨고 요청 좌표와 일치하는지
> 스크립트가 확인해준다. waypoint 저장(`save_waypoint.sh`)도 `/amcl_pose`가 있어야 map 프레임으로 기록됨.

### 명령어 목록

```bash
set_initialpose.sh              # 초기위치 원점(0,0,0) 발행 (RViz 대체, 주행 전 필수)
set_initialpose.sh 1.5 2.0 90   # 초기위치 x=1.5 y=2.0 yaw=90도로 발행
move.sh forward 0.3 --nav      # 전진 0.3m (장애물 회피 포함)
go.sh A                         # 웨이포인트 A로 이동
go.sh 1.5 2.0 90               # 좌표 (1.5, 2.0) yaw=90도로 이동
patrol.sh                       # waypoints.yaml 순서대로 순찰
patrol.sh --once                # 1회만
save_waypoint.sh A              # 현재 AMCL 위치를 A로 저장 (initialpose 후에!)
save_waypoint.sh --list         # 웨이포인트 목록
stop_all.sh                     # 비상 정지
```

### 주요 파라미터 파일

| 파일 | 역할 |
|------|------|
| `autonomous_nav/launch/navigation.launch` | 전체 stack 진입점 |
| `autonomous_nav/config/navigation_overrides.yaml` | 코스트맵 핵심 수정값 |
| `autonomous_nav/config/mecanum_dwa_params.yaml` | DWA 메카넘 설정 |
| `myagv_navigation/param/dwa_local_planner_param/amcl.yaml` | AMCL 기본값 |

### 핵심 수정 내역 (버그 픽스)

| 문제 | 원인 | 수정 |
|------|------|------|
| AMCL 위치 오차 큼 | `odom_model_type: diff` (차동 모델) | `omni` 로 변경 |
| DWA `Cost: -1` 오류 | `local_costmap/global_frame: map` + TF 지연 | `odom` 으로 변경 |
| Rotate recovery 실패 | `track_unknown_space: true` → 미지셀 = 충돌 | `false` 로 변경 |
| TF timeout | `transform_tolerance: 1.0` (너무 타이트) | `2.0` 으로 증가 |
| 코스트맵 갱신 느림 | `update_frequency: 0.3` Hz | 5 Hz / 1 Hz 로 증가 |

### 한계

- 맵이 있어야 함 (AMCL 의존)
- 시작 위치가 맵 원점과 일치해야 초기 위치 정확
- MCU 직렬 포트 에러 시 TF 체인 불안정
- AMCL 수렴까지 30초 내외 소요

---

## 맵 제작 (gmapping SLAM)

```bash
# 터미널 1 — SLAM 시작
bash /home/er/autonomous_nav/scripts/1_make_map.sh

# 터미널 2 — 방 전체를 천천히 이동 (출발점으로 돌아와서 끝내면 정확도 향상)
bash /home/er/autonomous_nav/scripts/move.sh forward 0.5
bash /home/er/autonomous_nav/scripts/move.sh turn 90
# ... 반복

# 탐색 완료 후 저장
bash /home/er/autonomous_nav/scripts/2_save_map.sh

# 터미널 1에서 Ctrl+C 후 navigation 시작
bash /home/er/autonomous_nav/scripts/3_start_navigation.sh
```

> 터미널 2 조종은 `map_teleop.sh`(단일키 스텝 드라이버) 권장:
> `i`/`k` 전·후진, `j`/`l` 좌·우회전, `u`/`o` 좌·우 횡이동, `r` 360° 제자리 스윕, `+`/`-` 스텝 조절, `space` 정지, `q` 종료.

### 매핑 요령 (넓은/긴 공간에서 시간 단축)

**핵심 개념: 매핑은 "바닥을 훑는" 게 아니라 "벽을 찍는" 것.**
free 공간은 라이다 레이가 자동으로 채운다(raycasting, `maxRange` 5m까지). 즉 free 바닥을 물리적으로 밟고 지나갈 필요가 없다. 필요한 건 **모든 벽/구조물이 언젠가 라이다 유효거리 안에 들어오는 것**뿐이다. 걸레질하듯 전체 면적을 훑을 필요 없음.

- **유효거리** = gmapping `maxUrange` **4.5m** (이 안의 히트만 벽으로 등록. 실무 마진 ~3.5~4m로 생각).
- **후방 ~100°(`ignore_array=-50,50`)는 실명** → 정지 스캔은 뒤를 못 봄. 코너·새 공간마다 `r` 스윕으로 보완.

| 항목 | 값 | 이유 |
|---|---|---|
| 평행 경로(왕복 줄) 간격 | **~4m 기본** | 라이다 1회 사거리. 인접 줄끼리 특징이 겹쳐 정합 안정 |
| ↳ 넓혀도 되는 경우 | 최대 ~7~8m | 두 줄 사이에 연속된 벽이 나란히 이어질 때(벽이 양쪽 줄을 이어줌) |
| 경로 방향 스텝 | **0.3~0.5m** | 연속 스캔 ~90% 겹침 → 정합 우수. **점프 금지** |
| 벽으로부터 최대 이격 | **~4m** | 이보다 멀면 벽이 유효거리 밖 → 위치추정 흔들림 |
| 스윕(`r`) | 코너·끝·새 공간·후방 채울 때 | 후방 실명 보완 + 주변 벽 확보 |

**직관:** 한 줄을 지나가면 좌우 각각 ~4m(폭 ~8m 띠)의 벽이 잡힌다.
- 통로 폭 ≤ 8m → 가운데로 한 번만 지나가도 양쪽 벽 다 찍힘, 왕복 불필요.
- 통로 폭 > 8m(예: 10m 넓은 공간) → 가운데는 맞출 벽이 없어 드리프트. **벽을 앵커로 삼아 벽에서 3~4m 이내로 외곽을 도는 벽추종 패턴**, 내부는 ~4m 간격 줄로만. 특징 없는 넓은 곳엔 임시 박스/콘을 놓아 앵커를 만들기도 함.

**실전 순서 (긴/넓은 공간):**
1. 걸레질 대신 → 외곽 벽 한 바퀴(벽에서 3~4m 유지)
2. 내부는 필요한 만큼만, 평행 줄 간격 ~4m (폭 8m 이하면 가운데 한 줄)
3. 스텝 0.3~0.5m, 점프 금지, 코너·새 공간마다 `r` 스윕
4. 어디서든 벽 4m 이내 유지, 중간중간 시작점/특징지점 복귀(루프 클로징으로 드리프트 보정)
5. 시작점 복귀 후 저장

⚠️ "빠르게"의 핵심은 **경로를 줄이는 것**이지 **스텝을 키우는 게 아님.** 크게 점프하면 드리프트로 없는 벽이 생겨 로봇이 갇힌다(예냉실 실패 원인).

맵 파일 위치: `/home/er/autonomous_nav/maps/`
현재 맵 심볼릭 링크: `current_map.yaml` → 가장 최근 저장 맵

---

## C안 — 라이다 통로 추종 (corridor_follow)

### 원리

```
/scan (LaserScan)
    │
    ├── 좌측 -90° 섹터 중앙값  → left_dist
    ├── 우측 +90° 섹터 중앙값  → right_dist
    └── 전방   0° 섹터 최솟값  → front_dist

error     = left_dist - right_dist   (양수 = 오른쪽 치우침)
integral += error × dt               (누적 = 헤딩 틀어짐 감지)

cmd_vel.linear.y  = Kp×error + Kd×Δerror   ← 위치 보정 (횡이동)
cmd_vel.angular.z = Ki×integral             ← 헤딩 보정 (회전)
cmd_vel.linear.x  = 전진속도 (전방 장애물 시 감속/정지)
```

**핵심: 두 가지 오차를 분리 처리**
- `linear.y` (P+D): 지금 중앙에서 얼마나 벗어났는지 → 즉시 횡이동으로 보정
- `angular.z` (I): 오차가 지속 누적 → 헤딩 자체를 수정 (턱/장애물 후 드리프트 복구)

일시적 충격은 오차가 금방 0으로 돌아와 적분이 안 쌓임 → 헤딩 보정 미발동.

### 라이다 각도 규약

```
         전방 0°
           ↑
  좌 -90° ← → 우 +90°
```
(YDLidar X2 마운팅 기준, inverted=true + roll=π TF 적용 후)

### 시작 방법

```bash
# start_direct.sh 실행 중인 상태에서

# 1. 센서 방향 확인 (최초 1회)
bash /home/er/autonomous_nav/scripts/corridor.sh --scan
# 왼쪽에 손 대면 좌값 감소, 오른쪽에 손 대면 우값 감소 확인

# 2. 제자리 중앙 정렬 테스트
bash /home/er/autonomous_nav/scripts/corridor.sh --center

# 3. 통로 주행
bash /home/er/autonomous_nav/scripts/corridor.sh
```

### 명령어 목록

```bash
corridor.sh                        # 전진 + 통로 중앙 유지 + 전방 정지
corridor.sh --scan                 # 좌/우/전방 거리 표시 (센서 확인용)
corridor.sh --center               # 제자리 중앙 정렬만
corridor.sh --speed 0.10           # 전진 속도 지정 (기본 0.12 m/s)
corridor.sh --stop-dist 0.8        # 전방 정지 거리 지정 (기본 0.5 m)
corridor.sh --speed 0.10 --stop-dist 0.8   # 조합 가능
```

### 전방 장애물 동작

| 전방 거리 | 동작 |
|----------|------|
| `stop_dist × 2` 이상 | 정속 전진 |
| `stop_dist` ~ `stop_dist × 2` | 선형 감속 (최소 30%) |
| `stop_dist` 이하 | **완전 정지** |

기본 `stop_dist = 0.5m` → 0.5m에서 정지, 1.0m부터 감속 시작.  
통로 끝 벽 앞에서 세우려면: `corridor.sh --stop-dist 0.6`

### 튜닝 파라미터

`/home/er/autonomous_nav/python/corridor_follow.py` 상단:

| 파라미터 | 기본값 | 역할 |
|---------|-------|------|
| `KP` | 0.4 | 위치 오차 → linear.y 비례 게인 |
| `KD` | 0.1 | 오차 변화율 → 진동 억제 |
| `KI` | 0.08 | 누적 오차 → angular.z (헤딩 보정) |
| `INTEGRAL_CLAMP` | 3.0 | 적분 누적 최대값 (와인드업 방지) |
| `INTEGRAL_DEADZONE` | 0.03 m | 이 이하 오차는 적분 제외 |
| `FORWARD_SPEED` | 0.12 m/s | 기본 전진 속도 |
| `FRONT_STOP_DIST` | 0.5 m | 기본 전방 정지 거리 |
| `SECTOR_WIDTH_DEG` | 30° | 좌/우 측정 섹터 폭 |

**튜닝 증상별 처방:**

| 증상 | 처방 |
|------|------|
| 좌우 진동 | `KP` ↓ 또는 `KD` ↑ |
| 헤딩 보정 너무 느림 | `KI` ↑ |
| 헤딩 보정 과해서 S자 주행 | `KI` ↓ 또는 `INTEGRAL_CLAMP` ↓ |
| 노이즈에도 헤딩 보정 발동 | `INTEGRAL_DEADZONE` ↑ (0.05~0.10) |
| 좁은 통로에서 벽 인식 불안정 | `SECTOR_WIDTH_DEG` ↓ (20°) |

---

## D안 — 실내 경로 왕복 주행 (indoor_run)

### 개요

라이다 기반으로 실내 특정 경로를 왕복하는 알고리즘.  
C안(corridor_follow)의 대칭 통로 가정을 제거하고, **시작점 실측 기준 거리**를 유지하는 방식으로 변경.

### 파일 구성

| 파일 | 역할 |
|------|------|
| `python/indoor_run.py` | 단계별 키보드 확인 버전 (수동 단계 진행) |
| `python/indoor_run_auto.py` | 완전 자동 버전 (키 입력 없음) |
| `scripts/indoor_run.sh` | 키보드 버전 실행 스크립트 |
| `scripts/indoor_run_auto.sh` | 자동 버전 실행 스크립트 |

### 동작 순서

```
[기준 측정] 시작점에서 REF_MEASURE_SEC 동안 좌/우 거리 중앙값 저장
      ↓
[1단계 전진] error = (left-ref_l) - (right-ref_r) 기반 PID 제어로 전진
            전방 장애물 벽 판정(각도 스팬) 시 정지
      ↓
[대기] PHASE_WAIT_SEC
      ↓
[2단계 회전] 제자리 회전 → 좌/우 라이다 값 교환 감지(≈180°) 또는 타임아웃
      ↓
[대기] PHASE_WAIT_SEC
      ↓
[3단계 귀환] 교환된 기준값으로 동일 방식 전진 → 전방 정지
```

### 제어 원리

```
기준 거리: ref_l, ref_r = 시작점 실측값 (비대칭 환경 대응)

error = (left - ref_l) - (right - ref_r)
  → 올바른 위치면 0, 치우치면 ±

linear.y  = Kp×error + Kd×Δerror   (횡이동 보정)
angular.z = Ki×∫error dt            (헤딩 보정)
```

### 기둥 / 일시 장애물 처리

```
좌우 측면: filter_pillar()
  현재 거리 < 기준 - PILLAR_IGNORE_DIST → 기둥으로 판단 → 기준값 사용

전방: front_obstacle_span() + 거리 적응형 임계값
  stop_dist 이내 포인트들의 각도 스팬 계산
  adaptive_min = WALL_SPAN_MIN_DEG × (front_stop / 현재거리)
  span ≥ adaptive_min → 벽 → 정지
  span <  adaptive_min → 기둥 → 무시하고 계속 전진
```

### 시작 방법

```bash
# start_direct.sh 실행 중인 상태에서

# 자동 버전 (권장)
bash /home/er/autonomous_nav/scripts/indoor_run_auto.sh

# 단계별 키보드 확인 버전
bash /home/er/autonomous_nav/scripts/indoor_run.sh
```

### 명령어 목록

```bash
# 자동 버전
indoor_run_auto.sh                          # 기본 파라미터
indoor_run_auto.sh --front-stop 0.9         # 전방 정지 거리 지정
indoor_run_auto.sh --fspeed 0.10            # 전진 속도 지정
indoor_run_auto.sh --scan                   # 센서값 확인

# 키보드 버전 (R=회전, G=귀환, Q=중단)
indoor_run.sh                               # 기본 파라미터
indoor_run.sh --front-stop 0.6
indoor_run.sh --fspeed 0.10
```

### 주요 튜닝 파라미터 (`indoor_run_auto.py` 상단)

| 파라미터 | 기본값 | 역할 |
|---------|-------|------|
| `FRONT_STOP_DIST` | 0.50 m | 전방 정지 거리 |
| `FORWARD_SPEED` | 0.12 m/s | 전진 속도 |
| `ROTATE_SPEED` | 0.30 rad/s | 회전 속도 |
| `PHASE_WAIT_SEC` | 2.0 s | 단계 전환 대기 시간 |
| `REF_MEASURE_SEC` | 2.0 s | 기준 거리 측정 시간 |
| `PILLAR_IGNORE_DIST` | 0.40 m | 측면 기둥 무시 임계값 |
| `WALL_SPAN_MIN_DEG` | 40° | 전방 벽 판정 최소 각도 스팬 |
| `SWAP_TOLERANCE` | 0.35 m | 180° 회전 완료 감지 허용 오차 |
| `ROTATE_TIMEOUT_SEC` | 12.0 s | 회전 타임아웃 |

### 한계 및 향후 과제

현재 라이다 기반 제어의 한계:

| 문제 | 원인 | 상태 |
|------|------|------|
| 헤딩 드리프트 후 기둥 방향 전진 | 턱/장애물 통과 후 heading 변화, PID 복구 불충분 | 미해결 |
| 기둥 가까이서 전방 오판 | 기둥이 가까울수록 각도 스팬 증가 → 벽으로 오인 | 적응형 임계값으로 부분 완화 |
| 옆 벽/다른 경로로 이탈 | 헤딩 드리프트 누적 시 경로 이탈 | 미해결 |

**향후 개선 방향:** 강화학습(RL) 기반 경로 추종 도입 검토

---

## 비상 정지

어떤 모드에서든:
```bash
bash /home/er/autonomous_nav/scripts/stop_all.sh
```

동작:
1. `/cmd_vel` 제로 명령 퍼블리시
2. 모든 ROS 노드 kill
3. GPIO 20 LOW → 라이다 모터 정지

---

## 프로세스 구조

```
A안 실행 시:
  lidar_power_daemon.py  (GPIO 20 HIGH)
  myagv_odometry_node    (odom 퍼블리시 + cmd_vel → 직렬)
  ydlidar_ros_driver     (scan 퍼블리시)

B안 추가 실행:
  map_server             (맵 로드)
  amcl                   (위치 추정)
  move_base              (경로 계획 + 장애물 회피)
  rviz                   (시각화, DISPLAY=:0)
```
