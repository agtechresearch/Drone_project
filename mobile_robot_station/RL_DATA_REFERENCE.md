# 강화학습 도입을 위한 데이터 참조

오늘(2026-05-16) 실내 경로 왕복 주행 테스트에서 수집된 데이터 종류 및 향후 RL 활용 참고.

---

## 1. 관측값 (Observation) — 센서 입력

### 라이다 거리 (`/scan`, sensor_msgs/LaserScan)

| 값 | 단위 | 측정 방법 | 비고 |
|----|------|----------|------|
| `left` | m | -90° 섹터 ±15° 중앙값 | 좌측 벽까지 거리 |
| `right` | m | +90° 섹터 ±15° 중앙값 | 우측 벽까지 거리 |
| `front` | m | 180° 섹터 ±10° 최솟값 | 전방 장애물까지 거리 |
| `front_span` | deg | 전방 60° 내 stop_dist 이하 포인트의 각도 스팬 | 벽/기둥 구분용 |
| `error` | m | `(left - ref_l) - (right - ref_r)` | 기준 대비 횡방향 오차 |
| `integral` | m·s | `∫error dt` (데드존·클램프 적용) | 헤딩 드리프트 누적량 |

**범위:**
- `left`, `right`: 0.10 ~ 12.0 m (inf = 벽 없음)
- `front`: 0.10 ~ 12.0 m
- `front_span`: 0° ~ 60°
- `error`: 약 -3.0 ~ +3.0 m
- `integral`: -3.0 ~ +3.0 m·s (INTEGRAL_CLAMP)

**소스:** `/scan` 토픽, YDLidar X2, 20Hz (RATE_HZ=20)

---

## 2. 행동값 (Action) — 제어 출력

### cmd_vel (`/cmd_vel`, geometry_msgs/Twist)

| 값 | 단위 | 범위 | 역할 |
|----|------|------|------|
| `linear.x` | m/s | 0.0 ~ 0.12 (전진), -0.30 (후진/회전 없음) | 전진/후진 속도 |
| `linear.y` | m/s | -0.15 ~ +0.15 | 횡이동 (메카넘) |
| `angular.z` | rad/s | -0.25 ~ +0.25 (주행 중), ±0.30 (회전 단계) | 요(yaw) 제어 |

**소스:** Python PID 계산 결과 → `/cmd_vel` 퍼블리시 → `myagv_odometry_node` → MCU

---

## 3. 상태 정보 (State)

| 값 | 단위 | 소스 | 비고 |
|----|------|------|------|
| `ref_l` | m | 시작점 측정 중앙값 | 좌측 기준 거리 |
| `ref_r` | m | 시작점 측정 중앙값 | 우측 기준 거리 |
| `state` | - | 상태머신 | FORWARD / ROTATING / RETURN / DONE |
| `vx` | m/s | calc_speed() 출력 | 현재 전진 속도 |

---

## 4. 보상 설계 아이디어 (향후 RL용)

| 보상 항목 | 조건 | 값 |
|----------|------|-----|
| 경로 유지 | `abs(error)` 작을수록 | +1 / (1 + abs(error)) |
| 전진 | `linear.x > 0` | +0.1 × vx |
| 벽 충돌 임박 | `front < 0.3m` | -1.0 |
| 기준 거리 이탈 | `abs(error) > 0.5m` | -0.5 |
| 목표 도달 | `front ≤ front_stop` AND 벽 판정 | +10.0 |
| 경로 이탈 | 좌/우 거리가 기준의 2배 초과 | -5.0 |

---

## 5. 현재 제어 파라미터 (기준값)

```python
# PID
KP = 0.40   # 위치 오차 → linear.y
KD = 0.10   # 오차 변화율
KI = 0.08   # 누적 오차 → angular.z

# 속도
FORWARD_SPEED  = 0.12   # m/s
ROTATE_SPEED   = 0.30   # rad/s

# 정지 조건
FRONT_STOP_DIST    = 0.50   # m
WALL_SPAN_MIN_DEG  = 40     # deg (적응형: × front_stop/f)

# 기둥 필터
PILLAR_IGNORE_DIST = 0.40   # m
```

---

## 6. 실제 측정된 대표 수치 (테스트 중 관측)

| 상황 | left | right | front | error |
|------|------|-------|-------|-------|
| 통로 정상 주행 | ~0.8m | ~1.2m (비대칭) | >2.0m | ~0 |
| 헤딩 드리프트 발생 | 급변 | 급변 | 감소 | ±0.3~0.5m |
| 기둥 근접 (전방) | — | — | 0.2~0.4m | — |
| 기둥 각도 스팬 | — | — | — | ~5~15° |
| 벽 각도 스팬 | — | — | — | ~40~100° |

---

## 7. 향후 RL 도입 시 고려사항

- **환경 시뮬레이터:** Gazebo + myAGV URDF (또는 Isaac Sim)로 sim-to-real 전략 권장
- **관측 공간:** `[left, right, front, error, integral, vx]` → 6차원
- **행동 공간:** `[linear.x, linear.y, angular.z]` → 연속 3차원 또는 이산화
- **핵심 문제:** 헤딩 드리프트 후 복구 — 현재 PID로는 기둥 방향으로 수렴하는 경우 있음
- **데이터 수집:** rosbag으로 `/scan`, `/cmd_vel`, `/odom` 동시 기록 권장
  ```bash
  rosbag record /scan /cmd_vel /odom -O corridor_run.bag
  ```
