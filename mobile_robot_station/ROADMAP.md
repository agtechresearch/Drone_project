# ROADMAP — mobile_robot_station

이동형 드론 스테이션 + 딸기 적재 모바일 로봇 자율주행의 **진행 현황 요약**.
날짜별 상세 이력은 [`worklog/`](worklog/), 구조는 [`PROJECT_STRUCTURE.md`](PROJECT_STRUCTURE.md) 참고.

---

## ✅ 완료된 작업

### 인식 / 조작 기반
- 카메라 라인추종 파이프라인 + 색 캘리브레이션 GUI + 샘플 라이브러리 (07-07)
- 단일키 텔레오프 드라이버 `map_teleop.py`, 정밀 이동 수단 정리 (07-09~10)

### 맵 기반 자율주행 파이프라인
- gmapping 매핑 → AMCL → move_base → **터치스크린 웨이포인트 UI** 전체 파이프라인 구축·검증 (07-07)
- UI에 **`⌂ 원점 초기화` 버튼** + 초기위치 팝업 추가(`/initialpose` 발행) (07-10)

### 주행 실패 원인들 규명·해결
1. **배터리 방전**(9.6V) → 소프트웨어 무관 확인 (07-07)
2. **초기위치 미설정** → AMCL이 원점 착각 → move_base 경로계획 실패(ABORTED). 매 세션 initialpose 설정 필요, UI 버튼으로 해소 (07-09~10)
3. **맵 품질(로봇이 좁은 방에 갇힘)** → 재매핑. `map_20260710_183322`(97% 도달) → 실증현장 `map_20260713_111502`(자유공간 37.9㎡, 갇힘 없음) (07-10, 07-13)
4. **메카넘 저속 다축 mix 데드밴드**(x·y·회전 동시 발행 시 각 성분이 모터 데드밴드 아래) → 벤더 **`TrajectoryPlannerROS`(diff-drive식, holonomic_robot:false)로 전환**. 첫 목표 주행 성공으로 해결 확인 (07-10 저녁~07-13)
5. **회전 버벅임/헤맴**(`max_vel_theta 0.1 < min_in_place_vel_theta 0.25` 모순 + `latch_xy_goal_tolerance false`) → `navigation_overrides.yaml`에 `max_vel_theta 0.5`/`min_vel_theta -0.5`/`latch true` 적용 (07-13)
6. **AMCL 위치추정 1단계 튜닝**(`odom_alpha2 0.4`, `odom_alpha4 0.25`, `update_min_d 0.10`, `update_min_a 0.15`) → 정합 중앙값 **34→15cm**, ≤15cm **21→50%**, move_base 계획실패(`Failed to get a plan`/`NO PATH`/복구행동) **다수→0건** (07-13)

**현재 상태**: 데드밴드·맵·회전 파라미터 해결. 첫 목표 주행 성공. 최신 맵 `map_20260713_111502`, 웨이포인트 2개(Room1, 예냉실).

---

## 🔴 진행해야 할 작업

### P1 — 위치추정 드리프트 **확정 진단** (최우선)
이동 후에도 정합이 느슨(15cm/50%)하고 AMCL이 과확신. 마지막 run은 스택 조기 종료로 **미측정 → 아직 확정 아님**.
- 스택을 **켠 채로**(loc_check 전 `stop_all` 금지) 목표 1개(예냉실)만 전송
- `scratchpad/loc_check.py`로 라이브 정합(≤15cm 비율) 측정
- `rostopic echo -n1 /move_base/GlobalPlanner/plan` 끝점이 실제 목표와 맞는지
- `/amcl_pose` vs 목표 좌표 거리

### P2 — 드리프트 확정 시 대응 (순서대로)
- (a) `escape_vel -0.0 → -0.1` (복구 시 전진→장애물 충돌 방지). **아직 미수정** — `myagv_ros/.../base_local_planner_params.yaml:22` 또는 overrides에
- (b) `odom_alpha1` 상향 / 파티클 수↑
- (c) 목표 간 재초기화 운용
- (d) **AprilTag 재국소화** (카메라 재활용, 근본 해결 — 산업 AGV 표준. 후방 라이다 사각+메카넘 슬립은 독립 기준으로만 완치)

### P3 — 기타
- 회전 버벅임: **도달되는 조건에서** 재판정
- 배터리: auto_report OFF 시 `/Voltage`·오도 안 나옴 → pymycobot로 켜기. **충전 실효 의심**(11.1V 정체)
- `waypoint_ui.py` `BATT_FULL_V(12.6)`/`BATT_LOW_V(10.5)` 완충 실측 보정
- 라인추종 실테이프 검증(구역별 캘리브 → 마스크 확인 → 실주행)
- (이월) slam_toolbox libceres 의존성

---

## 🟡 향후 확장 (미착수)
- **AprilTag/바닥마커 기반 waypoint 재국소화 노드** — 도착 지점에서 정확한 위치 동기화(자기참조 함정 회피)
- 360° 또는 후방 라이다 보강(사각 해소)
