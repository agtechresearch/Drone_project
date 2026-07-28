# CR 폴더 코드 vs v13 비교

> 질문: `~/CR/path_flight_phase1_1.py`가 최종인가? v13과 비교.
> 결론: **CR 파일은 최종이 아니다. v13이 최종·최신이다.** 단, CR에만 있는 안전장치 1개(지오펜스)가 v13에서 사라졌다.

## 한눈에

| 항목 | `CR/path_flight_phase1_1.py` | `path_flight_phase1_v13.py` |
|---|---|---|
| 정체 | "Phase 1 **PATCHED v2**" (FIX 1~12 적용 초기 안정판) | 최신 커스텀 미션 실행기 |
| 크기 | 19KB / **510줄** / 함수 20개 | 105KB / **2277줄** / 함수 62개 |
| 수정일 | 2026-05-16 | 2026-05-21 (더 최신) |
| 미션 정의 | `execute_mission`에 하드코딩 (forward 1m 수준) | 상단 **`MISSION_PLAN` 리스트** 편집식 (지그재그 커버리지) |
| 도달 판정 | MAVSDK telemetry NED | **voxl pipe 피드백**(`reach_pipe`, 27회 참조) — 더 정밀 |
| 속도 제어 | 없음 | **ramp 방식** setpoint 분할 |
| 좌표계 | NED 고정 | **`MOVE_FRAME` body/world** 선택 |
| 회전(yaw/turn) | 최소 | 본격 지원 (turn 125, yaw 197 참조) |
| goto/home 절대이동 | 미미 | 지원 |
| preland 정밀착륙 | 없음 | 있음 (47 참조) |
| CSV 로깅 | 없음 | 있음 (29 참조) |

## 안전장치 비교 (핵심)

| 안전장치 | CR | v13 |
|---|---|---|
| VIO/health 손실 → emergency land | ✅ (`vio_quality_monitor`) | ✅ (`health_monitor`) |
| 셋포인트 스트림 실패 → emergency | ✅ | ✅ |
| emergency land 시 land 실패하면 **kill 폴백** | ✅ (`action.kill()`) | ✅ **있음** (2026-07-27 확인. `land()` 실패 → `except` → `action.kill()`) |
| **지오펜스(수평/고도 편차 초과 → emergency land)** | ✅ **실제 강제** (`position_monitor` L178-187) | ❌ **상수만 정의, 미강제** |
| 장애물 회피 (obstacle/tof/voa) | ❌ 0건 | ❌ 0건 |

### 결정적 발견: 지오펜스 회귀(regression)
- **CR**: `MAX_HORIZONTAL_DEV = FORWARD_DISTANCE + 1.5`, `MAX_ALT_DEVIATION`를
  `position_monitor`에서 매 샘플 검사 → 초과 시 `trigger_emergency_land("GEOFENCE...")` 실제 발동.
- **v13**: `MAX_HORIZONTAL_DEV = 2.5`, `MAX_ALT_DEVIATION = 1.5` 상수와 `--max-horizontal-dev`
  CLI 인자는 남아있으나, **감시 루프에서 이를 검사·발동하는 코드가 없음.**
- 즉 v13로 오면서 기능은 대폭 커졌지만 **"위치 폭주 방지" 안전망이 빠졌다.**

## 권장 조치
1. v13를 메인으로 유지 (기능·정밀도 우위 명확).
2. **CR의 지오펜스 로직을 v13의 `mavsdk_position_monitor` 또는 `health_monitor`에 이식** → 즉시 안전성 회복 (저위험, 고효과).
3. 이후 ToF/VOA 기반 장애물 회피를 신규 추가 (양쪽 다 없음 → docs/02 로드맵).
