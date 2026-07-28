# v14 — 지오펜스 복구 (단계 0)

> 작성일: 2026-07-27
> 파일: `flight/path_flight_phase1_v14.py` (v13 사본 + 감시 로직 추가, **v13 대비 +154행**)
> 비행 로직은 v13과 **완전히 동일**. 감시만 추가했다.
> 상태: 로컬 작성·단위테스트 완료. **기체 업로드·md5 검증 완료** (md5 `b1ee69d6a547769d58106539ae63117c` 로컬·원격 일치, 기체 python3 문법 통과, v13 원본 무손상). **실비행 미검증.**
> (2026-07-27 정정: 최초 작성 시 "업로드 전"으로 적었으나 같은 날 업로드·검증까지 완료했다. worklog 2차 세션 기록 참조.)

## 1. 문제

v13은 `MAX_HORIZONTAL_DEV = 2.5`, `MAX_ALT_DEVIATION = 1.5` 상수와 `--max-horizontal-dev`
CLI 인자를 갖고 있으나 **이를 검사·발동하는 코드가 없었다.** v13의 해당 인자 help 문구가
`"Reserved safety parameter"`인 것에서 미배선 상태가 코드에 그대로 드러나 있었다.
CR 초기판(`CR/path_flight_phase1_1.py:179-187`)에는 있었으므로 **회귀(regression)**다.

## 2. CR 로직을 그대로 이식하면 안 되는 이유

CR의 검사 기준:
```python
dx = self.current_x - self.initial_x        # ← 홈(이륙지점) 기준
dy = self.current_y - self.initial_y
horizontal_dev = math.sqrt(dx*dx + dy*dy)
if horizontal_dev > MAX_HORIZONTAL_DEV:     # CR에서는 FORWARD_DISTANCE + 1.5 = 2.5m
    trigger_emergency_land("GEOFENCE: ...")
```

CR은 "전방 1m 이동"만 하는 미션이라 홈 반경 2.5m가 타당했다.
그런데 **현재 `MISSION_PLAN`은 홈에서 10m 이상 이동한다**:
```python
("right", 1.35, 0.50, 0.0),
("right", 10.0, 0.50, 0.0),     # ← 여기서 이미 홈 반경 2.5m를 훨씬 초과
("left",  9.5, 0.50, 0.0),
```
→ **그대로 이식했다면 이륙 직후 emergency land가 걸린다.**

참고: CR의 **고도** 검사는 이미 `self.target.down_m`(명령값) 기준이었다.
즉 수평만 홈 기준이었고, 그게 문제였다.

## 3. v14 설계 — 두 겹 펜스

### 층위 1: 추종 오차 펜스 (주 방어선, 항상 켜짐)
**실제 위치 vs 명령 setpoint**의 차이를 본다. 미션이 홈에서 얼마나 멀리 가든 무관하게
"위치 폭주(flyaway)"만 잡아낸다 — 이게 원래 잡고 싶었던 위험이다.

```python
h_dev = hypot(mav_x - target.north_m, mav_y - target.east_m)   # > 2.5m
v_dev = abs(mav_z - target.down_m)                             # > 1.5m
```

### 층위 2: 홈 기준 절대 펜스 (기본 꺼짐)
운용 구역 자체를 벗어났는지 본다. 미션이 10m+ 이동하므로 **기본값 0(비활성)**이며,
운용 공간을 알고 나서 켠다.
- `--max-home-dist` : 이륙지점 기준 수평 반경
- `--max-alt-agl` : 이륙지점 기준 상승 한계

## 4. 오발동 방지 장치 3개

안전장치가 오발동해서 비행 중에 착륙해버리면 그 자체가 위험하다. 3중으로 막았다.

| 장치 | 내용 |
|---|---|
| **arm 시점 제한** | `offboard.start()` 성공 이후에만 감시 시작. 프리플라이트·이륙 전에는 꺼둠. 착륙(`land_and_disarm`)·비상착륙 시작 시 즉시 해제 (착륙 중엔 setpoint를 안 따르는 게 정상이므로) |
| **지속시간 요구** | 위반이 `--geofence-violation-sec`(기본 **1.0s**) 이상 **연속** 지속돼야 발동. 단발 VIO 글리치로 착륙시키지 않기 위함. 첫 위반 시엔 경고 로그만 남김 |
| **setpoint 점프 유예** | speed 미지정 이동은 setpoint가 목적지로 즉시 점프해 추종 오차가 정상적으로 커진다. 한 샘플에 `--geofence-jump-thresh`(기본 0.5m) 넘게 튀면 수렴할 때까지 **층위 1만** 유예. 단 `--geofence-jump-timeout`(기본 30s) 안에 수렴 못 하면 그것 자체를 고장으로 보고 발동 |

> 유예 중에도 **층위 2(절대 펜스)는 계속 감시**한다. 점프를 핑계로 완전 무방비가 되지 않게.

> ramp 이동은 20Hz에서 샘플당 2~3cm씩만 움직이므로 점프 임계값에 걸리지 않는다 (테스트 4번에서 검증).

## 5. 변경 지점

| 위치 | 내용 |
|---|---|
| 헤더 docstring | v14 변경점 명시 |
| 상수부 | `DEFAULT_GEOFENCE_*`, `DEFAULT_MAX_HOME_DIST`, `DEFAULT_MAX_ALT_AGL` 추가 |
| `__init__` | `geofence_armed`, `_gf_violation_since`, `_gf_last_target`, `_gf_jump_pending`, `_gf_jump_since` |
| **`_geofence_check()`** (신규) | 판정 본체. 위반 사유 문자열 또는 None 반환 |
| `mavsdk_position_monitor()` | 매 샘플 `_geofence_check()` 호출 → 사유 있으면 `trigger_emergency_land` |
| `arm_and_start_offboard()` | offboard 성공 후 펜스 ARM + 설정값 로그 |
| `trigger_emergency_land()` | 진입 시 펜스 해제 |
| `land_and_disarm()` | 진입 시 펜스 해제 |
| argparse | `--max-alt-deviation`, `--geofence-violation-sec`, `--geofence-jump-thresh`, `--geofence-jump-timeout`, `--max-home-dist`, `--max-alt-agl`, `--no-geofence` 추가. `--max-horizontal-dev` help 정정 |

## 6. 검증 — 단위 테스트 25/25 통과

`analysis/test_geofence_v14.py` (재실행: `python analysis/test_geofence_v14.py`).
mavsdk를 스텁으로 대체하고 가짜 시계를 주입해
`_geofence_check`만 떼어 검증했다.

| # | 항목 | 결과 |
|---|---|---|
| 1 | 기본 (펜스 안 / armed=False / `--no-geofence`) | ✅ 3/3 |
| 2 | 수평 위반 + 지속시간 (0.5s 미발동 → 1.0s 초과 발동 → 복귀 시 타이머 리셋 → 재위반 새 타이머) | ✅ 4/4 |
| 3 | 고도 위반 발동 | ✅ |
| 4 | **ramp 이동을 점프로 오인하지 않음** (0.025m/샘플 × 40) | ✅ 2/2 |
| 5 | setpoint 10m 즉시 점프 → 유예 → 수렴 → 해제 → 재폭주 시 정상 발동 | ✅ 5/5 |
| 6 | 점프 후 30s 수렴 실패 → 타임아웃 발동 | ✅ 2/2 |
| 7 | 홈 절대 펜스 (15m 한계 / 0이면 무제한) | ✅ 3/3 |
| 8 | AGL 절대 펜스 | ✅ |
| 9 | **실제 MISSION_PLAN 회귀 검증** — 1.35m → 10m → -9.5m 주행, 15cm 뒤처짐 추종 | ✅ **오발동 0건** |

9번이 핵심이다. CR 로직을 그대로 이식했다면 여기서 터졌다.

## 7. 남은 일 / 주의

- [x] **기체 업로드 완료** (2026-07-27, md5 일치 검증). v13은 손대지 않았다(5/21 105636B 그대로).
- [ ] **실비행 미검증.** 단위 테스트는 로직만 본 것이고, 실제 VIO 노이즈 수준에서
      `--geofence-violation-sec 1.0`과 `--max-horizontal-dev 2.5`가 적절한지는 실측이 필요하다.
      첫 비행은 **`--max-home-dist`를 운용 공간에 맞게 켜고** 보수적으로 나가는 걸 권장.
- [ ] 실비행 전 권장: 지상에서 `--no-geofence` 없이 arm까지만 해보고 `[geofence] ARMED` 로그와
      오발동 여부 확인.
- [ ] 배터리 저전압 failsafe (파이썬 레벨)는 여전히 없음 — 별건.

### v13에서 이미 되어 있던 것 (조사 중 확인, docs/03 정정)
`trigger_emergency_land`의 **kill 폴백은 v13에 이미 있다** (`land()` 실패 → `except` → `action.kill()`).
docs/03에서 "❓ 확인 필요"로 남겼던 항목이며, **추가 작업 불필요**.
