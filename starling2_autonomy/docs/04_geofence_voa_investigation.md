# 지오펜스 · VOA 조사 결과 (공식 문서 + 기체 실측)

> 조사일: 2026-07-27
> 방법: PX4 v1.14 공식 문서 + PX4-Autopilot v1.14.0 소스 파라미터 정의 + ModalAI 공식 문서/포럼 + **기체 실측(`px4-param show`, `voxl-vision-hub.conf`)**

## 0. 기체 버전 (실측)

| 항목 | 값 |
|---|---|
| PX4 | **1.14.0** (git `3e7db37f`), Vendor 2.0.133, 빌드 2026-01-21 |
| HW arch | MODALAI_VOXL2 |
| system-image | 1.8.06-M0054-14.1a-perf |
| voxl-suite | **1.6.3** (sdk-1.6 repo, 2026-05-06 갱신) |
| SKU | MRB-D0014-4-V1-C26-T8-M36-X0 |

---

## 1. 질문: 지오펜스 설정은 수정 가능한가?

**가능하다. 단 서로 독립된 2개 층위가 있고, 각각 문제가 다르다.**

### 층위 A — PX4 파라미터 지오펜스 (`GF_*`)

기체 실측값 / PX4 v1.14.0 `geofence_params.c` 정의:

| 파라미터 | 기체 현재값 | 의미 | 범위·enum |
|---|---|---|---|
| `GF_MAX_HOR_DIST` | **0.0** | home에서 최대 수평거리 | 0–10000 m, **0이면 지오펜스 비활성** |
| `GF_MAX_VER_DIST` | **0.0** | home에서 최대 수직거리 | 0–10000 m, **0이면 비활성** |
| `GF_ACTION` | 2 | 위반 시 동작 | 0 None / 1 Warning / **2 Hold** / 3 Return / 4 Terminate(=kill) / 5 Land |
| `GF_SOURCE` | 0 | 위치 소스 | **0 GPOS**(추정 전역위치) / 1 GPS(원시) |
| `GF_ALTMODE` | 0 | 고도 소스 | 0 추정기 전역고도 / 1 원시 기압계 |
| `GF_COUNT` | -1 | 연속 위반 샘플 수 | -1–10 |
| `GF_PREDICT` | 0 | 예측 트리거 | 공식 경고: *"experimental, may cause flyaways"* |

→ **수정 자체는 `px4-param set`으로 가능. 그런데 지금 켜는 건 권장하지 않는다:**

1. **현재 완전히 꺼져 있다.** 수평·수직 둘 다 0.
2. **VIO-only 기체라 동작이 불확실하다.** `EKF2_HGT_REF = 3`(vision)이고 GPS가 없다. `GF_SOURCE=0`은 추정 전역위치(GPOS)를 쓰는데, GPS 없는 기체의 전역위치는 fake origin 기반이라 home 기준 거리가 의도대로 잡힐지 **실증 필요**.
3. **`GF_ACTION=2`(Hold)는 offboard 미션과 충돌한다.** 위반 시 PX4가 Hold로 모드 전환 → offboard 끊김 → 파이썬 미션 로직이 제어권을 잃는다. 파이썬 쪽 emergency land 경로와 이중으로 얽힐 수 있다.

### 층위 B — 파이썬 레벨 지오펜스 (v13 코드) ← **권장 경로**

| 위치 | 내용 |
|---|---|
| `path_flight_phase1_v13.py:69` | `MAX_ALT_DEVIATION = 1.5` |
| `path_flight_phase1_v13.py:70` | `MAX_HORIZONTAL_DEV = 2.5` |
| `path_flight_phase1_v13.py:2145` | `--max-horizontal-dev` CLI 인자 |

> ⚠️ **값 수정은 자유롭지만, 지금 수정해도 아무 효과가 없다.**
> 이 상수·인자를 **검사하고 발동하는 코드가 v13에 존재하지 않는다** (grep 결과 위 3곳이 전부).
> 즉 "설정 수정 가능?"의 실질적 답: **먼저 강제 로직을 이식해야 설정이 의미를 갖는다.** (docs/03 회귀 이슈)

**결론**: 지오펜스는 **층위 B(파이썬)로 복구**하는 게 맞다. offboard 제어권을 유지한 채 우리가 정의한 emergency land를 태울 수 있기 때문. 층위 A는 최후의 백스톱으로 나중에 검토.

---

## 2. VOA / Collision Prevention 조사 — 핵심 발견

### 2-1. VOXL 쪽은 이미 다 켜져서 돌고 있다

`/etc/modalai/voxl-vision-hub.conf` 실측:

```
"en_voa": true
"voa_send_rate_hz": 20          → 20Hz로 OBSTACLE_DISTANCE MAVLink 송신 중
"voa_pie_slices": 36            → PX4의 36섹터와 일치
"voa_pie_min_dist_m": 0.25      "voa_pie_max_dist_m": 20
"voa_pie_threshold": 3          "voa_pie_bin_depth_m": 0.15
"voa_pie_under_trim_m": 1       → 드론 아래 1m 버블 내 포인트 무시
"voa_upper_bound_m": -0.15      "voa_lower_bound_m": 0.15
```

`voa_inputs`에 5개 항목이 `enabled: true`로 있으나 — **⚠️ config의 `enabled`는 "이 파이프를 구독 시도하라"는 뜻일 뿐 하드웨어 존재를 뜻하지 않는다.** 실측 대조 결과 **3개는 존재하지 않는 파이프를 가리키는 죽은 템플릿 엔트리**다:

| 입력 파이프 | type | frame | depth 범위 | FOV | **실제 존재?** |
|---|---|---|---|---|---|
| `tof` | tof | tof | 0.15–6 m | **106.5×85.1°** | ✅ 파이프·카메라·extrinsics 모두 존재 |
| `rangefinders` | rangefinder | body | 0.3–8 m | 68×56° | ✅ 파이프 존재 |
| ~~`dfs_point_cloud`~~ | point_cloud | stereo_l | 0.3–8 m | 68×56° | ❌ 파이프 없음, 프레임 없음 |
| ~~`stereo_front_pc`~~ | point_cloud | stereo_front_l | 0.3–8 m | 68×56° | ❌ 파이프 없음, 프레임 없음 |
| ~~`stereo_rear_pc`~~ | point_cloud | stereo_rear_l | 0.3–8 m | 68×56° | ❌ 파이프 없음, 프레임 없음 |

**→ 실제 VOA 입력은 ToF 1개 + 하방 rangefinder뿐이다. 이 기체에 스테레오 카메라는 없다.**

#### 근거 — 삼중 확인

**(1) SKU 디코드** (`voxl-inspect-sku`):
```
family code:   MRB-D0014 (starling-2 (D0014))
compute board: 4 (voxl2)      hw version: 1
cam config:    26             modem: 36 (Bots Unlimited WiFi M0213)
tx config:     8 (elrs_m0184) extras: 0 (none)
```

**(2) 카메라 실측** (`/etc/modalai/voxl-camera-server.conf`) — **총 4개, 스테레오 페어 없음**:
| 이름 | 센서 | 종류 |
|---|---|---|
| `tracking_front` | ar0144 | **단안** 트래킹 |
| `tracking_down` | ar0144 | **단안** 트래킹(하방) |
| `hires` | imx412 | 컬러 |
| `tof` | pmd-tof-liow2 | ToF 깊이 |

**(3) MPA 파이프 실측** (`ls /run/mpa/`) — `stereo_*`·`dfs_point_cloud` **0건**.
존재하는 인식 관련 파이프: `tof`, `tof_depth`, `tof_pc`, `tof_ir`, `tof_conf`, `rangefinders`,
`tracking_front*`, `tracking_down*`, `hires_*`, `voa_pc_out`, `ov`/`ov_extended`/`ov_status`.

**(4) extrinsics 프레임** (`/etc/modalai/extrinsics.conf`) — 정의된 프레임 전체:
`body`, `ground`, `hires`, `imu_apps`, `imu_px4`, `lepton0_raw`, `tof`, `tracking_down`, `tracking_front`, `tracking_rear`.
→ VOA가 참조하는 `stereo_l`/`stereo_front_l`/`stereo_rear_l`은 **여기에도 없다.**
→ `tracking_rear` 프레임은 정의만 있고 대응 카메라·파이프가 없는 미사용 템플릿이며, VOA가 쓰는 이름(`stereo_rear_l`)과도 다르다.

### 2-1b. 그래도 VOA 출력은 살아 있다 (ToF 단독으로)

`voxl-inspect-points voa_pc_out` 실측:
```
timestamp(ms)| # points | # bytes |   1st point   | Format
    7088302  |    745   |   8940  | 4.2  3.7 -1.0 | Float XYZ
```
- **약 700–745 포인트 / 프레임**, **20Hz**(50ms 간격, `voa_send_rate_hz`와 일치)
- **Float XYZ, 12 bytes/point** — 파싱 단순
- 프레임마다 값이 변동 → 실제 센서 피드 확인

→ **ToF 단독으로도 VOA 융합 포인트클라우드는 정상 생성된다.** 파이썬 회피 로직의 입력으로 쓰기에 충분하다.

> ⚠️ **커버리지 한계**: 전방위가 아니다. **ToF FOV(106.5°×85.1°, 0.15–6 m) 전방 원뿔 + 하방 rangefinder**가 전부다.
> **측면·후방은 완전 사각지대.** 회피 정책은 이 제약 위에서 설계해야 한다
> (예: 측·후방 회피 기동 금지 / 정지 후 yaw 스캔으로 시야 확보 후 이동).

### 2-2. 그런데 PX4가 전부 버리고 있다

기체 실측 `CP_*` / `COM_OBS_AVOID` / `MPC_POS_MODE`:

| 파라미터 | 기체값 | PX4 기본 | ModalAI 실내 권장 | 판정 |
|---|---|---|---|---|
| `CP_DIST` | **-1.0** | -1 | **1.5** | ❌ **음수 = Collision Prevention 완전 비활성** |
| `CP_DELAY` | 0.4 | 0.4 | **0.0** (vision-hub가 타임스탬프 처리) | ⚠️ 권장과 불일치 |
| `CP_GUIDE_ANG` | 30.0 | 30 | **0.0** | ⚠️ 권장과 불일치 |
| `CP_GO_NO_DATA` | 0 | 0 | **1** | ⚠️ 권장과 불일치 |
| `COM_OBS_AVOID` | **0** | 0 | 1 (rerouting 필요 시) | ❌ 비활성 |
| `MPC_POS_MODE` | **4** (Acceleration based) | 4 | **0 또는 3** | ❌ CP 활성화 불가 값 |

**즉 CP_* 파라미터가 전부 PX4 공장 기본값 그대로다. ModalAI VOA 셋업이 한 번도 적용된 적 없다.**
VOXL이 20Hz로 열심히 보내는 `OBSTACLE_DISTANCE`를 PX4가 그냥 폐기하고 있는 상태.

`MPC_POS_MODE` enum (PX4 v1.14.0 `mc_pos_control_params.c` 소스 확인):
- 0 = Simple position control
- 3 = Smooth position control (Jerk optimized)
- 4 = Acceleration based input ← **현재값, 기본값**

### 2-3. 결정적: offboard 모드에서는 애초에 동작하지 않는다

PX4 v1.14 공식 문서 원문:

> **"Set to 0 or 3 to enable Collision Prevention in Position Mode (default is 4)."**
> — PX4 v1.14 Collision Prevention, MPC_POS_MODE 항목

> Collision Prevention **"can be enabled for multicopter vehicles in Position mode"**

Offboard 모드 문서에는 collision prevention·geofence 언급이 **아예 없고**, 유일하게 명시된 failsafe는 offboard 신호 손실(`COM_OBL_RC_ACT` / `COM_OF_LOSS_T`)뿐이다. 문서는 이렇게 못 박는다:

> "RC control is disabled except to change modes" — **안전 책임 전부가 외부 컨트롤러에 있다.**

**→ docs/02 로드맵의 "핵심 검증 항목"에 문서상 답이 나왔다:**
**현 구조(Offboard position setpoint)에서 VOA는 셋포인트에 개입하지 못한다.**
CP_DIST를 켜고 MPC_POS_MODE를 3으로 바꿔도, Position 모드로 날 때만 효과가 있고 v13의 offboard 미션에는 무관하다.

---

## 3. 로드맵에 미치는 영향

| 로드맵 항목 | 조사 후 판정 |
|---|---|
| 단계1-**(A)** VOXL 내장 VOA 활성화 | ⛔ **현 구조로는 막힘.** Position 모드 전용. offboard 미션에 효과 없음 |
| 단계1-**(B)** 파이썬 레벨 반응형 회피 | ✅ **사실상 유일한 경로. 우선순위 1위로 승격** |
| 단계0 지오펜스 복구 | ✅ 유효. 단 **PX4 `GF_*`가 아니라 파이썬 레벨**로 (offboard 제어권 유지) |

### 다만 (A)가 완전히 무가치하진 않다
- VOA가 만드는 `voa_pc_out`은 **이미 필터링·좌표변환·시간누적(`voa_memory_s`)이 끝난 Float XYZ 포인트클라우드**(20Hz, ~700pt)다.
  파이썬이 이걸 직접 읽으면 `tof_pc` raw를 extrinsics 변환부터 처리하는 것보다 훨씬 싸게 먹힌다.
  → **단계1-(B)의 입력으로 `voa_pc_out`을 쓰는 게 최선.**
  (36섹터 `voa_pie_slices`는 PX4로 보내는 `OBSTACLE_DISTANCE` MAVLink 메시지의 분할 수이지, `voa_pc_out` 파이프의 포맷이 아니다.)
- Position 모드로 수동 비행할 때의 백스톱으로 CP_DIST를 켜두는 건 별개로 유효.
- **단, 입력이 ToF 1개뿐이라 전방 원뿔 + 하방만 커버.** 측·후방 사각지대는 (A)든 (B)든 해결되지 않는다.

---

## 4. 남은 미확인 사항

- [ ] `voa_upper_bound_m = -0.15` / `voa_lower_bound_m = 0.15` **부호 규약** — NED(위가 음수) 기준이면 드론 상하 ±15cm 밴드만 본다는 뜻이 되어 지나치게 좁다. ModalAI 공식 문서에 이 필드 설명이 없어 **소스/포럼 추가 확인 필요**.
- [ ] VIO-only 기체에서 PX4 `GF_*`가 실제로 동작하는지 (전역위치 fake origin 문제).
- [ ] offboard 중 `GF_ACTION=2`(Hold) 발동 시 v13 파이썬 로직의 거동.
- [x] ~~`voa_pc_out` 데이터 포맷~~ → **확인 완료: Float XYZ, 12 B/point, ~700pt, 20Hz.** 단 **좌표계(body 기준인지 local NED인지)는 아직 미확인** — 파이썬 파서 작성 전 필수.
- [ ] 측·후방 사각지대 대응 정책 결정 (정지 후 yaw 스캔 / 전진만 허용 / 저속 제한).
- [ ] 배터리 저전압 failsafe (파이썬 레벨) 현황.

---

## 출처

- [Collision Prevention | PX4 v1.14](https://docs.px4.io/v1.14/en/computer_vision/collision_prevention.html)
- [PX4-user_guide v1.14 collision_prevention.md (원문)](https://github.com/PX4/PX4-user_guide/blob/main/en/computer_vision/collision_prevention.md)
- [Offboard Mode | PX4 v1.14](https://docs.px4.io/v1.14/en/flight_modes/offboard.html)
- [Safety Configuration (Geofence) | PX4 v1.14](https://docs.px4.io/v1.14/en/config/safety.html)
- PX4-Autopilot v1.14.0 소스: `src/modules/navigator/geofence_params.c`, `src/modules/mc_pos_control/mc_pos_control_params.c`
- [VOA | ModalAI Technical Docs](https://docs.modalai.com/voa/)
- [VOXL Vision Hub | ModalAI Technical Docs](https://docs.modalai.com/voxl-vision-hub/)
- [ModalAI Forum — avoidance collision prevention disabled](https://forum.modalai.com/topic/2834/avoidance-collision-prevention-disabled/41)
- [ModalAI Forum — Doubts regarding collision avoidance and collision prevention](https://forum.modalai.com/topic/1499/doubts-regarding-collision-avoidance-and-collision-prevention)
- 기체 실측: `px4-param show GF_* / CP_* / MPC_POS_MODE / COM_OBS_AVOID / EKF2_HGT_REF`, `/etc/modalai/voxl-vision-hub.conf`
