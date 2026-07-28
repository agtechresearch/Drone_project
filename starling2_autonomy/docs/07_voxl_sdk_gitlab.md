# VOXL SDK (GitLab) + 공식 문서 사이트 정리

> 작성일: 2026-07-27
> 조사 방법: GitLab API로 프로젝트 93개 전수 열거 + 핵심 8개 얕은 클론 후 소스 직독 + `docs.modalai.com` 구조 확인
> 기체 미연결 상태 (공개 소스·문서만 사용)
> 선행 문서: `docs/06_modalai_github_org.md` (GitHub 조직 편)

---

## 0. 여기가 VOXL SDK 본체다

`docs/06`에서 밝혔듯 GitHub에는 VOXL SDK가 없다. 전부 여기 있다.

| 위치 | 프로젝트 수 | 내용 |
|---|---|---|
| `gitlab.com/voxl-public/voxl-sdk/**services**` | **36** | 기체에서 도는 서비스 (`voxl-vision-hub`, `voxl-camera-server`, `voxl-mapper` …) |
| `gitlab.com/voxl-public/voxl-sdk/**core-libs**` | **12** | 코어 라이브러리 (`libmodal-pipe` …) |
| `gitlab.com/voxl-public/voxl-sdk/**utilities**` | **25** | 도구 (`voxl-mpa-tools`, `voxl-px4-params` …) |
| `gitlab.com/voxl-public/voxl-sdk/**third-party**` | **20** | 외부 라이브러리 VOXL 빌드 (`voxl-voxblox`, `voxl-opencv` …) |
| **합계** | **93** | |

`voxl-public` 최상위에는 이 외에 `system-image-build`, `voxl-docker-images`,
`flight-core-px4`, `rb5-flight`, `support`, `Deprecated` 서브그룹도 있다.

---

## 1. 이번 조사의 최대 성과 3개

### 발견 A. `voa_pc_out` 파서를 지금 바로 짤 수 있다 — 와이어 포맷 확정

`libmodal-pipe`의 헤더를 직독해 **바이트 단위 포맷을 확정**했다.
`docs/04`의 실측(“Float XYZ, 12B/pt, ~700pt, 20Hz”)과 정확히 정합한다.

**`point_cloud_metadata_t`** (`library/include/pipe_interfaces/point_cloud_metadata_t.h`):

```c
typedef struct point_cloud_metadata_t {
    uint32_t magic_number;   // 0x564F584C  ("VOXL")
    int64_t  timestamp_ns;
    uint32_t n_points;       // 뒤따르는 점 개수
    uint32_t format;         // 0 = FLOAT_XYZ (12B/pt)
    uint32_t id;
    char     server_name[32];
    uint32_t reserved;
} __attribute__((packed)) point_cloud_metadata_t;   // ← packed, 총 60 바이트
```

- **Python `struct` 포맷: `"<IqIII32sI"` → `calcsize` = 60** (검산 완료)
- 헤더 뒤에 `n_points × 12` 바이트의 `float32 XYZ`가 이어짐
- `magic_number` = **`POINT_CLOUD_MAGIC_NUMBER = 0x564F584C`** (`"VOXL"` ASCII).
  카메라·ToF·IMU·pose_4dof와 **같은 값을 공유**하므로 magic만으로 타입 구분 불가 —
  파이프 이름으로 구분해야 한다.

포맷 상수 전체:

| 값 | 이름 | 바이트/점 |
|---|---|---|
| **0** | `FLOAT_XYZ` | **12** ← `voa_pc_out`이 이것 |
| 1 | `FLOAT_XYZC` (+신뢰도) | 16 |
| 2 | `FLOAT_XYZRGB` | 15 |
| 3 | `FLOAT_XYZCRGB` | 19 |
| 4 | `FLOAT_XY` | 8 |
| 5 | `FLOAT_XYC` | 12 |

**MPA 파이프 구독 프로토콜** (`library/src/client.c`, `server.c` 직독):

```
1. 베이스 디렉토리는 /run/mpa/           (MODAL_PIPE_DEFAULT_BASE_DIR)
2. 서버 생존 확인: /run/mpa/<파이프>/request 존재 여부
3. 클라이언트 이름 생성: "<내이름>" + 8자리 0패딩 난수  ("%08d", 0~99999999)
4. 그 이름을 request FIFO에 write (NULL 종료 문자 포함)
5. 서버가 /run/mpa/<파이프>/<그이름> 으로 mkfifo 생성
6. 클라이언트가 그 FIFO를 open 후 read → [60B 헤더][12B×n_points] 반복
```

디렉토리 구성: `request`(FIFO) · `info`(JSON 메타) · `control`(FIFO, 선택적) · 클라이언트별 FIFO들.

→ **`libmodal_pipe.so`에 링크할 필요 없이 순수 파이썬(`open`+`struct.unpack`)으로 구독 가능하다.**
이게 우리 v14(MAVSDK-Python) 구조에 그대로 맞는다.

> **주의**: `libmodal-pipe`에 `python/pympa.py`가 있지만 **66줄이고 IMU·카메라 전용**이다
> (`publish_imu`, `get_image`, `camera_subscribe`). 포인트클라우드 지원이 없고,
> 자동생성 `pympa_types` 모듈과 `libmodal_pipe.so`를 요구한다.
> **우리 용도로는 쓸 수 없다** → 위 프로토콜대로 직접 구현하는 것이 맞다.

### 발견 B. ModalAI 자신이 "VOA는 Position 모드 전용"이라고 파라미터 파일에 적어놨다

`voxl-px4-params/params/v1.14/voa_helpers/voa_enable_indoor.params` 원문 주석:

```
############################################################
# TEMPORARILY SWAP POS MODE TO 0 FOR VOA
# this is because PX4 mainline currently does not support
# VOA in the default position mode of 4
############################################################
1	1	MPC_POS_MODE	0	6
```

**이제 3중으로 확증됐다:**

| 근거 | 출처 | 문서 |
|---|---|---|
| 1. PX4 공식 문서 문장 | docs.px4.io | `docs/04` |
| 2. 기체 실제 펌웨어 소스 (`CollisionPrevention`이 `FlightTaskManualPosition`에만) | `modalai/px4-firmware` `v1.14.0-2.0.133-dev` | `docs/06` §1-B |
| 3. **ModalAI 자체 파라미터 파일 주석** | `voxl-px4-params` | **이 문서** |

→ **`docs/02` 로드맵 "단계1-(A) VOA 활성화 경로 폐기" 판단은 완전히 확정.
파이썬 레벨 회피(B)가 유일한 경로다.** 더 검증할 것 없음.

### 발견 C. ModalAI의 회피 기본값 — 우리 임계값의 출발점으로 쓸 수 있다

`voa_helpers/` 3개 파일 전문을 확보했다.

| 파라미터 | 실내 | 실외 | 의미 |
|---|---|---|---|
| `MPC_POS_MODE` | 0 | 0 | VOA용으로 임시 전환 (기본 4) |
| **`CP_DIST`** | **1.0 m** | **3.0 m** | **최소 허용 접근거리** |
| `CP_DELAY` | 0.0 | 0.0 | ModalAI 주석: *"정확한 타임스탬프·지연보상을 하므로 0"* |
| `CP_GUIDE_ANG` | 0.0 | 0.0 | ModalAI 주석: *">0이면 예측불가하게 동작, 0으로 두라"* |
| **`CP_GO_NO_DATA`** | **1** | **1** | **센서 커버리지가 없는 방향으로도 이동 허용** |

`voa_disable.params`는 `MPC_POS_MODE`를 4로, `CP_DIST`를 -1로 되돌린다.

**우리 회피 정책 설계에 직결되는 해석 3개:**

1. **`CP_DIST` 1.0m(실내)/3.0m(실외)** — ModalAI가 이 기체에 권장하는 정지거리다.
   우리 파이썬 회피의 임계값을 **맨땅에서 정하지 않고 여기서 출발**할 수 있다.
   ToF 유효범위가 0.15–6m이므로 실내 1.0m는 여유가 충분하고 실외 3.0m도 범위 안이다.
2. **`CP_GO_NO_DATA=1`이 핵심 시사점이다.** ModalAI는 전방 ToF만 있는 이 기체에서
   **"센서가 못 보는 방향으로의 이동을 막지 않는" 쪽을 기본으로 택했다.**
   즉 **측·후방 사각지대에 대한 ModalAI의 공식 답은 "보호하지 않는다"**다.
   → 우리가 사각지대까지 방어하려면 **ModalAI 기본 정책보다 더 보수적으로** 직접 설계해야 한다.
   (유력안인 "정지 후 yaw 스캔"은 이 공백을 메우는 방향이라 타당하다)
3. `CP_DELAY=0`의 근거가 "vision-hub가 타임스탬프·지연보상을 정확히 한다"는 것 →
   `voa_pc_out`의 모션보상 품질을 ModalAI가 신뢰한다는 뜻. `docs/04`에서 확인한
   "VIO 기반 모션보상 적용됨"과 정합.

> **충돌 주의**: `starling_2_indoor_position.params`는 `MPC_POS_MODE`를 **4**로 두는데
> `voa_enable_*.params`는 **0**을 요구한다. 두 파일을 같이 적용하면 뒤에 로드된 쪽이 이긴다.
> 우리는 어차피 VOA를 안 쓰므로 실무상 문제는 없지만, 파라미터 파일을 섞어 적용할 때 알아야 할 함정.

---

## 2. `voxl-px4-params` — 우리 기체 공식 파라미터 (관련도 최상)

`utilities/voxl-px4-params`. **우리 SKU `MRB-D0014-4-V1-C26-T8-M36-X0` → `D0014` = Starling 2.**

```
params/
├── v1.10, v1.11, v1.12, v1.13      구버전 (v1.12에 Starling_V2 rev A~E)
├── v1.14/          ★ 우리 기체 PX4 버전
│   ├── platforms/
│   │   ├── D0014_Starling_2.params        ★★★ 우리 기체
│   │   ├── D0012_Starling_2_Max.params
│   │   └── D0005_Starling.params          (구 Starling)
│   ├── voa_helpers/          ★★★ voa_disable / voa_enable_indoor / voa_enable_outdoor
│   ├── other_helpers/        ★★  starling_2_indoor_position / _max_indoor / _max_outdoor
│   │                             hitl.params, hitl_joystick_vfc.params, msp_dp_osd*
│   ├── EKF2_helpers/         ★   상태추정 튜닝
│   ├── battery_helpers/      ★   배터리 (저전압 failsafe 검토 시)
│   ├── radio_helpers/, fpv_helpers/, voxl2_io_helpers/, ci_helpers/
│   └── experimental_do_not_use/   (이름 그대로 쓰지 말 것)
└── v1.18/          동일 구조 (ModalAI 최신 라인)
docs/fpv-v1.14-to-v1.18-param-migration.md
scripts/, old_tools/, pkg/
```

### `D0014_Starling_2.params` (227행) 실측 발췌

```
# Hardware-Specific Configuration for ModalAI D0014 Starling 2
# This is NOT a complete set of PX4 parameters. These are non-default parameters
# specific to the airframe and are meant to be loaded over the defaults.
# work in progress
SYS_AUTOSTART   4001      (쿼드콥터 X)
MAV_TYPE        2
SENS_BOARD_ROT  0
EKF2_EV_POS_X/Y/Z  0.0    (VIO 위치 오프셋)
EKF2_EV_QMIN    16
EKF2_HGT_REF    3         ← 기체 실측값과 일치 (docs/04)
```

**`EKF2_HGT_REF=3`이 공식 플랫폼 파일과 기체 실측이 일치한다** → 우리 기체는
ModalAI 공식 Starling 2 설정을 그대로 쓰고 있다. 누가 임의로 바꿔놓은 게 아니다.

**이 파일에 `GF_*`·`CP_*`·`COM_OBS_AVOID`가 없다.** `docs/04`의 "전부 공장 기본값"
실측과 정합하며, `docs/06` §2.5의 "부팅 스크립트도 설정하지 않음"과 합쳐 결론이 선다:
**ModalAI는 지오펜스·회피를 기본으로 켜주지 않고, 원하면 helper 파일을 별도 적용하는 설계다.**
→ 우리가 v14에서 파이썬 레벨로 지오펜스를 만든 건 이 설계와 어긋나지 않는다.

### `starling_2_indoor_position.params` — 실내 타이트 추종 튜닝 (실측)

ModalAI가 *"a focus on tight path following indoors"*라고 명시한 파일. 우리 지그재그
커버리지 미션과 목적이 같다.

| 항목 | 값 | 우리 관련 |
|---|---|---|
| `MPC_XY_VEL_MAX` / `MPC_XY_CRUISE` / `MPC_VEL_MANUAL` | **3.0 m/s** | 실내 속도 상한 |
| `MPC_Z_VEL_MAX_UP/DN`, `MPC_LAND_SPEED` | 1.0 m/s | 수직 속도 |
| `MPC_XY_P` | 3.50 | **수평 위치 P — 추종 오차 크기를 좌우** |
| `MPC_XY_VEL_P_ACC` / `I` / `D` | 3.00 / 0.10 / 0.00 | |
| `MPC_Z_P` | 5.0 | 수직 (*"very tight"*) |
| `MPC_ACC_HOR` / `MPC_JERK_MAX` | 3.0 / 40.0 | |
| `MPC_TILTMAX_AIR` / `MPC_MAN_TILT_MAX` | 30° / 35° | |
| `MPC_HOLD_MAX_XY/Z` | 0.0 | pos/vel 피드백 전환 비활성 |
| `MPC_ACC_HOR_MAX` | 1000.0 | ModalAI 주석: *"PX4가 altitude 모드로 떨어지는 원인일 수 있어 한계를 사실상 없앰. 대신 tiltmax로 제한"* |

> **v14 지오펜스 임계값과의 연결**: `--max-horizontal-dev 2.5`는 "실제 위치 vs 명령
> setpoint" 오차 한계다. 추종 오차는 `MPC_XY_P`·속도 상한·명령 setpoint의 변화율로 결정된다.
> 위 값들이 우리 기체 실제 파라미터와 같은지 **기체에서 대조하면 2.5m가 타당한지
> 책상에서 1차 판단이 가능하다.** (다음 세션 작업 후보)

---

## 3. `libmodal-pipe` — MPA의 심장 (관련도 최상)

`core-libs/libmodal-pipe`. VOXL에서 파이프 데이터를 다루는 **모든** 코드가 이걸 쓴다
(`docs/06`에서 본 `nanotrack` 의존성에도, PX4의 `boards/modalai/voxl2/src/lib/mpa`에도 등장).

```
library/include/
├── modal_pipe.h                 (통합 헤더)
├── modal_pipe_client.h          구독자 API
├── modal_pipe_server.h          발행자 API
├── modal_pipe_sink.h            싱크
├── modal_pipe_common.h / _defines.h / _deprecated.h / _interfaces.h
├── modal_pipe_buffers.h         (DMA/GBM 버퍼)
├── modal_start_stop.h           프로세스 라이프사이클
└── pipe_interfaces/             ★ 자료형 정의 — 각 파이프의 바이트 포맷
    ├── point_cloud_metadata_t.h ★★★ voa_pc_out, tof_pc (§1-A)
    ├── pose_vel_6dof_t.h        ★★★ px4_vehicle_local_position (우리 도달판정)
    ├── pose_vel_6dof2_t.h, pose_4dof_t.h
    ├── vio_data_t.h             ★★ OpenVINS 출력 (VIO 품질 감시)
    ├── tof_data_t.h, tof2_data_t.h  ★★ ToF 원본
    ├── imu_data_t.h, baro_data_t.h, magic_number.h
    ├── camera_image_metadata_t.h, mpa_ion_buf_t.h
    ├── object_tracking_t.h, tag_detection_t.h
    ├── rc_channels_t.h, crsf_raw_t.h, mavlink_message_t.h
    ├── payload_status_t.h, cpu_stats2_t.h, vfc_data_t.h, vqf_data_t.h
library/src/  client.c  server.c  sink.c  common.c  interfaces.c  misc.c
              start_stop.c  pympa.cpp  buffers.cpp  buffers/{dma,gbm}.cpp
python/       pympa.py (66행, IMU·카메라만 — §1-A 주의)  test/test_pympa.py
examples/     modal-hello-{server,client,sink,pause}.c  modal-pipe-ping.c
              modal-kill-pipe.c  modal-test-fault-code.c
tools/        voxl-inspect-pipe-info.c
```

**magic number 표** (`magic_number.h` 실측) — 파이프 파싱 시 헤더 검증에 쓴다:

| 상수 | 값 | ASCII |
|---|---|---|
| `POINT_CLOUD` / `CAMERA` / `TOF` / `IMU` / `TAG_DETECTION` / `POSE_4DOF` / `POSE_VEL_6DOF2` / `CPU_MON` | `0x564F584C` | `"VOXL"` (공용) |
| `POSE_VEL_6DOF` | `0x706F7365` | `"pose"` |
| `TOF2` | `0x564F584D` | `"VOXM"` |
| `VIO` | `0x05455524` | |
| `BARO` | `0x4241524F` | `"BARO"` |
| `RC_CHANNELS` | `0x52434348` | `"RCCH"` |
| `VQF` | `0x56514644` | `"VQFD"` |
| `PAYLOAD_STATUS` | `0x5041594C` | `"PAYL"` |
| `CRSF_RAW` | `0x43525346` | `"CRSF"` |
| `CPU_STATS2` | `0x43505532` | `"CPU2"` |
| `MPA_ION_BUFFER` | `0x494F4E42` | `"IONB"` |
| `EXT_VFC` | `0x45564643` | `"EVFC"` |
| `VFC` / `OT` | `0x05455525` / `0x05455526` | |

> `px4_vehicle_local_position`은 `POSE_VEL_6DOF_MAGIC_NUMBER = 0x706F7365`(`"pose"`)로
> **구분 가능**하다. 반면 `voa_pc_out`은 `"VOXL"` 공용값이라 파이프 이름에 의존해야 한다.

**참고 구현 순서 (다음 세션)**: `examples/modal-hello-client.c` → `library/src/client.c`
→ 우리 파이썬 구현. `tools/voxl-inspect-pipe-info.c`는 `info` JSON 읽는 예시.

---

## 4. `voxl-mapper` — 회피/매핑을 근본적으로 바꿀 수 있는 후보

`services/voxl-mapper`. **`docs/02`에서 "미설치"로만 적어뒀던 그 패키지의 소스다.**

README 첫 문장: *"3d Mapping and Path Planning"* + 경고 배너:

> ⚠️ **"This project is only authorized for beta usage"**
> *"MAKE SURE YOU ARE FOLLOWING ALL PROPER SAFETY PROTOCOLS BEFORE ATTEMPTING
> AUTONOMOUS FLIGHT. ModalAI is NOT responsible for any damages caused by flight path."*

### 구조 (실측)

```
server/
├── voxl-mapper/                    ★ ModalAI 자체 코드
│   ├── main.cc
│   ├── voxl_mapper.cc/.h           맵 서버 (voxblox TSDF/ESDF)
│   ├── voxl_planner.cc/.h          경로계획 (TrajectoryProtocol 포함)
│   ├── obs_pc_filter.cc/.h         ★ 장애물 포인트클라우드 필터
│   ├── config_file.cc/.h           ★ ToF 0/1/2 입력 설정
│   ├── global_planners/            전역 계획
│   ├── local_planners/             지역 계획
│   ├── planner_utils, conversions, timing_utils
│   ├── rc_transform(.c) / _ringbuf  좌표변환 (librc-math)
│   └── mesh_vis.h, path_vis.h, ptcloud_vis.h   시각화(voxl-portal)
├── loco_planner/                   ETH continuous-time 최적화 플래너
├── mav_trajectory_generation/      다항식 궤적 생성
├── mav_path_smoothing/             loco / polynomial / velocity_ramp 스무더
├── mav_local_planner/, mav_planning_common/  (yaw_policy, physical_constraints 등)
├── mav_comm/ (mav_msgs, mav_planning_msgs)
└── voxblox_planning_common/
debug_tools/  images/  services/  pkg/  build.sh  install_build_deps.sh
```

의존성: `voxl-voxblox`, `voxl-ceres-solver`, `voxl-nlopt`, `libmodal-pipe`,
`voxl-mpa-tools`, `voxl-vision-hub`, `voxl-mavlink`, `libmodal-json`, `libvoxl-cutils`

### 우리 기체에서 쓸 수 있나 — 판단 근거

**긍정적 (중요)**: `config_file.h`에 **`tof_0` / `tof_1` / `tof_2`** 각각에 대해
`_pipe`, `_enable`, `_rate`, `tf_tof_N_wrt_body`(외부파라미터), `_extrinsics_N_name`이
정의돼 있다. **즉 ToF만으로 동작하도록 설계돼 있다.**
`docs/04`에서 확인한 우리 제약(**스테레오 없음, ToF 1개 + 하방**)이 이 패키지를 배제하지 않는다.
(소스에 `stereo_l`/`stereo_front_l`/`stereo_rear_l` 참조도 있지만 선택적 입력이다.)

**부정적 / 미해결**:
- **beta 전용**이고 ModalAI가 책임 면제를 명시했다.
- 빌드에 **`voxl-cross` 도커 이미지(≥4.4)** 가 필요하다. 기체에서 바로 빌드하는 게 아니라
  크로스컴파일 후 ipk/deb 설치. 플랫폼 인자 `qrb5165`(18.04) / `qrb5165-2`(20.04) 구분 필요.
- **출력 경로가 우리 구조와 충돌할 소지가 크다.** `voxl_planner`가 `TrajectoryProtocol`로
  `trajectory_t`를 내보내는데, 이게 vision-hub를 거쳐 PX4로 가는 흐름이면
  **우리 파이썬의 offboard setpoint와 제어권을 다툰다.** 정확한 흐름 미확인.
- 우리 미션은 `MISSION_PLAN` 지그재그 **커버리지**로, "A→B 최단경로 계획"과 목적이 다르다.
  전역 플래너가 우리 커버리지 패턴을 대체해버리면 미션 의도가 깨진다.

### 판단

**당장 도입하지 않는 게 맞다.** 이유: beta + 제어권 충돌 위험 + 우리 미션 형태 불일치 +
v14 실비행 검증조차 안 끝난 상태에서 변수를 크게 늘린다.
다만 **`obs_pc_filter.cc`는 별도로 읽을 값이 크다** — ModalAI가 ToF 포인트클라우드에서
장애물을 어떻게 걸러내는지의 참조 구현이고, 우리 파이썬 필터 설계에 직접 참고된다.
→ 다음 세션 후보로 기록.

---

## 5. 그 외 우리 프로젝트 관련 SDK 프로젝트

### 5.1 services (36개) — 관련도 순

| 프로젝트 | 우리 관련 | 비고 |
|---|---|---|
| **`voxl-vision-hub`** | ★★★ | VOA·MAVLink 중계 허브. `docs/04`에서 소스 직독 완료 (`voa_manager.c`, `geometry.c`) |
| **`voxl-mapper`** | ★★★ | §4 |
| **`voxl-px4`** | ★★★ | PX4 래퍼/CI. `px4-firmware`를 서브모듈로 두고 설정·서비스 관리. `documentation/`, `build-{apps,slpi,fc}.sh`, `test/`, `services/` |
| `voxl-open-vins-server` | ★★ | 우리 VIO. `server/`, `clients/`, `external/`, `opencv_compat/`, **`ROADMAP`** 파일 존재 |
| `voxl-camera-server` | ★★ | 카메라 4개(`tracking_front/down`, `hires`, **`tof`**) 서빙. 우리 ToF의 실제 서버 |
| `voxl-mavlink-server` | ★★ | MAVLink 라우팅. MAVSDK 접속 경로 |
| `voxl-portal` | ★★ | **웹 UI.** 브라우저에서 파이프·포인트클라우드 시각화 → 회피 개발 시 디버깅에 유용 |
| `qrb5165-rangefinder-server` | ★ | 하방 거리센서(`rangefinders`) |
| `voxl-mini-tof-server` | ★ | **VL53L8** ToF (멀티플렉서 지원). §5.4 참고 |
| `voxl-px4-imu-server`, `voxl-imu-server`, `voxl-barometer-server` | ☆ | 센서 서버 |
| `voxl-tflite-server`, `voxl-nano-tracker`, `voxl-tag-detector`, `voxl-feature-tracker`, `voxl-flow-server` | ☆ | 인식·추적 |
| `voxl-streamer`, `voxl-hires-server`, `voxl-stitcher`, `voxl-mavcam-manager`, `voxl-uvc-server`, `voxl-gphoto2-server`, `voxl-lepton-server` | ☆ | 영상 |
| `voxl-qvio-server` | ☆ | 구 VIO (OpenVINS 이전) |
| `qrb5165-dfs-server` | ☆ | **스테레오 깊이(DFS)** — 우리 기체는 스테레오 없어 무효 (`docs/04`) |
| `voxl-fault-manager`, `voxl-cpu-monitor`, `voxl-rc-server`, `voxl-joystick-server`, `voxl-remote-id`, `voxl-rpx-server`, `voxl-iim-server`, `qrb5165-chirp-server`, `voxl-mainline-px4`, `apq8096-camera-server` | ☆ | |

### 5.2 core-libs (12개)

| 프로젝트 | 우리 관련 |
|---|---|
| **`libmodal-pipe`** | ★★★ §3 |
| `librc-math` | ★★ 좌표변환·회전(`rc_tf_t`). `voa_pc_out` 좌표 다룰 때 규약 참고 |
| `libmodal-json` | ★★ 모든 `.conf` 파싱 (vision-hub·mapper 설정) |
| `libmodal-journal` | ★ 로깅 |
| `libmodal-exposure` | ☆ 카메라 노출 |
| `libfc-sensor-api` | ☆ SLPI 센서 API (PX4 쪽에도 동일 이름 존재) |
| `libslpi-link-api` | ☆ SLPI 통신 (GitHub `ap_host`에도 등장) |
| `libvoxl-cutils`, `libqrb5165-io`, `libvoxl-cci-direct`, `libvoxl-codec`, `voxl-slpi-uart-bridge` | ☆ |

### 5.3 utilities (25개)

| 프로젝트 | 우리 관련 | 비고 |
|---|---|---|
| **`voxl-px4-params`** | ★★★ §2 |
| **`voxl-mpa-tools`** | ★★★ 우리가 이미 쓰는 `voxl-inspect-*` 전부. §5.5 |
| `voxl-logger` | ★★ **MPA 데이터 기록·재생.** 회피 로직을 실비행 데이터로 오프라인 검증하는 길 |
| `voxl-configurator` | ★★ 설정 일괄 관리 |
| `voxl-replay-analysis` | ★★ 기록 데이터 분석 |
| `voxl-camera-calibration` | ★ 카메라 캘리브레이션 |
| `voxl-benchmark-vio` | ★ **VIO 성능 측정** — VIO 신뢰도 검토 시 |
| `voxl-hitl-vio-server`, `voxl-hitl-rangefinder-server` | ★ **HITL** (`docs/06` §2.5의 HITL 경로와 연결) |
| `voxl-mpa-to-ros`, `voxl-mpa-to-ros2` | ★ ROS 연동 필요 시 |
| `voxl-utils`, `voxl-esc`, `voxl2-io`, `voxl-time-sync`, `voxl-modem`, `voxl-elrs`, `voxl-tunnel`, `voxl-docker-support`, `voxl-reset-slpi`, `voxl-bind-spektrum`, `voxl-focus-camera`, `voxl-uqmi`, `voxl-mpa-abi-validator`, `qrb5165-system-tweaks` | ☆ |

### 5.4 third-party (20개)

우리 관련: **`voxl-voxblox`**(★★ mapper의 TSDF/ESDF 엔진), `voxl-ceres-solver`·`voxl-nlopt`
(★ mapper 최적화), `voxl-opencv`·`voxl-eigen3`(★ 공통), `voxl-mavlink`(★ MAVLink 정의),
`voxl-libnanotrack`·`voxl-libncnn`·`qrb5165-tflite`(☆ 추론), `voxl-ros2-foxy`·
`voxl-microdds-agent`(☆ ROS2), 나머지(`voxl-ffmpeg`, `voxl-jpeg-turbo`, `voxl-libgeographic`,
`voxl-libgphoto2`, `voxl-libuvc`, `voxl-libyaml`, `voxl-mongoose`, `voxl-googletest`, `voxl-opencl`) ☆

> **`voxl-mini-tof-server` 관련 주의 + 기회**: 이 서비스는 **ST VL53L8**용이고
> `docs/`에 **멀티플렉서** 자료(`VL53L8_MULTIPLEXER.pdf`, `VL53L8_SETUP_SCHEMATIC`)가 있다.
> 우리 기체 ToF는 `pmd-tof-liow2`(다른 부품, `voxl-camera-server`가 서빙)이므로 **직접 관련은 없다.**
> 다만 **VL53L8 여러 개를 멀티플렉서로 붙이는 공식 경로가 존재한다**는 뜻이라,
> `docs/04`의 최대 제약인 **측·후방 사각지대를 하드웨어로 메우는 옵션**이 될 수 있다.
> → 회피 정책 설계 시 "소프트웨어로만 해결" 외의 선택지로 검토 가치 있음. **단 미검증 가설이다**
> (전원·I2C 여유, 마운트, extrinsics 등 확인 필요).

### 5.5 `voxl-mpa-tools/tools` — 사용 가능한 진단 도구 전체 (실측)

우리가 이미 `voxl-inspect-pose`를 쓰고 있는데, 그 형제들이 이만큼 있다:

```
voxl-inspect-points     ★★★ 포인트클라우드 (voa_pc_out 검증에 바로 사용!)
voxl-inspect-pose       ★★★ (현재 사용 중)
voxl-inspect-tof        ★★★ ToF 원본
voxl-inspect-vio        ★★  VIO 상태·품질
voxl-inspect-vio-cams   ★★
voxl-inspect-cam / -cam-ascii   ★  카메라 (ascii는 SSH에서 바로 보기)
voxl-inspect-extrinsics ★★  좌표계 확인 (docs/04에서 이미 활용)
voxl-inspect-battery    ★★  배터리 (저전압 failsafe 검토)
voxl-inspect-mavlink    ★★  MAVLink 트래픽
voxl-inspect-imu / -baro / -gps / -vibration / -detections / -osd / -ion-stream
voxl-integrate-gyro-3d, voxl-check-camera-repetition
voxl-kill-pipe, voxl-record-raw-image, voxl-record-video
```

> **다음 세션 즉시 활용**: `voxl-inspect-points voa_pc_out`으로 §1-A의 포맷 해석이
> 맞는지 기체에서 **파이썬 코드를 짜기 전에** 확인할 수 있다.
> (`docs/04` 메모대로 출력은 `tail` 말고 `head`로 받을 것)

---

## 6. 공식 문서 사이트 `docs.modalai.com` 구조

소스 못지않게 문서 사이트가 잘 정리돼 있다. 우리 주제와 맞는 경로:

| 섹션 | 경로 | 우리 관련 항목 |
|---|---|---|
| **VOXL SDK** | `/voxl-sdk/` | **VOA(Vision-based Obstacle Avoidance)**, **VOXL Mapper(매핑·경로계획)**, VIO, Open-VINS 서버, MPA 설정·extrinsics, VOXL Vision Hub, **VOXL PX4 통합**, Inspect 도구, 로깅·재생, TFLite, Apriltag, ROS/ROS2, 커스텀 앱 개발, 릴리스노트(0.9~1.6) |
| **Dev Drones** | `/voxl-dev-drones/` | **Starling 2 / Starling 2 Max** 사용자 가이드 |
| **VOXL Developer Bootcamp** | `/voxl-developer-bootcamp/` | VOXL2·Starling 퀵스타트, ADB·WiFi 설정, 캘리브레이션 점검, QGC 연결, **Starling 첫 비행** |
| VOXL Autopilot & Compute | `/voxl-computers/` | VOXL2 사용자 가이드·데이터시트 |
| VOXL Dev Kits / FPV / 모뎀·확장보드·액세서리 | `/voxl-dev-kits/`, `/fpv/`, `/modems/` 등 | ☆ (사각지대 하드웨어 검토 시 액세서리 참고) |

**우선 읽을 문서 3개 (다음 세션)**:
1. `/voxl-sdk/` → **VOA 페이지** — `voa_helpers` 파라미터의 공식 설명과 절차
2. `/voxl-sdk/` → **VOXL Mapper 페이지** — §4의 미해결점(출력 경로·제어권)을 여기서 확인 가능
3. `/voxl-sdk/` → **MPA 설정 / extrinsics 페이지** — 파이프·좌표계 규약 공식 설명

> 릴리스노트가 1.6까지 있고 우리 기체는 **voxl-suite 1.6.3**(`docs/04`)이므로
> 릴리스노트에서 우리 버전의 알려진 이슈를 확인할 수 있다.

---

## 7. 이 조사로 갱신되는 프로젝트 판단

### 확정 (추정 → 사실)

| 항목 | 지금 상태 |
|---|---|
| VOA/CP가 Offboard에서 무효 | **3중 확증 완료** (PX4문서 + 기체 펌웨어 소스 + ModalAI 파라미터 주석). 더 검증 불필요 |
| `voa_pc_out` 파서 가능성 | **와이어 포맷 60B 헤더 + 12B/pt 확정, 구독 프로토콜 확정. 순수 파이썬으로 구현 가능** |
| 우리 기체 파라미터 정상성 | **`EKF2_HGT_REF=3` 등이 ModalAI 공식 `D0014_Starling_2.params`와 일치** |
| 지오펜스·회피 기본 꺼짐 | **의도된 설계** (플랫폼 파일·부팅스크립트 둘 다 미설정, helper 별도 제공) |
| ModalAI의 사각지대 정책 | **`CP_GO_NO_DATA=1` — "보호하지 않음"이 공식 기본값** |
| `voxl-mapper` 가용성 | **소스 존재, ToF-only 설계로 우리 하드웨어와 호환. 단 beta + 제어권 충돌 위험 → 당장 도입 보류** |

### 다음 세션 작업 (우선순위)

1. **`voa_pc_out` 파이썬 파서 작성** — §1-A로 근거가 다 갖춰졌다. 기체 연결 시
   `voxl-inspect-points voa_pc_out`으로 먼저 검증 → 그 다음 파서.
2. **회피 정책 설계 (사용자와 함께)** — 이제 근거가 늘었다:
   `CP_DIST` 1.0m(실내)/3.0m(실외)를 임계값 출발점으로, `CP_GO_NO_DATA=1`이
   ModalAI의 사각지대 포기를 뜻한다는 점을 감안해 우리는 더 보수적으로.
   **운용 환경(온실/실내/실외) 확인이 여전히 선행 조건.**
3. **`obs_pc_filter.cc` 직독** — ModalAI의 ToF 장애물 필터 참조 구현.
4. **`docs.modalai.com` VOA·Mapper·MPA 페이지 정독** (§6).
5. **v14 임계값 책상 검토** — `starling_2_indoor_position.params`(§2)와 기체 실제 파라미터를
   대조해 `--max-horizontal-dev 2.5`의 타당성을 실비행 전에 1차 판단.
6. `px4-flight-review`로 과거 ulog 추종오차 분석 (`docs/06` §5와 동일 항목).
7. 배터리 저전압 failsafe — `voxl-px4-params/v1.14/battery_helpers/` + `voxl-inspect-battery` 먼저 확인.

---

## 8. 재현용 명령 / 조사 범위

```bash
# 서브그룹·프로젝트 전수 열거 (인증 불필요)
curl -s "https://gitlab.com/api/v4/groups/voxl-public%2Fvoxl-sdk/subgroups?per_page=100"
curl -s "https://gitlab.com/api/v4/groups/voxl-public%2Fvoxl-sdk%2Fservices/projects?per_page=100"

# 얕은 클론 (트리 전체, 파일내용은 필요할 때만)
git clone --depth 1 --filter=blob:none \
  https://gitlab.com/voxl-public/voxl-sdk/core-libs/libmodal-pipe.git
git clone --depth 1 --filter=blob:none \
  https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-px4-params.git

# 포인트클라우드 헤더 크기 검산
python -c "import struct; print(struct.calcsize('<IqIII32sI'))"   # → 60
```

### 조사 범위와 한계 (정직한 기록)

- **프로젝트 93개 전수**를 API로 열거하고 이름·설명을 확보했다. §5 표는 93개를 빠짐없이 담았다.
- **소스를 직독한 것은 8개**: `libmodal-pipe`, `voxl-mapper`, `voxl-vision-hub`,
  `voxl-mini-tof-server`, `voxl-mpa-tools`, `voxl-px4-params`, `voxl-open-vins-server`, `voxl-px4`.
- **파일 단위로 정독한 것**은 그중에서도 우리 작업에 직결되는 것들이다:
  `point_cloud_metadata_t.h`, `magic_number.h`, `client.c`/`server.c`(프로토콜 부분),
  `pympa.py`, `voa_*.params` 3개, `D0014_Starling_2.params`, `starling_2_indoor_position.params`,
  `voxl-mapper/README` + `config_file.h` + 파일목록.
- **아직 안 본 것 (다음 세션 몫)**:
  - `voxl-mapper/obs_pc_filter.cc` 내용 (존재만 확인)
  - `voxl-mapper`의 출력 경로 = `TrajectoryProtocol`이 PX4에 어떻게 도달하는지 (§4 미해결)
  - `voxl-vision-hub` 나머지 (VOA 부분은 `docs/04`에서 이미 봄)
  - `voxl-open-vins-server`의 `ROADMAP`, `voxl-px4/documentation/`
  - `voxl-px4-params/v1.14/EKF2_helpers`, `battery_helpers` 내용
  - `docs.modalai.com` 개별 페이지 본문 (구조만 확인, VOA·Mapper 페이지 미정독)
  - services/core-libs/utilities/third-party 중 나머지 85개 프로젝트 소스
- **기체 미연결 상태**라 이 문서의 내용은 **공개 소스·문서 기반**이다. §1-A의 포맷 해석은
  헤더 직독 + `struct.calcsize` 검산 + `docs/04`의 과거 실측치와의 정합으로 뒷받침되지만,
  **기체에서 `voxl-inspect-points`로 재확인하는 절차를 다음 세션에 넣었다.**
