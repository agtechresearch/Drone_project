# ModalAI GitHub 조직 전수 정리 (github.com/modalai)

> 작성일: 2026-07-27
> 조사 방법: GitHub API로 저장소 44개 전수 열거 + 핵심 저장소 얕은 클론(`--depth 1 --filter=blob:none`) 후 실제 디렉토리·파일 직독
> 기체 미연결 상태에서 수행 (공개 소스만 사용)

---

## 0. 가장 먼저 알아야 할 것 — 여기엔 VOXL SDK가 없다

**`github.com/modalai`의 저장소 44개 중 37개(84%)가 upstream 오픈소스 프로젝트의 fork다.**
ModalAI 자체 저장소는 **7개뿐**이고, 그중 우리 프로젝트에 관련된 건 거의 없다.

우리가 기체에서 매일 쓰는 `voxl-vision-hub`, `voxl-camera-server`, `voxl-mapper`,
`libmodal-pipe`, `voxl-mpa-tools` 같은 **VOXL SDK 본체는 GitHub에 아예 없다.**
전부 **GitLab `gitlab.com/voxl-public`** 에 있다 (→ `docs/07_voxl_sdk_gitlab.md`).

| 플랫폼 | 무엇이 있나 | 우리 프로젝트 관련도 |
|---|---|---|
| **GitHub** `modalai` | upstream fork(PX4·ArduPilot·OpenVINS·베타플라이트·무선링크·커널드라이버) + 자체 7개 | **PX4 펌웨어 소스 하나가 결정적**, 나머지는 낮음 |
| **GitLab** `voxl-public` | VOXL SDK 전체 (서비스 36 + 코어라이브러리 12 + 유틸 25 + 서드파티 20 = **93개**) | **매우 높음** |

> GitHub에서 `voxl`로 이름이 시작하는 저장소는 `Voxl-Plugin` 단 하나이고, 그마저도
> 외부 개인(`amit-hers`)의 GStreamer 템플릿 fork다. ModalAI 공식 산출물이 아니다.

### 그래도 GitHub 조사가 헛되지 않은 이유 — 아래 §1의 발견 2개

---

## 1. 이번 조사의 최대 성과 2개

### 발견 A. 우리 기체가 돌리는 PX4 소스를 커밋 단위로 특정했다

기체 실측값(`docs/04`)은 **PX4 1.14.0 / git `3e7db37f` / Vendor 2.0.133** 이었다.
`modalai/px4-firmware`의 태그를 조회해 대조한 결과:

```
3e7db37f06278a7756531d926d8bac8a71c6555f    refs/tags/v1.14.0-2.0.133-dev
```

→ **기체 펌웨어 = `modalai/px4-firmware` 태그 `v1.14.0-2.0.133-dev`.** 완전 일치.
Vendor 버전 `2.0.133`이 태그명의 `2.0.133`과 그대로 대응하는 명명 규칙이다.

**의미**: 이제 PX4 동작에 대한 의문은 문서 추정이 아니라 **우리 기체가 실제로 실행하는
바로 그 소스를 읽어서** 답할 수 있다. 재현 명령:

```bash
git clone --depth 1 --filter=blob:none \
  --branch v1.14.0-2.0.133-dev https://github.com/modalai/px4-firmware.git
```

**버전 위치**: 같은 1.14 라인은 현재 `2.0.146`까지 나와 있어 우리는 **13패치 뒤처져 있다.**
ModalAI 주력은 이미 **PX4 1.17.0 (`v1.17.0-8.0.x-dev`)** 으로 이동했다.

### 발견 B. "Collision Prevention은 Offboard에 개입 못 한다"를 소스로 증명했다

`docs/04`에서는 PX4 공식 문서 문장(*"Set to 0 or 3 to enable Collision Prevention in
Position Mode"*)에 근거해 추정했다. 이번엔 **우리 기체 펌웨어 소스에서 직접 확인**했다.

`CollisionPrevention` 클래스를 인스턴스화하는 곳을 전수 검색한 결과:

```
src/modules/flight_mode_manager/tasks/ManualPosition/FlightTaskManualPosition.cpp
src/modules/flight_mode_manager/tasks/ManualPosition/FlightTaskManualPosition.hpp
src/modules/flight_mode_manager/tasks/ManualPosition/CMakeLists.txt
```

**`ManualPosition` 태스크 단 한 곳뿐이다.** flight_mode_manager의 다른 태스크
(`Auto`, `ManualAltitude`, `ManualAcceleration`, `Orbit`, `Descend`, `Failsafe`, `Transition` 등)
어디에도 없다.

여기에 구조적 사실이 하나 더 붙는다: **Offboard 셋포인트는 flight_mode_manager를 거치지 않는다.**
외부에서 온 setpoint는 `trajectory_setpoint`로 `mc_pos_control`에 직접 들어간다.
즉 CP가 끼어들 지점이 애초에 경로상에 없다.

→ **`docs/04`의 결론과 `docs/02` 로드맵의 "단계1-(A) 폐기" 판단은 소스 레벨에서 확정.
파이썬 레벨 회피(B)가 유일한 경로라는 게 이제 추정이 아니라 사실이다.**

참고로 `src/lib/collision_prevention/`에는 구현이 온전히 들어있다
(`CollisionPrevention.cpp/.hpp`, `collisionprevention_params.c`, `CollisionPreventionTest.cpp`).
**코드가 없어서 안 되는 게 아니라, 호출되는 모드가 Position 수동조종뿐이라 안 되는 것.**

---

## 2. `modalai/px4-firmware` — 가장 중요한 저장소 상세

- upstream: **`PX4/PX4-Autopilot`** (fork)
- 크기 539MB / 브랜치 **616개** / 언어 C++
- 우리가 볼 태그: **`v1.14.0-2.0.133-dev`** (= 기체 실물)

### 2.1 최상위 디렉토리

PX4 upstream 표준 구조와 동일하다. VOXL 관련해 우리가 볼 곳만 표시.

| 경로 | 내용 | 우리 관련 |
|---|---|---|
| `boards/` | 보드별 정의·빌드설정·부팅스크립트 | ★★★ `boards/modalai/` |
| `src/` | 전체 소스 (드라이버·모듈·라이브러리) | ★★★ |
| `msg/` | uORB 메시지 정의 (`.msg`) | ★★ 셋포인트/위치 구조 확인용 |
| `platforms/` | posix / qurt / nuttx 플랫폼 추상화 | ★★ VOXL2는 posix + **qurt** |
| `ROMFS/` | 기본 기체 설정·믹서 (NuttX FC용) | ☆ VOXL2는 미사용 |
| `posix-configs/` | posix 타깃 실행 설정 | ★ |
| `Tools/` | 빌드·로그분석·시뮬 스크립트 | ★ |
| `Documentation/`, `test/`, `integrationtests/`, `validation/`, `launch/`, `cmake/` | 문서·테스트·빌드 | ☆ |

### 2.2 `boards/modalai/` — ModalAI 보드 5종

```
boards/modalai/
├── fc-v1/        Flight Core v1 (별도 STM32 FC 보드, VOXL2와 무관)
├── fc-v2/        Flight Core v2 (동일)
├── voxl2/        ★ VOXL2 앱 프로세서(aarch64 Linux) 측
├── voxl2-slpi/   ★ VOXL2 SLPI DSP(Hexagon/QuRT) 측 — 실제 비행제어
├── voxl2-io/     M0065 IO 보드(PWM/SBUS 확장)
└── src/          보드 공통 소스
```

> **VOXL2 PX4는 두 프로세서로 쪼개져 돌아간다.** 이 구조를 모르면 파라미터가
> 어디서 도는지 오해한다.
> - **앱 프로세서(`voxl2`)**: Linux 상의 `voxl-px4` 프로세스. MAVLink·로거·navigator·commander·MPA 브리지
> - **SLPI DSP(`voxl2-slpi`)**: 실시간 제어. **EKF2·mc_pos_control·mc_att/rate_control·센서·land_detector**
> - 둘은 **muORB**(`muorb_apps` ↔ `muorb_slpi`)로 uORB 토픽을 넘긴다

### 2.3 `voxl2/default.px4board` — 앱 프로세서에 빌드되는 것 (실측)

```
CONFIG_PLATFORM_POSIX=y          CONFIG_BOARD_TOOLCHAIN="aarch64-linux-gnu"
CONFIG_BOARD_ROOTFSDIR="/data/px4"
CONFIG_MODULES_MUORB_APPS=y      ← SLPI와 uORB 브리지
CONFIG_MODULES_MAVLINK=y         ← 우리 MAVSDK가 붙는 지점
CONFIG_MODULES_COMMANDER=y  NAVIGATOR=y  DATAMAN=y  LOGGER=y
CONFIG_MODULES_CONTROL_ALLOCATOR=y  LOAD_MON=y  MC_AUTOTUNE_ATTITUDE_CONTROL=y
CONFIG_MODULES_MICRODDS_CLIENT=y ← ROS2 uXRCE-DDS (쓰면 ROS2 연동 가능)
CONFIG_DRIVERS_VOXL2_IO=y  ACTUATORS_VOXL_ESC=y  GPS=y  RC_INPUT=y
CONFIG_DRIVERS_BAROMETER_DPS310=y  INVENSENSE_ICP101XX=y
CONFIG_DRIVERS_OSD_MSP_OSD=y  QSHELL_POSIX=y
CONFIG_PARAM_SERVER=y  ORB_COMMUNICATOR=y
```

**여기에 `MC_POS_CONTROL`·`EKF2`가 없다.** 앱 프로세서엔 위치제어가 없다.

### 2.4 `voxl2-slpi/default.px4board` — SLPI DSP (실제 비행제어, 실측)

```
CONFIG_PLATFORM_QURT=y           CONFIG_BOARD_TOOLCHAIN="qurt"
CONFIG_MODULES_MUORB_SLPI=y      CONFIG_PARAM_CLIENT=y
CONFIG_MODULES_EKF2=y                    ← 상태추정 (EKF2_* 파라미터가 여기서 돈다)
CONFIG_MODULES_MC_POS_CONTROL=y          ← 위치제어 (MPC_* 파라미터)
CONFIG_MODULES_MC_ATT_CONTROL=y  MC_RATE_CONTROL=y  MC_HOVER_THRUST_ESTIMATOR=y
CONFIG_MODULES_FLIGHT_MODE_MANAGER=y     ← ★ CollisionPrevention이 여기 안에 있다
CONFIG_MODULES_COMMANDER=y  SENSORS=y  LAND_DETECTOR=y  MANUAL_CONTROL=y
CONFIG_MODULES_RC_UPDATE=y  CONTROL_ALLOCATOR=y  TEMPERATURE_COMPENSATION=y
CONFIG_MODULES_FW_POS_CONTROL=y  FW_ATT_CONTROL=y  FW_RATE_CONTROL=y  AIRSPEED_SELECTOR=y
CONFIG_MODULES_PARAM_SET_SELECTOR=y  LOAD_MON=y
CONFIG_DRIVERS_DISTANCE_SENSOR_VL53L0X=y  VL53L1X=y  LIGHTWARE_LASER_SERIAL=y
CONFIG_DRIVERS_POWER_MONITOR_VOXLPM=y    ← 배터리 (저전압 failsafe 볼 때 참고)
CONFIG_DRIVERS_BAROMETER_* (ICP101XX/MS5611/BMP280/BMP388/DPS310)
CONFIG_DRIVERS_MAGNETOMETER_* (IST8310/IST8308/QMC5883L/IIS2MDC)
CONFIG_DRIVERS_GPS=y  RC_CRSF_RC=y  OSD_MSP_OSD=y  DIFFERENTIAL_PRESSURE_MS4525DO=y
```

**우리 프로젝트에 직접 쓸 관찰 3개:**
1. `FLIGHT_MODE_MANAGER`가 빌드돼 있으니 CP 코드는 기체에 **존재**한다. 단 §1-B대로 Offboard에선 무효.
2. **거리센서 드라이버 3종(VL53L0X/VL53L1X/Lightware)** 이 컴파일돼 있다 →
   PX4가 직접 읽는 I2C/시리얼 거리센서를 **하드웨어로 추가하면** PX4 레벨 지면·장애물 감지를
   쓸 수 있다는 뜻. 다만 우리 ToF는 이 경로가 아니라 MPA 경로다.
3. `VOXLPM` 전력 모니터 드라이버 존재 → 배터리 저전압 failsafe(미구현 항목)는
   PX4 파라미터(`COM_LOW_BAT_ACT` 등) 쪽 검토가 먼저다. 파이썬으로 새로 짤 필요가 없을 수 있다.

### 2.5 `voxl2/target/` — 기체에서 실제 실행되는 부팅 스크립트

```
voxl-px4                                    실행 래퍼
voxl-px4-start                    (10.7KB)  ★ PX4 부팅 시퀀스 본체
voxl-px4-fake-imu-calibration.config
voxl-px4-hitl                               HITL(시뮬) 래퍼
voxl-px4-hitl-start               (3.6KB)   HITL 부팅
voxl-px4-hitl-set-default-parameters.config HITL 기본 파라미터
```

- `voxl-px4-start`가 **모듈 기동 순서와 기체 형상**을 정하는 곳이다. 기체 동작이
  이상할 때 1차로 볼 파일.
- 이 스크립트는 `GF_*`·`CP_DIST`·`MPC_POS_MODE` 같은 안전 파라미터를 **설정하지 않는다**
  (검색 결과 `param touch SYS_AUTOCONFIG` 한 줄만). → `docs/04`에서 실측한
  "전부 공장 기본값" 상태와 정합한다. **ModalAI가 VOXL2 기본 부팅에서 회피·지오펜스를
  켜주지 않는다는 게 확인됐다.**
- **HITL 자산이 있다**: `voxl-px4-hitl-start`. 기체 없이 시뮬로 미션·회피 로직을
  검증할 길이 열려 있다 → 아래 §5 후속 작업 후보.

### 2.6 `voxl2/src/` — ModalAI가 PX4에 추가한 코드 (우리 코드와 직결)

```
boards/modalai/voxl2/src/
├── board_config.h, boardctl.c, init.c, i2c.cpp, spi.cpp, CMakeLists.txt
├── drivers/
│   └── apps_sbus/                     앱 프로세서 SBUS 수신
├── lib/
│   └── mpa/  (mpa.cpp, mpa.hpp)        ★ PX4 ↔ MPA 파이프 브리지 라이브러리
└── modules/
    ├── vehicle_local_position_bridge/  ★★★ 우리 v13/v14가 읽는 파이프를 만드는 모듈
    ├── mavlink_odometry_bridge/        VIO(OpenVINS) 포즈를 PX4로 주입
    ├── sensor_baro_bridge/             기압계 MPA→PX4
    ├── vehicle_air_data_bridge/        기압/고도 PX4→MPA
    └── crsf_bridge/                    CRSF 조종기
```

#### `vehicle_local_position_bridge` — 우리 도달 판정의 정체 (소스 직독)

v13/v14는 MAVSDK telemetry가 아니라 `px4_vehicle_local_position` MPA 파이프로
도달을 판정한다. 그 파이프를 만드는 코드가 바로 이것이다.

```cpp
// vehicle_local_position_bridge.cpp
WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)   // :75
MPA::Initialize();                                                   // :81
char pipe_name[] = "px4_vehicle_local_position";                     // :86
_pipe_ch = MPA::PipeCreate(pipe_name);                               // :87
MPA::PipeWrite(_pipe_ch, (void*)&pose, sizeof(pose_vel_6dof_t));     // :170
```

모듈 설명 문구: *"publishes vehicle_local_position to MPA pipe as pose_vel_6dof_t"*

**우리 프로젝트에 주는 정보 3개:**
1. 데이터 원본은 uORB **`vehicle_local_position`** — 즉 **EKF2 출력 그 자체**다.
   MAVLink를 타지 않으므로 MAVSDK telemetry보다 **지연이 짧고 손실이 없다.**
   v13이 이 파이프를 택한 건 옳은 설계였다는 근거가 된다.
2. 자료형은 **`pose_vel_6dof_t`** (libmodal-pipe 표준 타입). 위치+속도 6DOF.
   → `voa_pc_out` 파서를 짤 때 같은 라이브러리 규약을 재사용할 수 있다.
3. `nav_and_controllers` 워크큐에서 도는 **SLPI 측 EKF2 갱신에 동기**된 발행이다.
   별도 저속 타이머가 아니다.

### 2.7 ModalAI 전용 드라이버 (`src/drivers/`)

| 드라이버 | 내용 |
|---|---|
| `src/drivers/actuators/voxl_esc` | ModalAI ESC(UART 프로토콜) — 우리 기체 모터 구동 |
| `src/drivers/voxl2_io` | M0065 IO 보드(PWM/SBUS) |

### 2.8 브랜치 체계 읽는 법 (616개)

| 패턴 | 뜻 | 예 |
|---|---|---|
| `voxl-dev`, `voxl-dev-1.1x` | **VOXL 주력 개발 브랜치** (버전별) | `voxl-dev-1.18.0` |
| `voxl-fpv-dev` | FPV 제품 라인 | `voxl-fpv-dev-1.18.0` |
| `voxl2_*`, `voxl2-io*` | VOXL2 보드/IO 작업 | `voxl2_main_dev` |
| `voxl-esc-*` | ESC 드라이버 작업 | `voxl-esc-cleanup-init` |
| `ascend/*` | 사내 "Ascend" 과제 브랜치 (약 다수) | `ascend/geofence/voxl-fpv-dev` |
| `pr-*`, `pr_*` | upstream PR 준비용 | `pr-voxl2-io-bsp` |
| 사람이름-* | 개인 작업 | `eric-fix-m10`, `Paul*` |

**태그 체계**: `v<PX4버전>-<Vendor버전>-dev` → `v1.14.0-2.0.133-dev`.
기체 `voxl-version` 출력의 PX4 버전·Vendor 버전으로 **정확한 태그를 항상 역산할 수 있다.**

> 눈에 띄는 것: **`ascend/geofence/voxl-fpv-dev`, `ascend/geofence_voxl_dev_fix`,
> `ascend/geofence-download/voxl-fpv-dev`** — ModalAI가 지오펜스를 별도로 손댄 브랜치가
> 있다. 우리 v14는 파이썬 레벨로 이미 해결했으므로 급하지 않지만, PX4 레벨 지오펜스를
> 나중에 재검토할 때 참고 대상.

---

## 3. ModalAI 자체 저장소 7개 (fork 아님)

| 저장소 | 기본브랜치 | 크기 | 내용 | 우리 관련도 |
|---|---|---|---|---|
| **`px4-flight-review`** | `voxl-dev` | 27MB | PX4 **비행로그(ulog) 웹 분석 도구**의 VOXL 판. PX4 `flight_review`(Bokeh 기반) 계열 | **★★★** 비행 후 분석에 바로 유용 |
| `qrb5165-kernel` | `master` | **1.16GB** | VOXL2 SoC(QRB5165) **리눅스 커널 4.19.125** 소스. 브랜치 `chipcode-14.1a`/`qrb5165-ubun1.0-14.1a`가 기체 system-image `14.1a`와 대응 | ★ 커널 손댈 때만 |
| `nanotrack` | `main` | 3.6MB | **단일객체 시각추적**(NanoTrack) VOXL 이식. `HonglinChu/NanoTrack` 재구성 | ★★ 대상 추적 기능 필요 시 |
| `ap_host` | `main` | 6KB | SLPI DSP에서 **ArduPilot** 구동용 임시 리눅스 앱. `libslpi-link-api` 포함 | ☆ PX4 쓰므로 무관 |
| `ax88772b` | `main` | 46KB | ASIX USB 이더넷 커널 드라이버 (벤더 소스) | ☆ USB-이더넷 쓸 때만 |
| `setup-sandbox` | `main` | 4KB | 내부 셋업 스크립트(`raven-sdk1.3` 디렉토리 하나) | ☆ |
| `ci-sandbox` | `main` | 90KB | 내부 CI 실험(Dockerfile) | ☆ |

### `nanotrack` 구조 (실측) — 나중에 대상추적 붙일 때 참고

```
nanotrack.cpp / nanotrack.hpp   추적기 본체
include/  lib/  model/          헤더 · 라이브러리 · 학습된 모델
build.sh  clean.sh  make_package.sh  install_build_deps.sh  CMakeLists.txt  pkg/
```
의존성(README): `voxl-opencv`, `voxl-eigen3`, **`libmodal-pipe`**, `libmodal-json`,
`voxl-mpa-tools`, `voxl-mavlink`, `librc-math`, `libmodal-journal`

> **여기서도 `libmodal-pipe`가 나온다.** VOXL에서 MPA 데이터를 다루는 모든 코드가
> 이 라이브러리를 쓴다. 우리 `voa_pc_out` 파서의 참조 구현으로 삼기 좋다.
> (GitLab에 있음 → `docs/07`)
> 참고: SDK에는 이걸 서비스로 감싼 `voxl-nano-tracker`도 따로 있다.

### `px4-flight-review` — 우선순위를 올릴 만한 발견

기체는 `voxl-px4` 로거로 **ulog**를 남긴다. 이 저장소는 그 ulog를 브라우저에서
그래프로 뜯어보는 도구다. `voxl-dev` 브랜치가 VOXL용으로 조정된 판.

우리 상황과 맞는 부분: **v14 지오펜스 임계값(`--max-horizontal-dev 2.5`,
`--geofence-violation-sec 1.0`)이 실제 VIO 노이즈에 적절한지가 미검증 상태**다.
ulog에서 `vehicle_local_position` vs `trajectory_setpoint` 추종 오차를
그려보면 **실비행 없이도 과거 로그로 임계값을 검토할 수 있다.**
→ §5 후속 작업에 반영.

---

## 4. fork 저장소 37개 전수 목록

ModalAI가 upstream을 fork해 둔 것들. **대부분은 "우리가 쓰는 부품의 원본 소스"**라는
의미 이상은 없다. 관련도만 빠르게 판단할 수 있게 분류했다.

### 4.1 비행 스택 (관련도 높음)

| 저장소 | upstream | 크기 | 최근 push | 우리 관련 |
|---|---|---|---|---|
| **`px4-firmware`** | `PX4/PX4-Autopilot` | 539MB | 2026-07-24 | **★★★ 기체 펌웨어 본체 (§2)** |
| **`open_vins`** | `rpng/open_vins` | 197MB | 2026-07-20 | **★★★ 우리 VIO 원본 (§4.2)** |
| `mavlink` | `mavlink/mavlink` | 13.5MB | 2026-05-15 | ★★ MAVLink 메시지 정의. `eric-add-modalai-payload-types` 등 ModalAI 커스텀 메시지 브랜치 존재 |
| `PX4-GPSDrivers` | `PX4/PX4-GPSDrivers` | 400KB | 2026-07-07 | ★ GPS 드라이버. **우리 기체는 GPS 미사용**. `voxl-dev`/`voxl-gps-dev` 브랜치 있음 |
| `ardupilot` | `ArduPilot/ardupilot` | 565MB | 2026-04-07 | ☆ PX4 쓰므로 무관 |
| `tridge-ardupilot` | `tridge/ardupilot` | 392MB | 2024-07-15 | ☆ |
| `ardupilot_wiki` | `ArduPilot/ardupilot_wiki` | 644MB | 2024-04-18 | ☆ |
| `ecl` | `PX4/ecl` | 26.7MB | 2022-06-09 | ☆ 구 EKF 라이브러리(현재 PX4에 흡수) |
| `NuttX` | `PX4/NuttX` | 142MB | 2020-01-03 | ☆ VOXL2는 NuttX 미사용 |
| `px4-bootloader` | PX4 Bootloader | 537KB | 2022-08-29 | ☆ STM32 FC용 |
| `PX4-user_guide` | `PX4/PX4-user_guide` | 423MB | 2022-10-14 | ★ **PX4 공식 문서 소스**. 단 2022년 fork로 오래됨 → 최신은 docs.px4.io 직접 참조 |
| `PX4-SITL_gazebo-classic` | PX4 | 96MB | 2024-08-08 | ★ 시뮬레이션(HITL/SITL) 볼 때 |
| `qgroundcontrol` | `mavlink/qgroundcontrol` | 369MB | 2019-12-04 | ☆ 매우 오래됨 |
| `c_library_v2` | `mavlink/c_library_v2` | 11MB | 2021-07-10 | ☆ MAVLink C 헤더 |
| `dspal` | DSP Abstraction Layer | 362KB | 2021-06-04 | ☆ 구 Snapdragon Flight 시절 |

### 4.2 `open_vins` — 우리 VIO의 원본

- upstream **`rpng/open_vins`** (델라웨어대 RPNG 연구실, MSCKF 기반 VI 상태추정기)
- 기체의 `voxl-open-vins-server`가 이걸 감싼 것. **SDK 래퍼는 GitLab**에 있다.
- 우리가 볼 브랜치: **`voxl-sdk_v1.x`** (voxl-suite 1.6.3 계열과 대응)

구조(실측):
```
ov_core/   특징점 추적·카메라모델·타입 (공통 CV 코드)
ov_init/   초기화 (정지/동적 초기화)
ov_msckf/  ★ MSCKF 필터 본체 (상태·전파·업데이트)
ov_eval/   정확도 평가·플롯 도구
CMakeLists.txt  ReadMe.md  LICENSE
```
ModalAI 관련 브랜치: `voxl-sdk_v1.x`, `mai-dev`, `mai-master`,
`openvins_prod_v1.1~v1.3`, `quality`/`quality8`, `in-air-resets`, `bmi270`,
`gpu_integ`, `stereo-*`, `vft-integ`, `zbft_*`

> `in-air-resets`, `quality*` 브랜치가 눈에 띈다. **VIO 품질 판정과 공중 리셋**은
> 우리 프리플라이트 VIO 검증(`check_vio_quality_preflight`)·비행 중 VIO 신뢰도
> 감시와 직결되는 주제다. `voa_pc_out`의 모션보상이 VIO 실패 시 항등변환으로
> 폴백한다는 점(`docs/04`)과 맞물려, **VIO 상태 감시 설계 시 참고 가치 있음.**

### 4.3 무선 링크 / 조종기 (관련도 낮음 — 우리는 WiFi + MAVSDK)

| 저장소 | upstream | 크기 | 내용 |
|---|---|---|---|
| `betaflight` | betaflight | 417MB | FPV 비행 컨트롤러 펌웨어 |
| `ExpressLRS` | ExpressLRS | 327MB | 장거리 RC 링크 (STM32/ESP32) |
| `ExpressLRS_3.2.1` | ExpressLRS | 323MB | 3.2.1 고정판 |
| `ExpressLRS-Configurator` | ExpressLRS | 306MB | ELRS 설정 GUI |
| `edgetx` | EdgeTX | 342MB | RC 조종기 펌웨어 |
| `wifibroadcast` | svpcom | 1.4MB | 원시 WiFi UDP 영상전송 |
| `rtl8812au-wfb` | — | 66MB | wifibroadcast용 RTL8812AU 드라이버 |
| `rtl8188eus` | aircrack-ng 계열 | 5.5MB | 모니터모드 WiFi 드라이버 |
| `8821cu-20210916` | morrownr | 14MB | RTL8811CU/8821CU WiFi 드라이버 |
| `esptool` | espressif | 14MB | ESP 플래싱 도구 |

### 4.4 인식 / 영상 / 매핑 (선택적 관련)

| 저장소 | upstream | 크기 | 우리 관련 |
|---|---|---|---|
| `ncnn` | `Tencent/ncnn` | 32MB | ★★ 모바일 NN 추론. `nanotrack`·온보드 추론의 기반 |
| `rtabmap_ros` | `introlab/rtabmap_ros` | 55MB | ★★ **RGB-D SLAM(ROS)**. 매핑 대안 검토 시. 단 ROS 필요 |
| `octomap_rviz_plugins` | OctoMap | 214KB | ★ 점유맵 RViz 시각화 |
| `libseek-thermal` | — | 1.7MB | ☆ Seek 열화상 카메라 |
| `libuvc` | `libuvc/libuvc` | 481KB | ☆ USB 비디오 |
| `gscam` | ros-drivers | 134KB | ☆ GStreamer→ROS 카메라 |
| `Voxl-Plugin` | `amit-hers/Voxl-Plugin` | 130KB | ☆ **외부 개인의** MPA GStreamer 플러그인 템플릿 (`gst-app`, `gst-plugin`, meson 빌드) |
| `ruy` | `google/ruy` | 1.7MB | ☆ tflite 행렬연산 백엔드 |
| `geographiclib` | geographiclib | 21MB | ☆ 측지 계산 |

### 4.5 플랫폼 / 저수준

| 저장소 | upstream | 크기 | 내용 |
|---|---|---|---|
| `sample-apps-for-robotics-platforms` | **`quic/sample-apps...`** (Qualcomm) | 39MB | ★ Qualcomm 로보틱스 플랫폼 예제. GStreamer·OpenGLES·OpenMAX·커널 샘플. `decode-from-pipe` 브랜치 존재 |
| `qdl` | `linux-msm/qdl` | 68KB | ☆ Qualcomm 플래싱 |
| `tokio-tun` | yaa110 | 41KB | ☆ Rust TUN/TAP |

---

## 5. 이 조사로 갱신되는 우리 프로젝트 판단

### 확정된 것 (추정 → 사실)

| 항목 | 이전 상태 | 지금 |
|---|---|---|
| CP가 Offboard에 개입 못 함 | PX4 문서 문장 근거 **추정** | **기체 실제 펌웨어 소스로 확증** (§1-B). `docs/02` 단계1-(A) 폐기 확정 |
| 기체 펌웨어 정체 | 버전 문자열만 알았음 | **태그 `v1.14.0-2.0.133-dev` 특정, 소스 열람 가능** (§1-A) |
| `px4_vehicle_local_position` 파이프의 정체 | "PX4 로컬 위치" 정도 | **EKF2 `vehicle_local_position` 직결, `pose_vel_6dof_t`, MAVLink 미경유** (§2.6) |
| ModalAI 기본 부팅이 회피/펜스를 켜주는가 | 실측으로 꺼져 있음 확인 | **부팅 스크립트에 설정 자체가 없음 — 의도된 기본값** (§2.5) |

### 새로 생긴 후속 작업 후보

1. **`px4-flight-review`로 과거 ulog 분석** — v14 지오펜스 임계값을 **실비행 전에**
   과거 로그의 추종 오차 분포로 검토. 현재 "실비행 미검증"인 항목을 상당부분 앞당길 수 있다. (★ 추천)
2. **HITL 시뮬 검토** — `voxl-px4-hitl-start`가 기체에 존재. 회피 로직을 기체 없이 시험할 경로.
3. **배터리 저전압 failsafe** — 파이썬으로 짜기 전에 PX4 `VOXLPM` + `COM_LOW_BAT_ACT`
   파라미터로 되는지 먼저 확인. 파이썬 구현이 불필요할 수 있다.
4. **`open_vins` `quality`/`in-air-resets` 브랜치 검토** — VIO 신뢰도 감시 설계 참고.
5. **PX4 업그레이드는 당장 하지 말 것** — 1.14 라인은 `2.0.146`까지, 주력은 1.17로 이동.
   업그레이드는 v14 검증과 회피 구현이 끝난 뒤에 별건으로. 지금 건드리면 변수만 늘어난다.

### 하지 말 것 / 오해 주의

- **GitHub에서 `voxl-mapper`·`libmodal-pipe`를 찾지 말 것.** 없다. GitLab이다 (`docs/07`).
- `PX4-user_guide` fork(2022)를 최신 문서로 읽지 말 것 → `docs.px4.io` 직접 참조.
- `qgroundcontrol` fork는 2019년판이다.
- fork 저장소의 `master`는 대개 upstream 그대로다. **ModalAI 수정은 `voxl-*`/`mai-*`
  브랜치와 태그에 있다.** 브랜치를 안 지정하면 upstream을 읽게 된다.

---

## 6. 재현용 명령 모음

```bash
# 조직 저장소 전수 열거 (API 한도 60/hr 주의)
curl -s "https://api.github.com/orgs/modalai/repos?per_page=100&sort=updated"

# 브랜치·태그 열거 — API 한도 안 씀 (권장)
git ls-remote --heads https://github.com/modalai/px4-firmware.git
git ls-remote --tags  https://github.com/modalai/px4-firmware.git

# 기체 버전(PX4 1.14.0 / Vendor 2.0.133)에 대응하는 정확한 소스
git clone --depth 1 --filter=blob:none \
  --branch v1.14.0-2.0.133-dev https://github.com/modalai/px4-firmware.git

# 기체에서 버전 확인 → 위 태그 역산
#   voxl-version ;  ver all      (voxl-px4 셸)
```

> **팁**: `--depth 1 --filter=blob:none`이면 539MB 저장소도 100MB 남짓에 트리 전체를
> 볼 수 있다. 파일 내용은 열 때만 받아온다. `git ls-remote`는 클론 없이 브랜치·태그만
> 조회하며 GitHub API 한도를 소모하지 않는다.

---

## 7. 조사 범위와 한계 (정직한 기록)

- 저장소 **44개 전수**를 API로 열거하고, 이름·설명·언어·크기·fork여부·기본브랜치·최근 push를
  모두 확보했다. 위 표는 44개를 빠짐없이 담고 있다.
- **파일 단위로 직독한 것**은 우리 프로젝트 관련도가 있는 9개 저장소다:
  `px4-firmware`(기체 태그), `open_vins`, `nanotrack`, `ap_host`, `Voxl-Plugin`,
  `PX4-GPSDrivers`, `px4-flight-review`, `setup-sandbox`, `ax88772b`.
- **직독하지 않은 것**: 나머지 fork들(betaflight 417MB, ardupilot 565MB, edgetx 342MB,
  ExpressLRS 327MB, qrb5165-kernel 1.16GB 등). 이들은 **upstream 오픈소스와 동일한
  구조이고 우리 프로젝트와 무관**하다고 판단해 메타데이터·설명 수준으로만 정리했다.
  필요해지면 그때 개별 조사하는 게 맞다.
- upstream(`parent`)은 `px4-firmware`·`open_vins`·`px4-flight-review`·`nanotrack`·
  `sample-apps-for-robotics-platforms`·`Voxl-Plugin`·`mavlink`·`ardupilot` **8개는 API로 실확인**했다.
  나머지는 저장소명·설명에 근거한 추정이며, 표에서 upstream 칸이 확실치 않은 항목은 그렇게 읽어야 한다.
- `px4-firmware` 브랜치는 616개라 전체 목록은 담지 않고 **명명 규칙 + `voxl` 포함 70개**로 정리했다(§2.8).
