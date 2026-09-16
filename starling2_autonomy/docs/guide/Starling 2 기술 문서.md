---
title: "Starling 2 기술 문서"
subtitle: "ModalAI Starling 2 / VOXL 2 시스템 구조 및 소프트웨어 스택"
date: "2026-09-02"
lang: ko
---

> 이 문서는 ModalAI 공식 문서(docs.modalai.com), voxl-public GitLab, ModalAI 포럼, PX4 공식 문서를 근거로 작성했습니다. "우리 드론"에 해당하는 값은 `[확인 필요]`로 표시했으니 `voxl-version`, `voxl-inspect-sku`, `voxl-inspect-services` 결과로 채워 넣으세요. 공식 자료에서 확인되지 않은 항목은 `(미확인)`으로 표시했습니다.

# 시스템 개요

## 하드웨어 구성

Starling 2는 ModalAI의 **실내 비행용 플래그십 개발 드론**입니다(1세대 Starling은 EOL). 공식 데이터시트 사양:

| 항목 | 사양 |
|---|---|
| 오토파일럿/컴퓨트 | VOXL 2 (M0054) + 프런트엔드 보드 M0173 |
| 카메라 구성 | **C26**: IMX412(hires) + PMD ToF + AR0144 ×2 (tracking 전방/하방) / **C27**: IMX412 + ToF + AR0144 ×3 |
| 이륙 중량 | 285 g (배터리 제외 182 g) |
| 크기 | 대각 230 mm, 3 mm 카본 프레임 |
| 모터 / 프롭 | 1504 3000 KV / 120 mm 접이식 |
| ESC | ModalAI 4-in-1 Mini ESC (M0129, 전원 모듈 내장) |
| 비행 시간 | > 35분 |
| 배터리 | Sony VTC6 3000 mAh **2S Li-Ion 18650** (XT30) 또는 동급 2S 18650 팩 |
| GPS/자력계 | u-blox M10 (J19, 야외용) |
| RC 수신기 | 915 MHz ELRS (M0184) 또는 2.4 GHz Ghost Atto |
| WiFi | Alfa Networks AWUS036EACS (USB) |

참고로 Starling 2 **Max**는 C28/C29 구성(IMX412 ×2, AR0144 ×2, ToF 선택), 566 g, 4S(2S ×2 직렬), 180 mm 프롭으로 완전히 다른 체급입니다.

## 소프트웨어 스택

```
┌──────────────────────────────────────────────────────────────┐
│ 사용자 코드: MAVSDK Python / QGroundControl / voxl-portal(웹) │
├──────────────────────────────────────────────────────────────┤
│ voxl-mavlink-server (UDP 14550 GCS) ── voxl-vision-hub (14551)│
│      │ MAVLink                              │ ODOMETRY/VOA    │
│ voxl-px4 (PX4 1.14 ModalAI fork, apps proc + SLPI DSP 센서)   │
├──────────────────────────────────────────────────────────────┤
│ MPA 서비스: voxl-camera-server · voxl-imu-server ·             │
│   voxl-open-vins-server(또는 voxl-qvio-server) · voxl-tag-    │
│   detector · voxl-cpu-monitor · voxl-portal · voxl-logger ·   │
│   voxl-streamer · voxl-tflite-server · voxl-mapper(옵션)      │
├──────────────────────────────────────────────────────────────┤
│ libmodal_pipe (/run/mpa/*)  ·  systemd                        │
├──────────────────────────────────────────────────────────────┤
│ Ubuntu 18.04 / Linux 4.19 (aarch64)  ·  Python 3.6.9          │
└──────────────────────────────────────────────────────────────┘
```

- **VOXL SDK** = `voxl-suite` 메타패키지(모든 voxl-* 패키지) + 시스템 이미지 + 설치 스크립트. 개별 `apt upgrade`가 아니라 SDK 전체를 플래시하는 방식으로 관리됩니다.
- 버전 흐름: SDK 1.0(2023) → 1.4(2025-01) → 1.5(2025-06) → **1.6.x(현행 안정판, 1.6.6 = 2026-07-10)** → 1.7 베타. Starling 2는 **SDK 1.6.4 이상** 권장. 시스템 이미지: 1.8.06(SDK 1.6.0~1.6.3) → 1.8.08(SDK 1.6.4~1.6.6).
- PX4: ModalAI 포크(`github.com/modalai/px4-firmware`, 브랜치 `voxl-dev`), SDK 1.6.x 기준 **PX4 1.14.0-8.x**.
- VIO: SDK 1.4.0부터 `voxl-open-vins-server` 제공, **SDK 1.6.2 이후 기본 VINS 솔루션이 Open-VINS**(QVIO는 여전히 선택 가능).

## 우리 드론 SKU 분석

SKU 형식: `MRB-D0014-4-V1-CXX-TX-M22-X0`

| 필드 | 의미 | 우리 값 |
|---|---|---|
| `MRB-D0014` | 제품군 = Starling 2 | D0014 |
| `4` | 컴퓨트 보드 = VOXL 2 | 4 |
| `V1` | 하드웨어 리비전 | `[확인 필요]` |
| `C26` / `C27` | 카메라 구성 (AR0144 2개 / 3개) | `[확인 필요]` |
| `T7` / `T9` | RC: T7 = ELRS 915 MHz, T9 = Ghost 2.4 GHz | `[확인 필요]` |
| `M22` | 모뎀 = WiFi 동글 | M22 |

확인 명령:

```bash
cat /data/modalai/sku.txt      # SKU 원문
voxl-inspect-sku               # 파싱된 SKU
voxl-version                   # 시스템 이미지, 커널, voxl-suite 버전, HW 플랫폼
```

SKU를 바꾸거나 잘못 잡혔을 때는 `voxl-configure-sku --wizard` → `voxl-configure-mpa` (extrinsics/카메라/IMU/PX4 파라미터를 SKU에 맞게 일괄 재설정) → 전원 재인가.

우리 드론의 중요 결론: **스테레오 카메라가 없다**(개별 AR0144 트래킹 카메라 + ToF 구성). 따라서 `voxl-dfs-server`(스테레오 깊이)는 사용할 수 없고, 장애물 감지는 **ToF 하나**에 의존합니다.

# 핵심 시스템

## VOXL2 (컴퓨트 보드)

| 항목 | 사양 |
|---|---|
| SoC | Qualcomm QRB5165, 8코어(최대 3.091 GHz), Adreno 650 GPU, NPU 15 TOPS |
| 메모리/저장 | 8 GB LPDDR5 / 128 GB 플래시 |
| 크기/무게 | 70 × 36 mm / 16 g |
| 전원 | 5 V ±5 %, 서지 6 A 필요 (J4) |
| 카메라 입력 | 4-lane MIPI CSI ×6 (J6/J7/J8 그룹당 2포트) |
| 주요 커넥터 | J2 팬(5 V + PWM) · J4 전원/배터리 모니터 I2C · J9 USB-C(ADB) · J18 ESC UART · J19 GNSS UART + 자력계 I2C + RC UART |
| 온보드 IMU | ICM-42688P ×2 — IMU0(SLPI DSP → PX4, 파이프 `imu_px4`), IMU1(앱 프로세서 `/dev/spidev3.0`, 파이프 `imu_apps`) |
| 온보드 기압계 | BMP388 + ICP-10100 (SLPI, PX4용) |
| 자력계 | 온보드 없음(J19 외장 — Starling 2는 GPS 모듈에 포함) |

- PX4의 센서 처리는 **SLPI(센서 DSP)** 에서, PX4 본체와 나머지 서비스는 **앱 프로세서(Linux)** 에서 돕니다.
- 발열: 약 **90 °C부터 클럭 스로틀링**. ModalAI는 "공기 흐름이 가장 큰 변수"라고 명시하며, 비행 중에는 프롭 바람으로 충분하지만 벤치에서는 J2 팬 또는 외부 팬이 필요합니다. `voxl-fan on|slow|off`, 기본은 `voxl-cpu-monitor`가 자동 제어.

## PX4 (Flight Controller)

- 패키지/서비스 `voxl-px4`: PX4를 Linux 서비스로 실행. 설정 파일 **`/etc/modalai/voxl-px4.conf`**:
  - `GPS=[NONE | AUTODETECT]`
  - `RC=[SPEKTRUM | CRSF_MAV | CRSF_RAW | GHST | M0065_SBUS | EXTERNAL | FAKE_RC_INPUT]` — ELRS는 `CRSF_RAW`, Ghost는 `GHST`
  - `ESC=[VOXL_ESC | VOXL2_IO_PWM_ESC]` — Starling 2는 `VOXL_ESC`
  - logger 옵션(`-e` 추가 시 부팅부터 로깅)
- 파라미터: 기본값과의 **차이만** `/data/px4/param/parameters`에 저장. 삭제하면 `/etc/modalai/voxl-px4-set-default-parameters.config`로부터 재생성(=공장 초기화). 캘리브레이션은 `/data/px4/param/parameters_{gyro,acc,mag,level}.cal`.
- 로그: `/data/px4/log/sessNNN/logNNN.ulg` (시동~해제 구간).
- 도구: `px4-param show|set`, `px4-listener <uORB 토픽>`, `px4-commander check`, `px4-logger stop`, `px4-shell`(미확인 이름).
- SKU별 프리셋 적용: `voxl-configure-px4-params` (`voxl-configure-mpa`가 호출), 위저드 `voxl-configure-px4-params -w`.
- QGC 연결: `voxl-mavlink-server` UDP 14550. SoftAP에서 PC는 192.168.8.10을 받으므로 기본 설정으로 바로 연결됨. Station 모드에서는 `/etc/modalai/voxl-mavlink-server.conf`의 `primary_static_gcs_ip`를 PC IP로 바꾸고 `systemctl restart voxl-mavlink-server`. **USB-C로는 QGC 연결이 불가**합니다.

## voxl-vision-hub

"VIO를 MAVLink odometry로 오토파일럿에 전달하고, 장애물 센서를 융합해 VOA를 수행하며, 온보드 자율 비행 동작을 제공"하는 핵심 서비스입니다.

- VIO 바디 프레임 → **중력 정렬 로컬 프레임**(원점 = 이륙 지점의 무게중심, +Z 중력 방향)으로 변환해 PX4 `ODOMETRY`로 송신.
- 설정 **`/etc/modalai/voxl-vision-hub.conf`** 주요 필드(기본값):

| 필드 | 기본값 | 의미 |
|---|---|---|
| `en_vio` | true | VIO 전달 |
| `vio_pipe` / `secondary_vio_pipe` | `"qvio"` / `"ov"` | 1차/2차 VIO 입력 파이프. Open-VINS 사용 시 `"ov"`가 실제 소스 `[우리 값 확인 필요]` |
| `vio_warmup_s` | 3.0 | VIO 워밍업 시간 |
| `offboard_mode` | `figure_eight` | `off / figure_eight / follow_tag / trajectory / vfc / backtrack / wps` |
| `figure_eight_move_home` | true | 8자 중심을 offboard 진입 지점으로 |
| `en_voa` | true | VOA 활성화 |
| `voa_upper_bound_m` / `voa_lower_bound_m` | −0.15 / 0.15 | 장애물 판정 높이 범위 |
| `voa_memory_s` | 1.0 | 장애물 기억 시간 |
| `voa_pie_slices` | 36 | OBSTACLE_DISTANCE 분할 수(10°) |
| `voa_inputs[]` | ≤ 6개 | `type(point_cloud/tof/rangefinder)`, `input_pipe`, `frame`, `max/min_depth`, `cell_size`, `threshold`, `x/y_fov_deg`, `conf_cutoff` |
| `robot_radius` | 0.3 | trajectory 모드 충돌 반경 |
| `en_tag_fixed_frame` | false | AprilTag 기준 고정 프레임 |
| `en_localhost_mavlink_udp` | **false** | MAVSDK/MAVROS용 localhost UDP |
| `localhost_udp_port_number` | 14551 | 위 포트 |

- 퍼블리시 파이프: `vvhub_body_wrt_local`, `vvhub_body_wrt_fixed`, `vvhub_aligned_vio`, `voa_pc_out`, `vfc`. 구독: VIO 파이프, `imu_apps`, `tag_detections`, `plan_msgs`, mavlink 파이프.
- 도구: `voxl-configure-vision-hub wizard|enable|disable`, 디버그 `voxl-vision-hub -c -o -b -d --debug_offboard`(서비스 중지 후 포그라운드 실행).
- ModalAI 답변: vision-hub는 기본적으로 Seeker/Starling/Starling 2의 **PMD ToF**와 VL53L1CX 미니 거리계를 VOA 입력으로 소비하도록 설정되어 있음.

## MAVSDK / MAVLink

- **voxl-mavlink-server**: 네트워크 ↔ MPA ↔ 비행 컨트롤러 간 MAVLink 라우팅. 설정 `/etc/modalai/voxl-mavlink-server.conf`:
  - `primary_static_gcs_ip "192.168.8.10"`, `secondary_static_gcs_ip "192.168.8.11"` (포트 14550, 동시 GCS 최대 16개)
  - 내부 포트: `onboard_port_to_autopilot 14556`, `onboard_port_from_autopilot 14557`, `gcs_port_to_autopilot 14558`, `gcs_port_from_autopilot 14559`
  - `udp_mtu 512`, `gcs_timeout_s 4.5`
- 퍼블리시 파이프: `mavlink_onboard`, `mavlink_ap_heartbeat`, `mavlink_sys_status`, `mavlink_gps_raw_int`, `mavlink_attitude`, `mavlink_local_position_ned`, `imu_mavlink`, `px4_baro`, `rc_channels`, `rc_active`, `mavlink_to_gcs`, `mavlink_from_gcs`, `gcs_ip_list`, `mission`.
- **MAVSDK 온보드 연결 경로**: vision-hub의 `en_localhost_mavlink_udp: true` → `udp://:14551`(localhost 전용). ModalAI 공식 방법은 Docker(`voxl-configure-docker-support` → `gcr.io/modalai-public/voxl-mavsdk-python:v1.1`, `--net=host`)이며, 우리는 네이티브 Python 3.6.9 + `mavsdk==0.12.0`(PyPI requires_python ≥ 3.6)으로 운용합니다. aarch64 휠에는 `mavsdk_server` 바이너리가 없어 서버를 별도 실행해야 합니다(메인 가이드 참고).

# 위치 추정

## Open-VINS (VIO)

- `voxl-open-vins-server`: OpenVINS 기반 **MSCKF** 추정기. 카메라 최대 3대 + IMU 융합, 동기/비동기 카메라 혼용 가능, 롤링/글로벌 셔터 혼용 가능. SDK ≥ 1.4.0.
- 설정 파일:
  - `/etc/modalai/voxl-open-vins-server.conf` — `en_auto_reset true`, `imu_body_frame_mode true`, `auto_reset_max_velocity 20.0`, `auto_reset_min_features 1`, `takeoff_alt_threshold 0.5`, `quality_high_thresh 35`, `quality_low_thresh_good 14`
  - `/etc/modalai/vio_cams.conf` — 카메라별 `enable, name, pipe_for_tracking, is_occluded_on_ground, imu, cal_file`
  - `/etc/modalai/extrinsics.conf` — IMU↔카메라 변환
- 입력: `imu_apps`(또는 `imu_apps_body`), `tracking_front`, `tracking_down`. 출력 파이프: **`ov`**(포즈/속도), `ov_extended`(공분산 포함), `ov_status`.
- 상태 머신: INITIAL / GOOD / BAD (히스테리시스). **움직이는 중에도 수십 ms 내 초기화**(QVIO는 정지 상태 필요).
- 전환 절차: `voxl-configure-qvio disable` → `voxl-configure-open-vins <camera_configuration>` → `systemctl restart voxl-open-vins-server`. **VIO 소스는 한 번에 하나만** 활성화(둘 다 켜면 실패 사례).
- 점검: `voxl-inspect-vins [-v -b -z -n]`(원시 출력), `voxl-inspect-vio`(기본 파이프 `vvhub_aligned_vio`; dt | T_imu_wrt_vio | RPY | features | quality % | state | error_codes), 리셋 `voxl-reset-vins`.
- 에러 코드: `0x1 COV_ERROR`, `0x10 NO_FEATURES`, `0x800 BAD_CAM_CAL` 등. Open-VINS는 QVIO보다 **캘리브레이션 품질에 민감**합니다.

## EKF2 개요

PX4의 EKF2가 VIO(외부 비전, EV), 기압계, 거리계, IMU를 융합해 `vehicle_local_position`을 만듭니다. VIO 전용 실내 비행의 핵심 파라미터:

| 파라미터 | 값 | 의미 |
|---|---|---|
| `EKF2_EV_CTRL` | 15 | EV 융합 비트마스크: 수평 위치 + 수직 위치 + 속도 + yaw 모두 사용 (1.14에서 `EKF2_AID_MASK` 대체) |
| `EKF2_HGT_REF` | 3 | 고도 기준 = Vision |
| `EKF2_GPS_CTRL` | 0 | GPS 융합 끔 (`EKF2_HGT_REF` 설정 후에 적용해야 함) |
| `EKF2_BARO_CTRL` | 1 | 기압계 융합 |
| `EKF2_RNG_CTRL` / `EKF2_RNG_DELAY` | 1 / 50 ms | 하방 거리(ToF) 융합 |
| `EKF2_OF_CTRL` | 0 | 옵티컬 플로우 없음 |
| `EKF2_MAG_TYPE` | 5 | 자력계 사용 안 함 |
| `EKF2_EV_QMIN` | 16 | VIO 품질 하한 |
| `EKF2_EV_NOISE_MD` | 0 | 노이즈 파라미터 사용 |
| `EKF2_EVP_NOISE` / `EKF2_EVV_NOISE` / `EKF2_EVA_NOISE` | 0.1 / 0.1 / 0.1 | 위치/속도/자세 노이즈 |
| `EKF2_EVP_GATE` / `EKF2_EVV_GATE` | 5.0 / 3.0 | 이노베이션 게이트 |
| `EKF2_EV_DELAY` | 0.00 | VOXL이 타임싱크를 처리하므로 0 |

증상 판별: `px4-listener vehicle_local_position`에서 **Z만 변하고 X/Y가 0 고정**이면 VIO 자체가 아니라 EKF2가 EV를 융합하지 않는 것(파라미터 문제).

알려진 이슈: VOXL 2는 odometry를 `MAV_FRAME_LOCAL_FRD`로 보내는데, PX4 1.14에서 `SYS_HAS_MAG 0`이면 yaw 정렬이 NED EV에서만 이뤄져 **"heading estimate invalid"** 로 시동이 거부될 수 있음(PX4 이슈 #27031) → SDK 1.6 이상 사용 및 프리셋 적용으로 대응.

## PX4 파라미터 프리셋

프리셋 저장소: https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-px4-params (`params/v1.14/`). 디렉터리: `platforms`, `EKF2_helpers`, `radio_helpers`, `voa_helpers`, `battery_helpers`, `fpv_helpers`, `voxl2_io_helpers`, `other_helpers`, `ci_helpers`. `voxl-configure-px4-params -w` 위저드로 적용.

**플랫폼 프리셋 `platforms/D0014_Starling_2.params`** 주요 값: `SYS_AUTOSTART 4001`, `MPC_THR_HOVER 0.34`, `MPC_THR_MAX 0.60`, `MPC_THR_MIN 0.07`, `VOXL_ESC_RPM_MIN/MAX 2000/15000`, `VOXL_ESC_BAUD 2000000`, `EKF2_IMU_POS_X/Y/Z 0.0043/0.0073/−0.016`, `EKF2_EV_QMIN 16`, `EKF2_HGT_REF 3`, `MC_ROLLRATE_MAX/MC_PITCHRATE_MAX 130`, `MC_YAWRATE_MAX 150`(VIO 카메라 보호), `MC_AIRMODE 0`, `IMU_GYRO_DNF_EN 1`, `MPC_TKO_RAMP_T 1.0`, `COM_SPOOLUP_TIME 2.0`, `MPC_TKO_SPEED 1.5`, `COM_DISARM_LAND 0.1`, `COM_DISARM_PRFLT 20`.

### indoor_vio_missing_gps

- `EKF2_helpers/indoor_vio_missing_gps.params`: 위 "EKF2 개요" 표의 EKF2 블록과 동일하되, **GPS/자력계 하드웨어가 없는 기체용**으로 `SYS_HAS_MAG 0`, `SYS_HAS_GPS 0`, `COM_ARM_WO_GPS 1`을 설정합니다. 실내 전용 Starling 2에 적용하는 프리셋이며, `/etc/modalai/voxl-px4.conf`에 `GPS=NONE`을 함께 둡니다.
- 반대로 `indoor_vio.params`는 "VIO만 쓰지만 GPS/자력계가 물리적으로 존재하는" 기체용(`SYS_HAS_MAG 1`, `SYS_HAS_GPS 1`, `COM_ARM_WO_GPS 1`).
- 적용 후 재부팅하고 QGC 프리플라이트에서 "No valid local position estimate"가 사라지는지 확인합니다.

### 기타 프리셋

| 파일 | 용도 |
|---|---|
| `EKF2_helpers/no_vio_or_gps.params` | VIO/GPS 없이 Manual/Altitude만 |
| `EKF2_helpers/outdoor_gps.params`, `outdoor_gps_baro.params` | 야외 GPS 비행 |
| `EKF2_helpers/vio_gps_baro.params` | VIO + GPS 융합 |
| `EKF2_helpers/outdoor_flow.params` | 옵티컬 플로우 |
| `EKF2_helpers/ekf2_universal_tweaks.params`, `ekf2_altitude_seeker.params`, `exposed_baro.params`, `shielded_baro.params` | 공통 튠/기압계 노출 조건 |
| `radio_helpers/Commando_8.params` | ch1~4 롤/피치/스로틀/요, **ch5 킬 스위치**, **ch6 비행모드: 앞 Manual(0) / 중간 Position(2) / 뒤 Offboard(7)** |
| `voa_helpers/voa_enable_indoor.params` | `MPC_POS_MODE 0`, `CP_DIST 1.0`, `CP_DELAY 0.0`, `CP_GUIDE_ANG 0.0`, `CP_GO_NO_DATA 1` |
| `battery_helpers/*` | 배터리 셀 수/전압 프리셋 |

## 좌표계 (NED)

| 프레임 | 정의 | 사용처 |
|---|---|---|
| VIO 프레임 | Open-VINS 내부 프레임(IMU 바디 기준, `imu_body_frame_mode`) | `ov` 파이프 |
| vision-hub 로컬 프레임 | 이륙 지점 원점, 중력 정렬, **+Z 아래(중력 방향)**, 바닥 기준 | `vvhub_body_wrt_local`, PX4로 보내는 ODOMETRY |
| fixed 프레임 | AprilTag 등 외부 기준으로 정렬한 프레임 | `vvhub_body_wrt_fixed` (`en_tag_fixed_frame`) |
| PX4 로컬 NED | North-East-Down; 실내(자력계 없음)에서는 "N"이 이륙 시 기수 방향 | `vehicle_local_position`, MAVSDK `PositionNedYaw` |
| 바디 FRD | Forward-Right-Down | PX4 자세/속도 명령 |

실무 포인트: MAVSDK offboard에서 `PositionNedYaw(north, east, down, yaw)`의 **down은 음수가 위**입니다(고도 1 m = down −1.0). 원점은 배터리 연결(VIO 초기화) 지점이므로 항상 같은 자리에서 시작해야 재현성이 생깁니다.

# 카메라

## 카메라 종류

`voxl-configure-cameras <config#>`가 `/etc/modalai/voxl-camera-server.conf`를 생성합니다. C26/C27에서는 AR0144 드라이버가 슬레이브 모드로 `/usr/lib/camera`에 복사되어 **트래킹 카메라 프레임이 동기화**됩니다.

M0173 프런트엔드 포트 배치: J1 전방 트래킹(ID0) · J2 하방 트래킹(ID6) · J3 전방 ToF(ID3) · J4 전방 hires(ID1) · J5 하방 hires(ID2, Starling 2 미장착).

### tracking_front (VIO)

- 센서 **AR0144 (M0166)**: 1280×800, **글로벌 셔터**, 모노, FOV 162° × 139°(대각 84°는 문서 표기 참고), 3 µm 픽셀, f = 1.385 mm, F/2.4, 1.52 g.
- 전방을 향하며 VIO 1차 입력. 파이프 `tracking_front`. 캘리브레이션 파일 `/data/modalai/opencv_tracking_front_intrinsics.yml`.

### tracking_down (VIO)

- 동일 AR0144, 하방을 향함. 파이프 `tracking_down`. 바닥 질감을 보고 VIO 안정성을 높이며, 이륙 전 지면에 가려질 수 있어 `vio_cams.conf`의 `is_occluded_on_ground` 옵션이 존재합니다.
- 캘리브레이션 파일 `/data/modalai/opencv_tracking_down_intrinsics.yml`.

## hires

- 센서 **IMX412 (M0161)**: 4056×3040 (12.3 MP), **롤링 셔터**, 컬러, FOV 120.4° × 93.5°(대각 146°). 모드: 4056×3040 @ 15–60, 3840×2160 @ 15–80, 1920×1080 @ 15–240 fps.
- 파이프: `hires_small_color`(프리뷰/스트리밍), `hires_large_color`(고해상), 스냅샷(`hires_snapshot`), 인코딩 스트림(h264/h265). GPS가 있으면 스냅샷 지오태깅.
- **CPU/발열 부담이 가장 큰 카메라**입니다. 필요 없을 때는 `enabled: false` 또는 스트리밍 중지.

## 파이프 명세

카메라 서버 설정 필드: `type, name, enabled, camera_id, camera_id_second, fps, ae_mode(off|manual|auto|isp|lme_msv), en_preview, en_small_video, en_large_video, en_snapshot, *_venc_mode(h264|h265), *_venc_br_ctrl(cqp|cbr)` 등.

| 파이프 (`/run/mpa/`) | 소스 | 내용 |
|---|---|---|
| `tracking_front`, `tracking_down` | AR0144 | 모노 8-bit 프레임 (VIO 입력) |
| `hires_small_color` / `hires_large_color` | IMX412 | 컬러 프리뷰 / 대형 프레임 |
| `hires_snapshot` | IMX412 | JPEG 스냅샷 |
| `tof_depth` | PMD ToF | 깊이 이미지 |
| `tof_conf` | PMD ToF | 신뢰도 이미지 |
| `tof_pc` | PMD ToF | 포인트클라우드 (VOA 입력) |
| `tof_ir` | PMD ToF | IR 강도 이미지 |

기본 스트림 해상도/FPS는 공식 문서에 명시되지 않아 우리 설정 파일에서 확인해 기록합니다 `[확인 필요: cat /etc/modalai/voxl-camera-server.conf]`. 확인은 `voxl-inspect-cam <pipe>` 또는 포털 Cameras 탭.

## Stereo 카메라 (우리 드론 없음)

- Starling 2 C26/C27에는 스테레오 페어가 없습니다(개별 AR0144 트래킹 카메라). 따라서 `voxl-dfs-server`(`dfs_disparity`, `dfs_point_cloud`)는 동작하지 않고, 깊이 정보는 ToF 단독입니다.
- 스테레오가 필요한 실험(예: 장거리 깊이, 넓은 FOV 장애물 감지)은 Starling 2 Max나 별도 스테레오 모듈이 필요합니다.

# 센서

## TOF (하방 거리계)

- **PMD ToF (M0178, LIOW2)**: 240×180 px, 유효 범위 **약 0.1–6 m**(데이터시트 4–6 m 표기), FOV 106° × 86°(대각 138°), 5–45 fps, 940 nm, 약 0.9 W.
- Starling 2에서는 **전방**(M0173 J3)에 장착되어 VOA 입력(`tof_pc`)으로 쓰이고, vision-hub가 거리 데이터를 PX4 `DISTANCE_SENSOR`로도 전달해 `EKF2_RNG_CTRL 1`로 융합합니다. 하방 거리계 역할의 정확한 처리 경로(전방 ToF의 지면 반사 활용 여부)는 공식 문서에 명시되지 않아 (미확인)이며, 우리 드론에서 `px4-listener distance_sensor`로 실제 데이터 유무를 확인해 기록합니다 `[확인 필요]`.
- 점검: `voxl-inspect-tof`, `voxl-inspect-points tof_pc`, 포털 Pointclouds 탭.

## IMU

- VOXL 2 온보드 **ICM-42688P ×2**: IMU0 → SLPI(PX4 전용, 파이프 `imu_px4`), IMU1 → 앱 프로세서(`voxl-imu-server`, 파이프 `imu_apps`, VIO 입력).
- 설정 `/etc/modalai/voxl-imu-server.conf` (`imu_sample_rate_hz 1000`). 캘리브레이션 `/data/modalai/voxl-imu-server.cal`.
- 점검: `voxl-inspect-imu imu_apps`, 진동 `voxl-inspect-vibration` (RED > 5.00 m/s² 또는 > 0.50 rad/s, YELLOW > 2.00 m/s² 또는 > 0.20 rad/s). 진동이 RED면 VIO 품질이 급락합니다(프롭 밸런스/마운트 점검).

## Barometer

- VOXL 2 온보드 BMP388(0x76) + ICP-10100(0x63), SLPI에서 PX4가 직접 읽음. MPA로는 `px4_baro` 파이프(voxl-mavlink-server 경유), `voxl-inspect-baro`.
- 실내에서는 에어컨/문 개폐로 기압이 흔들리므로 고도 기준은 `EKF2_HGT_REF 3`(Vision)으로 두고 기압계는 보조로만 융합(`EKF2_BARO_CTRL 1`). 프리셋에 `exposed_baro` / `shielded_baro` 변형이 있습니다.

## Rangefinder

- Starling 2 기본 구성에는 **전용 1D 거리계가 없습니다**. `voxl-rangefinder-server`라는 서비스는 공식 문서에서 확인되지 않음(미확인). VL53L1CX 미니 거리계는 애드온으로 vision-hub `voa_inputs`의 `type: rangefinder`로 연결할 수 있습니다(docs.modalai.com/rangefinders).
- 현재 하방 거리는 ToF/VIO 고도에 의존하므로, 정밀 착륙이 필요하면 VL53L1CX 추가를 검토합니다.

# 장애물 회피 시스템

## VOA (Visual Obstacle Avoidance)

- vision-hub가 ToF(또는 DFS) 포인트클라우드를 받아 VIO 이력으로 **모션 보정**한 뒤 36개 파이 슬라이스(10°)로 축약해 MAVLink `OBSTACLE_DISTANCE`로 PX4에 전송합니다. 결과 포인트클라우드는 `voa_pc_out` 파이프에 퍼블리시(`voxl-inspect-points voa_pc_out`).
- 요구 조건: `en_voa true`, 올바른 `/etc/modalai/extrinsics.conf`, `voa_inputs`에 `tof` 항목 활성화.
- PX4 권장값(ModalAI): `CP_DIST 1.0~1.5`, `CP_DELAY 0.0`(vision-hub가 정확한 타임스탬프 처리), `CP_GUIDE_ANG 0.0`, `CP_GO_NO_DATA 1`, **`MPC_POS_MODE 0`**(PX4 mainline은 기본 Position 모드 4에서 VOA 미지원), `COM_OBS_AVOID 1`.
- 한계: ToF FOV(106°) 전방만 커버 → 측면/후방 장애물은 감지하지 않음. `voa_memory_s`로 최근 1초 장애물을 기억해 일부 보완.

## voxl-dfs-server (미지원)

- 스테레오 페어에서 disparity → `dfs_disparity`, `dfs_disparity_scaled`, `dfs_point_cloud`를 생성하는 서비스. **Starling 2에는 스테레오가 없어 사용 불가.** 서비스가 설치되어 있어도 비활성 상태로 둡니다(`systemctl disable voxl-dfs-server`).

## voxl-mapper (설치 가능)

- 3D 볼류메트릭 맵핑 + 경로 계획 서비스. ModalAI: "voxl-mapper를 돌리기 가장 좋은 플랫폼은 Starling 2" — 단, **ToF가 있는 SKU** 필요(우리 C26/C27 해당).
- 요구: VIO 정상, vision-hub, ToF, voxl-portal(Mapper 탭 의존).
- 설정 순서: ① `figure_eight` 모드로 Offboard 비행이 정상인지 먼저 검증 → ② `systemctl enable --now voxl-mapper` → ③ vision-hub `"offboard_mode": "trajectory"` → ④ 포털 Mapper 탭에서 목표 지정.
- 주의: 참조용(reference) 패키지이며 ModalAI가 손상에 책임지지 않음을 명시. SDK 1.4.5에서 trajectory 모드 중 천장으로 상승한 사례 → **SDK 1.6 이상** 필수.

## Collision Prevention (PX4)

- PX4 내장 기능. `OBSTACLE_DISTANCE`(또는 PX4 자체 distance_sensor)를 받아 **Position 모드에서** 장애물 방향의 속도 명령을 제한합니다.
- 파라미터: `CP_DIST`(최소 거리, −1 = 비활성), `CP_DELAY`(센서 지연 보상), `CP_GUIDE_ANG`(우회 유도 각), `CP_GO_NO_DATA`(데이터 없는 방향으로 이동 허용 여부).
- 제한: Position 모드 전용(Offboard/Mission에서는 동작하지 않음 → 자율 비행 코드에서는 별도 회피 로직 필요), 센서 데이터 없는 방향은 기본적으로 이동 차단.

# 비행 제어

## PX4 flight modes

| 모드 | 위치추정 | Starling 2 실내 사용 |
|---|---|---|
| Manual / Stabilized | 불필요 | 첫 이륙, VIO 이상 시 대피용 |
| Acro | 불필요 | 비권장 |
| Altitude | 불필요(기압) | VIO 상실 시 PX4가 자동 폴백 |
| **Position** | 필요(VIO) | 기본 비행 모드. ModalAI는 "Manual로 떠서 공중 전환보다 **Position으로 바로 이륙**이 더 안전"하다고 안내 |
| Hold | 필요 | 제자리 정지 |
| **Offboard** | 필요 | vision-hub 또는 MAVSDK가 setpoint 송신. 조종기 스틱 무시됨 |
| Takeoff / Land | 필요 | 자동 이착륙 |
| Return / Mission | 전역 위치 | 실내 사용 불가 |

Commando 8 프리셋 기준 ch6: 앞 = Manual, 중간 = Position, 뒤 = Offboard. ch5 = 킬 스위치.

## voxl-vision-hub offboard modes

PX4가 Offboard 모드로 들어가면 vision-hub가 `offboard_mode` 값에 따라 setpoint를 생성합니다.

### off (MAVSDK 방식)

- vision-hub는 setpoint를 보내지 않고 **외부 MAVLink 클라이언트(MAVSDK/MAVROS)** 가 Offboard를 제어합니다.
- 필수 설정: `"offboard_mode": "off"`, `"en_localhost_mavlink_udp": true`, `"localhost_udp_port_number": 14551`. 기본값이 `figure_eight`이므로 바꾸지 않으면 **MAVSDK 코드 대신 8자 비행이 실행**됩니다(포럼 다수 사례).
- MAVSDK는 Offboard 시작 전 setpoint를 먼저 스트리밍해야 하며(PX4 요구), 스트림이 끊기면 PX4가 failsafe(`COM_OBL_RC_ACT`)로 전환합니다.

### figure_eight

- Offboard 진입 지점을 중심으로(`figure_eight_move_home true`) 8자 궤적 비행. VIO/EKF2/vision-hub 연동을 검증하는 **표준 테스트 모드**(mapper/trajectory 사용 전 반드시 통과).
- 진입 시 "VIO 초기화 지점 위 2 m"로 이동한다는 공식 설명이 있어 천장 높이를 확인해야 합니다.

### follow_tag

- AprilTag(`voxl-tag-detector`, `tag_detections` 파이프)를 추적. ModalAI 공식 문구: **"R&D 전용, 권장하지 않음 — 장애물 인식 없이 태그를 쫓아감."** 포털 Follow 탭에서 설정.

### trajectory

- `voxl-mapper`가 퍼블리시하는 `plan_msgs`(경로)를 따라 비행. 경로는 현재 위치에서 시작해야 하며 `robot_radius`, `collision_sampling_dt`, `max_lookahead_distance`로 충돌 샘플링.
- 그 외 모드: `wps`(웨이포인트, `wps_move_home`, `wps_stride`), `backtrack`(왔던 길 되돌아가기), `vfc`(Visual Flight Control, `/etc/modalai/vfc.conf`).

## MAVSDK Python API

- 연결: `System(mavsdk_server_address="localhost", port=50051)` + 별도 실행한 `mavsdk_server udp://:14551`. (자동 서버 실행은 aarch64 휠에 바이너리가 없어 불가.)
- 주요 플러그인(0.12.0 기준, 이름은 최신과 대체로 동일):
  - `core.connection_state()` — 연결 확인
  - `telemetry.health()` (`is_local_position_ok`, `is_armable`), `telemetry.position_velocity_ned()`, `telemetry.attitude_euler()`, `telemetry.battery()`, `telemetry.flight_mode()`
  - `action.arm() / takeoff() / land() / kill() / set_takeoff_altitude()`
  - `offboard.set_position_ned(PositionNedYaw(n, e, d, yaw))` → `offboard.start()` → … → `offboard.stop()`; `set_velocity_ned`, `set_velocity_body`, `set_attitude`
  - `param.get_param_float / set_param_int`
  - `mission.*` — 실내(GPS 없음)에서는 사용하지 않음
- 실내에서 `takeoff()`는 `is_local_position_ok`가 true여야 성공합니다. `is_global_position_ok`는 GPS 없으면 항상 false이므로 예제 코드의 해당 조건은 제거/수정해야 합니다.
- 공식 예제: https://github.com/mavlink/MAVSDK-Python/tree/main/examples (`takeoff_and_land.py`, `offboard_position_ned.py`, `offboard_velocity_body.py`). 0.12.0에서는 `asyncio.get_event_loop().run_until_complete(run())` 형태를 사용.

# Modal Pipe Architecture (MPA)

## 파이프 개념

- ModalAI: "서비스, 도구, 그리고 이들을 연결하는 파이프의 조합을 **Modal Pipe Architecture(MPA)** 라 부른다."
- 각 서비스는 `/run/mpa/<파이프명>/` 디렉터리 아래 POSIX 파이프로 데이터를 퍼블리시하고, 클라이언트는 `libmodal_pipe`(C/C++)로 구독합니다. 1:N 구독, 낮은 지연, 프로세스 간 결합도 최소화가 목적.
- 서비스 간 의존 관계도 파이프로 표현됩니다(예: `voxl-open-vins-server` ← `imu_apps`, `tracking_*`; `voxl-vision-hub` ← `ov`, `tof_pc`, `mavlink_*`).

## 활성 파이프 목록

```bash
ls /run/mpa/            # 현재 존재하는 파이프
voxl-list-pipes         # 파이프 목록 + 타입
voxl-inspect-services   # 어떤 서비스가 켜져 있는지
```

우리 드론에서 기대되는 주요 파이프(실제 목록은 `[확인 필요: ls /run/mpa]`):

| 분류 | 파이프 | 퍼블리셔 |
|---|---|---|
| IMU | `imu_apps`, `imu_px4` | voxl-imu-server / PX4 |
| 카메라 | `tracking_front`, `tracking_down`, `hires_small_color`, `hires_large_color`, `hires_snapshot`, `tof_depth`, `tof_conf`, `tof_pc`, `tof_ir` | voxl-camera-server |
| VIO | `ov`, `ov_extended`, `ov_status` (QVIO면 `qvio`, `qvio_extended`, `qvio_overlay`) | voxl-open-vins-server |
| vision-hub | `vvhub_body_wrt_local`, `vvhub_body_wrt_fixed`, `vvhub_aligned_vio`, `voa_pc_out`, `vfc` | voxl-vision-hub |
| MAVLink | `mavlink_onboard`, `mavlink_ap_heartbeat`, `mavlink_sys_status`, `mavlink_attitude`, `mavlink_local_position_ned`, `mavlink_gps_raw_int`, `imu_mavlink`, `px4_baro`, `rc_channels`, `rc_active`, `mavlink_to_gcs`, `mavlink_from_gcs`, `gcs_ip_list`, `mission` | voxl-mavlink-server |
| 시스템 | `cpu_monitor` | voxl-cpu-monitor |
| 태그 | `tag_detections` | voxl-tag-detector |
| 맵퍼 | `plan_msgs` 등 | voxl-mapper (설치 시) |

## 파이프 접근 방법

1. **CLI 인스펙터** — `voxl-inspect-<종류> <파이프>`: `voxl-inspect-cam tracking_front`, `voxl-inspect-imu imu_apps`, `voxl-inspect-vio`, `voxl-inspect-points tof_pc`, `voxl-inspect-mavlink`, `voxl-inspect-cpu`, `voxl-inspect-battery`, `voxl-inspect-baro`, `voxl-inspect-tof`, `voxl-inspect-tags`, `voxl-inspect-vibration`, `voxl-inspect-extrinsics`, `voxl-inspect-vio-cams`.
2. **웹** — voxl-portal Cameras/Pointclouds/VIO 탭 (REST `/api/v1`).
3. **기록/재생** — `voxl-logger --preset_odometry --time 60`, `voxl-logger --cam tracking_front --samples 1`, `-i imu_apps`, `-a`(시동 중만) → `/data/voxl-logger/`; `voxl-replay`로 재생.
4. **C/C++** — `libmodal_pipe` 클라이언트 API(`pipe_client_open`, 콜백 등). 템플릿: https://gitlab.com/voxl-public/voxl-sdk/services (각 서비스 소스), 크로스 빌드는 `voxl-docker -i voxl-cross`.
5. **Python** — 공식 Python MPA 클라이언트는 문서에서 확인되지 않음(미확인). 실용적 대안: (a) MAVLink 경유(MAVSDK telemetry로 위치/자세/배터리), (b) `voxl-inspect-*` 출력을 `subprocess`로 파싱, (c) `/run/mpa/<pipe>/` 하위의 FIFO를 직접 열어 `libmodal_pipe` 메타데이터 구조체(`camera_image_metadata_t`, `pose_vel_6dof_t` 등)를 `struct`로 언패킹 — 구조체 정의는 `modal_pipe_interfaces.h` 참고.

# 서비스 관리

## systemd 서비스 목록

| 서비스 | 역할 | 우리 드론 상태 |
|---|---|---|
| `voxl-px4` | PX4 비행 컨트롤러 | `[확인 필요]` |
| `voxl-mavlink-server` | MAVLink 라우팅 (GCS 14550) | `[확인 필요]` |
| `voxl-vision-hub` | VIO→PX4, VOA, offboard 동작, localhost 14551 | `[확인 필요]` |
| `voxl-camera-server` | 모든 카메라 드라이버/파이프 | `[확인 필요]` |
| `voxl-imu-server` | IMU1 → `imu_apps` | `[확인 필요]` |
| `voxl-open-vins-server` | Open-VINS VIO (`ov`) | `[확인 필요]` |
| `voxl-qvio-server` | 구형 QVIO — Open-VINS와 동시 사용 금지 | `[확인 필요: disabled 권장]` |
| `voxl-cpu-monitor` | CPU/온도/팬 제어, `cpu_monitor` 파이프 | `[확인 필요]` |
| `voxl-portal` | 웹 대시보드(80) | `[확인 필요]` |
| `voxl-tag-detector` | AprilTag 검출 | `[확인 필요]` |
| `voxl-streamer` | RTSP 비디오 스트리밍 | `[확인 필요]` |
| `voxl-tflite-server` | TensorFlow Lite 추론(GPU/NPU) | `[확인 필요]` |
| `voxl-logger-auto` | 시동 중 자동 MPA 로깅 | `[확인 필요]` |
| `voxl-dfs-server` | 스테레오 깊이 — 우리 드론 미지원 | disabled |
| `voxl-mapper` | 3D 맵핑/경로계획 (옵션) | `[확인 필요]` |
| `voxl-io-server`, `voxl-rc-server`, `voxl-focus-camera` | SDK 1.4/1.6.4 신규 서비스 | `[확인 필요]` |
| `voxl-wait-for-fs`, `voxl-time-sync` 등 | 시스템 보조 | — |

한눈에 보기: `voxl-inspect-services` (옵션 `-v` 상세, `-j` JSON) — Enabled / Running / CPU 사용률 표시.

## 활성화/비활성화

```bash
systemctl status voxl-vision-hub
systemctl start|stop|restart voxl-vision-hub
systemctl enable|disable voxl-vision-hub          # 부팅 시 자동 시작 여부
systemctl enable --now voxl-mapper                # 활성화 + 즉시 시작
voxl-configure-vision-hub enable|disable|wizard   # 서비스별 configure 스크립트
voxl-configure-mpa                                # SKU 기준 전체 재구성(실패해도 계속 진행, 끝나면 전원 재인가)
```

- 설정 파일(`/etc/modalai/*.conf`)을 수정한 뒤에는 해당 서비스 `restart`. JSON 문법 오류가 있으면 서비스가 즉시 죽으므로 `journalctl`로 확인.
- 발열/CPU 절약을 위해 사용하지 않는 `voxl-tflite-server`, `voxl-streamer`, `voxl-tag-detector`는 `disable` 고려(단, follow_tag/fixed frame 사용 시 tag-detector 필요).

## 로그 확인

```bash
journalctl -u voxl-vision-hub -f          # 실시간
journalctl -u voxl-open-vins-server -n 100
journalctl -b -p err                      # 이번 부팅의 에러만
systemctl stop voxl-vision-hub && voxl-vision-hub -h   # 포그라운드 실행으로 상세 디버그
```

ModalAI 권장 디버그 루프: `systemctl status X` → `journalctl -u X` → 서비스 중지 후 바이너리를 포그라운드로 직접 실행. 흔한 원인: 서비스 disabled, `/etc/modalai/` JSON 오류, 상류(의존) 서비스 다운.

# 진단 도구

## voxl-inspect-*

`voxl-mpa-tools` 패키지의 인스펙터 모음. 대부분 `-h`로 옵션 확인.

| 명령 | 확인 내용 |
|---|---|
| `voxl-inspect-services` | 서비스 Enabled/Running/CPU |
| `voxl-inspect-cpu [-f -j -t]` | 코어별 클럭/온도/사용률, GPU, 메모리, governor(Auto/Perf/Powersave/Conserv/Cool) |
| `voxl-inspect-imu imu_apps` | 가속도/각속도, 샘플레이트 |
| `voxl-inspect-vibration` | FFT 기반 진동 등급 |
| `voxl-inspect-cam <pipe>` | 프레임 해상도/FPS/노출/지연 |
| `voxl-inspect-vio` / `voxl-inspect-vins` / `voxl-inspect-qvio` | VIO 포즈, 특징점 수, quality, state, 에러 코드 |
| `voxl-inspect-vio-cams`, `voxl-inspect-extrinsics` | VIO 카메라 설정/외부 파라미터 |
| `voxl-inspect-points <pipe>` / `voxl-inspect-tof` | 포인트클라우드/ToF 통계 |
| `voxl-inspect-mavlink <pipe>` | MAVLink 메시지 흐름 |
| `voxl-inspect-battery` | 전압/잔량/전류(`mavlink_sys_status` 기반) |
| `voxl-inspect-baro`, `voxl-inspect-gps` | 기압/GPS |
| `voxl-inspect-tags`, `voxl-inspect-detections` | AprilTag / tflite 검출 |
| `voxl-inspect-sku`, `voxl-version`, `voxl-check-calibration` | SKU/버전/캘리브레이션 파일 상태 |

## px4-listener

- PX4 uORB 토픽을 콘솔에 출력: `px4-listener <topic> [반복 횟수]`.
- 자주 쓰는 토픽: `vehicle_local_position`(X/Y/Z 갱신 확인), `vehicle_odometry`(VIO 입력), `estimator_status`, `input_rc`(조종기 수신), `battery_status`, `vehicle_status`(arm/mode), `distance_sensor`, `sensor_combined`, `vehicle_attitude`, `obstacle_distance`(VOA 수신 확인).
- 함께 쓰는 명령: `px4-param show [패턴]`, `px4-param set NAME VALUE`, `px4-commander check`(프리플라이트 사유), `px4-logger stop`.

## voxl-portal

- `http://<VOXL IP>/` (포트 80, 모든 인터페이스). 탭: **Cameras**(모든 이미지 파이프, Multi-View), **Pointclouds**, **Debug**(센서/기압계 캘리브레이션, RC 설정, 모터 테스트, 헬스), **VIO**(라이브/리플레이 궤적), **ELRS**, **Follow**, **Mapper**, **Benchmark**; CPU/IMU 플롯, FFT 진동 분석, 비행 정보. REST API `/api/v1`.
- 우측 상단 배터리/시동/모드가 `unknown`이면 mavlink-server 또는 PX4 미실행. 스트림은 **한 번에 하나만** 권장(CPU/발열).

## 여러 top/ps 응용

```bash
top -o %CPU                          # CPU 상위 프로세스
top -H -p $(pidof voxl-open-vins-server)   # 특정 서비스의 스레드별 부하
ps -eo pid,pcpu,pmem,comm --sort=-pcpu | head -15
voxl-inspect-cpu -f                  # ModalAI식: 온도 + 상위 프로세스
watch -n 1 'cat /sys/class/thermal/thermal_zone*/temp'   # 원시 온도(밀리℃)
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq  # 코어 클럭(스로틀링 확인)
df -h /data                          # 로그/VS Code 서버 용량
free -m                              # 메모리
iostat 1 / vmstat 1                  # (설치 시) I/O·컨텍스트 스위치
```

- VIO 프레임 드롭 의심 시: `voxl-inspect-cam tracking_front`의 FPS와 `voxl-inspect-cpu`의 클럭을 동시에 보며 스로틀링 상관관계 확인.
- `voxl-set-cpu-mode perf`로 governor를 고정하면 스로틀링이 클럭 저하로 명확히 관찰됩니다.

# 캘리브레이션

## IMU 캘리브레이션

- 명령: `voxl-calibrate-imu` — 정지 상태에서 자이로 자동 → 6방향 정지 자세 순서로 진행. 결과 `/data/modalai/voxl-imu-server.cal`.
- 온도 보정: `voxl-calibrate-imu-temp`(선택, IMU를 냉각/가열하며 진행) 후 일반 캘리브레이션.
- PX4 쪽 IMU(IMU0)는 별도로 QGC → Sensors(가속도/자이로/수평) 또는 포털 Debug 탭에서 캘리브레이션 → `/data/px4/param/parameters_{gyro,acc,level}.cal`.
- 재실행 시점: 충돌 후, 큰 온도 변화, 진동 등급 변화, VIO 드리프트 증가 시.

## 카메라 캘리브레이션

- **내부 파라미터(intrinsics)**: `voxl-calibrate-camera tracking_front -f`(피시아이 모델), 옵션 `-s 6x8`(내부 코너 수), `-l <m>`(격자 크기). ModalAI 기본 보드: 5×6 내부 코너, 65.5 mm. 출력 `/data/modalai/opencv_<cam>_intrinsics.yml`. 하방 카메라도 동일하게 `tracking_down`.
- **외부 파라미터(extrinsics)**: `/etc/modalai/extrinsics.conf` — 항목 `parent, child, T_child_wrt_parent[m], RPY_parent_to_child[deg]`. SKU별 공장 프리셋은 `voxl-configure-extrinsics`(configure-mpa가 실행), 확인은 `voxl-inspect-extrinsics`. Open-VINS는 extrinsics 오차에 민감하므로 카메라 마운트가 틀어졌다면 재측정 필요.
- 트리거 조건: VIO 에러 `0x800 BAD_CAM_CAL`, 렌즈/카메라 교체, 충돌 후 VIO quality 저하.

## 우리 드론 캘리브레이션 상태

```bash
voxl-check-calibration
ls -l /data/modalai/*.cal /data/modalai/*intrinsics.yml /data/px4/param/*.cal
voxl-inspect-extrinsics
```

| 항목 | 파일 | 상태 / 마지막 수행일 |
|---|---|---|
| IMU (apps, VIO용) | `/data/modalai/voxl-imu-server.cal` | `[확인 필요]` |
| tracking_front intrinsics | `/data/modalai/opencv_tracking_front_intrinsics.yml` | `[확인 필요]` |
| tracking_down intrinsics | `/data/modalai/opencv_tracking_down_intrinsics.yml` | `[확인 필요]` |
| extrinsics | `/etc/modalai/extrinsics.conf` (공장 프리셋) | `[확인 필요]` |
| PX4 gyro/accel/level | `/data/px4/param/parameters_{gyro,acc,level}.cal` | `[확인 필요]` |
| PX4 mag | `parameters_mag.cal` — 실내 구성(`SYS_HAS_MAG 0`)이면 불필요 | 해당 없음 |

Starling 2는 공장 캘리브레이션 상태로 출하되므로, 위 파일들을 **백업**해 두고 문제가 생겼을 때만 재캘리브레이션합니다.

# 알려진 이슈 및 제한

## 하드웨어 제한

- **발열**: 약 90 °C부터 스로틀링. 프롭 바람에 의존하는 설계라 벤치 장시간 구동/호버링 시 과열 → 팬 필수. hires 스트리밍이 주요 발열원.
- **깊이 센싱**: ToF 단독(0.1–6 m, 전방 106°). 스테레오 없음 → DFS 불가, 측·후방 장애물 미감지, 유리/거울/검은 표면에서 ToF 오측.
- **자력계/GPS**: 실내에서 사용 불가 → yaw 기준은 이륙 시 기수 방향. 세션 간 절대 방위 없음.
- **배터리**: 2S Li-Ion 전용(3S 금지). 비행 시간은 페이로드/스트리밍에 따라 급감.
- **트래킹 카메라**: 최소 초점 약 15 cm, 저조도·무질감 환경에서 VIO 저하. 롤링 셔터 hires는 VIO에 사용 불가.
- **전용 1D 거리계 없음**: 정밀 착륙/지면 추종은 ToF·VIO 고도에 의존.

## 소프트웨어 제한

- Ubuntu 18.04 / glibc 2.27 / Python 3.6.9 — 최신 VS Code Remote(1.86+/1.99+), 최신 MAVSDK-Python(3.7+), 최신 pip 패키지 다수 미지원. ModalAI는 온보드 개발에 Docker 권장.
- VIO 소스는 **하나만** 활성화 가능(QVIO + Open-VINS 동시 실행 시 실패).
- `offboard_mode` 기본값 `figure_eight` — MAVSDK 사용 시 `off`로 변경 필수.
- PX4 Collision Prevention은 Position 모드 전용, VOA는 `MPC_POS_MODE 0` 필요.
- `follow_tag`는 R&D 전용(공식 비권장). `voxl-mapper`/`trajectory`는 참조 구현(책임 면책).
- QGC는 WiFi(UDP 14550)만 가능, USB-C 불가.
- SDK는 개별 apt 업그레이드 대신 전체 플래시(플래시 중 전원 차단 = 벽돌 위험). SDK 1.4.5 이하에서 trajectory 천장 상승 버그 → 1.6 이상 필수.
- PX4 1.14 + FRD odometry + `SYS_HAS_MAG 0` 조합에서 "heading estimate invalid" 시동 거부 이슈(PX4 #27031).

## 미해결 문제

연구실에서 겪은 문제를 아래 형식으로 누적 기록합니다 `[작성 필요]`:

| 날짜 | 증상 | 재현 조건 | 시도한 조치 | 상태 |
|---|---|---|---|---|
| YYYY-MM-DD | 예: Position 모드 전환 직후 드리프트 | 예: 형광등 아래 무지 바닥 | 예: 바닥 패턴 추가, `voxl-reset-vins` | 진행 중 |
| | | | | |

관찰 시 함께 수집할 것: `voxl-version`, `voxl-inspect-services` 출력, `journalctl -u voxl-vision-hub -n 200`, 해당 비행 ulog(`/data/px4/log`), `voxl-logger --preset_odometry` 기록.

# 참고 링크

## 공식 문서

- Docs 루트: https://docs.modalai.com/
- Starling 2: https://docs.modalai.com/starling-2/ · 데이터시트 https://docs.modalai.com/starling-2-datasheet/ · 하드웨어 구조(D0014) https://docs.modalai.com/voxl2-d0014/ · 퀵스타트 https://docs.modalai.com/starling-2-hardware-quickstart/
- VOXL 2: https://docs.modalai.com/voxl2-connectors/ · 온보드 센서 https://docs.modalai.com/voxl2-onboard-sensors/ · 발열 https://docs.modalai.com/voxl2-thermal-performance/ · PX4 문서 https://docs.px4.io/main/en/flight_controller/modalai_voxl_2
- SDK: https://docs.modalai.com/voxl-sdk/ · voxl-suite/릴리스 노트 https://docs.modalai.com/voxl-suite/ · 시스템 이미지 https://docs.modalai.com/voxl2-voxl2-mini-system-image/ · 개발자 부트캠프 https://docs.modalai.com/voxl-developer-bootcamp/
- PX4 on VOXL: https://docs.modalai.com/voxl-px4/ · 파일 구조 https://docs.modalai.com/voxl-px4-files/ · RC 설정 https://docs.modalai.com/voxl2-rc-configs/ · VIO 비행 https://docs.modalai.com/flying-with-vio/ · QGC 연결 https://docs.modalai.com/qgc-wifi/
- 서비스: voxl-vision-hub https://docs.modalai.com/voxl-vision-hub/ · voxl-mavlink-server https://docs.modalai.com/voxl-mavlink-server/ · MAVSDK https://docs.modalai.com/mavsdk/ · Open-VINS https://docs.modalai.com/voxl-open-vins-server/ · 카메라 서버 https://docs.modalai.com/voxl-camera-server/ · IMU 서버 https://docs.modalai.com/voxl-imu-server/ · VOA https://docs.modalai.com/voa/ · DFS https://docs.modalai.com/voxl-dfs-server/ · Mapper https://docs.modalai.com/voxl-mapper/ , https://docs.modalai.com/setting-up-voxl-mapper/ · Portal https://docs.modalai.com/voxl-portal/ · Logger https://docs.modalai.com/voxl-logger/
- MPA/도구: https://docs.modalai.com/mpa/ · Inspect 도구 https://docs.modalai.com/inspect-tools/ · 서비스 디버깅 https://docs.modalai.com/debugging-services/ · configure-mpa https://docs.modalai.com/voxl-configure-mpa/
- 캘리브레이션: IMU https://docs.modalai.com/calibrate-imu/ · 카메라 https://docs.modalai.com/calibrate-cameras/ · extrinsics https://docs.modalai.com/configure-extrinsics/ · 확인 https://docs.modalai.com/check-calibration/
- 센서 모듈: AR0144 https://docs.modalai.com/M0166/ · IMX412 https://docs.modalai.com/M0161/ · ToF https://docs.modalai.com/M0178/ · 프런트엔드 https://docs.modalai.com/M0173/ · 거리계 https://docs.modalai.com/rangefinders/
- PX4: Collision Prevention https://docs.px4.io/main/en/computer_vision/collision_prevention.html · 비행 모드 https://docs.px4.io/main/en/flight_modes_mc/ · Starling https://docs.px4.io/main/en/complete_vehicles_mc/modalai_starling
- MAVSDK-Python: https://github.com/mavlink/MAVSDK-Python · API 문서 http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/

## GitLab 저장소

- voxl-public 그룹: https://gitlab.com/voxl-public
- 서비스 소스(voxl-vision-hub, voxl-camera-server, voxl-open-vins-server 등): https://gitlab.com/voxl-public/voxl-sdk/services
- PX4 파라미터 프리셋: https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-px4-params
- Docker 이미지(voxl-docker-mavsdk-python 등): https://gitlab.com/voxl-public/voxl-docker-images
- voxl-docker / voxl-cross: https://gitlab.com/voxl-public/support/voxl-docker
- PX4 ModalAI 포크(GitHub): https://github.com/modalai/px4-firmware

## Forum

- ModalAI 포럼: https://forum.modalai.com/ · Starling / Starling 2 카테고리: https://forum.modalai.com/category/44/starling-starling-2
- 참고 스레드: MAVSDK 서버 실행 https://forum.modalai.com/topic/4164/ · figure-8이 대신 실행되는 문제 https://forum.modalai.com/topic/4237/ · "No valid local position estimate" https://forum.modalai.com/topic/4272/ · Starling 2 Max 시동 불가 https://forum.modalai.com/topic/3961/ · CPU 스로틀링 https://forum.modalai.com/topic/3742/ · 천장 상승(SDK 1.4.5) https://forum.modalai.com/topic/4839/ · VS Code Remote 불가 https://forum.modalai.com/topic/4367/ · hires 과열 https://forum.modalai.com/topic/3946/
