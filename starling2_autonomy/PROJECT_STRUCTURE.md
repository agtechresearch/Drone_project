# Starling2 Autonomy — 프로젝트 구조 및 현황

> ModalAI VOXL2 기반 Starling 2 드론의 자율비행(장애물 회피·매핑·안전장치) 개발 작업공간
> 최종 갱신: 2026-07-27

---

## 1. 하드웨어 / 플랫폼

| 항목 | 내용 |
|---|---|
| 기체 | ModalAI **Starling 2** |
| 온보드 컴퓨터 | **VOXL2** (QRB5165, aarch64) — hostname `m0054` |
| OS / 커널 | Linux 4.19.125 aarch64 |
| 비행 컨트롤러 | **PX4** (voxl-px4, 온보드에서 직접 구동) |
| 상태추정 | **OpenVINS VIO** (`voxl-open-vins-server`) — GPS 없이 위치추정 |
| 통신 | MAVLink (`voxl-mavlink-server`) ↔ MAVSDK-Python |
| 원격접속 | SSH (같은 네트워크), user `root`. 대상은 `~/.ssh/config` alias 로 관리 |

### 접속 정보

실제 IP·호스트명은 **저장소에 커밋하지 않는다**. `.env.example` 을 `.env.local` 로 복사해
본인 환경값을 채운다(`.gitignore` 처리됨). 설정 방법은 `docs/01`.

기체는 접속 경로가 셋이며 네트워크마다 IP가 다르다 — alias 를 셋 만들어두고 골라 쓴다:
**랩 네트워크 / 기체 자체 AP / 모바일 핫스팟.**

---

## 2. 센서 / MPA 파이프 (장애물 회피·매핑 가용 자원)

VOXL은 MPA(Modal Pipe Architecture)로 센서 데이터를 파이프(`/run/mpa/*`)로 공유한다.
**자율주행 안전장치 구현에 직접 쓸 수 있는 데이터가 이미 흐르고 있음:**

| 파이프 | 내용 | 용도 |
|---|---|---|
| `tof`, `tof_depth`, `tof_pc`, `tof_ir` | **전방 ToF 깊이센서** (240x180, Zmax 7.4m, 확인시 정면 ~2.0m) | 전방 장애물 감지 |
| `voa_pc_out` | **VOA 장애물 포인트클라우드** (vision-hub가 이미 생성 중) | 충돌 방지 입력 |
| `rangefinders` | 하방 거리센서 | 고도/지면 감지 |
| `tracking_front`, `tracking_down` | 트래킹 카메라 (ar0144 **단안** ×2 — 스테레오 아님, docs/04) | VIO / 추가 인식 |
| `ov`, `ov_extended`, `ov_status` | OpenVINS 포즈/상태 | 상태추정 |
| `px4_vehicle_local_position` | PX4 로컬 위치 | 도달 판정 (현 코드가 사용) |
| `hires_*` | 고해상 컬러 카메라 | 매핑/기록 |

### 미설치 (SDK 경로계획 패키지)
- `voxl-mapper` (3D 점유맵) — **미설치**
- `voxl-planner` (경로계획) — **미설치**
- → 3D 매핑/전역 경로계획을 SDK로 쓰려면 별도 설치 필요

### vision-hub VOA (충돌 방지) 기능
- `voxl-vision-hub.conf`에 **`en_voa` 등 Collision Prevention 설정 존재** (내장 기능).
- PX4에 `OBSTACLE_DISTANCE` MAVLink 메시지를 보내 충돌 방지.
- ⚠️ **주의(확인 필요)**: PX4 Collision Prevention은 기본적으로 *Position 모드*에서 동작.
  현재 비행 코드는 **Offboard position setpoint** 방식이라, VOA가 offboard 셋포인트를
  가로채는지 실제 검증 필요. (→ docs/02 참고)

---

## 3. 기체 파일 구조 (`/home/root/`)

기체는 **실행 사본**이다. 원본은 이 저장소의 `flight/` 이며 배포는 `tools/deploy.sh` 로 한다.

### 메인 비행 코드
- `path_flight_phase1_v14.py` (115KB, 2431줄) — **최신. v13 + 지오펜스 감시**(업로드 완료, 실비행 미검증)
- `path_flight_phase1_v13.py` (105KB, 5/21) — **직전 안정판, 롤백용으로 보존.** 비행 실적 있음
  (v14의 비행 로직은 v13과 완전히 동일)
- `path_flight_phase1_v3~v12.py`, `_s1~s5.py` — 이전 버전들 (git 이전 시대의 증분 백업)
- `*.bak.<타임스탬프>` — `deploy.sh` 가 덮어쓰기 전에 자동 생성하는 백업
- `.deploy_log` — 배포 이력 (타임스탬프 / md5 / git 커밋)

### 보조 스크립트 (저장소 `flight/tools_onboard/`)
- `check_health.py` — 헬스체크
- `backup_params.py` — PX4 파라미터 백업
- `position_test.py` — 위치 테스트

### 로그/데이터
- `px4_reach_sync_*.csv`, `vio_latency_sync_*.csv` — 비행 로그
  (용량이 커서 `.gitignore` 처리. 분석이 필요하면 `logs/` 로 내려받아 쓴다)

---

## 4. 현재 비행 코드(v13) 아키텍처

`DroneController` 클래스 (async / MAVSDK 기반):

```
main() → main_async(args)
  └ DroneController
     ├ connect()                       MAVSDK 연결
     ├ wait_position_stable()          로컬 위치 health 대기
     ├ check_vio_quality_preflight()   VIO 프리플라이트
     ├ capture_initial_mavsdk_position() 초기 NED 원점 캡처
     ├ arm_and_start_offboard()        모니터 태스크 시작 + arm + offboard
     │   ├ setpoint_streamer()   20Hz로 self.target 셋포인트 전송
     │   ├ mavsdk_position_monitor()   위치 갱신
     │   ├ health_monitor()      health 손실 3회 → emergency_land
     │   └ voxl_pose_reader()    px4 pipe에서 도달 판정용 포즈 읽기
     ├ execute_mission()               MISSION_PLAN 순차 실행
     │   ├ go_to_relative() / ramp_setpoint_to()  위치 이동
     │   ├ go_yaw() / ramp_yaw_to()    회전
     │   ├ hover()                     정지 유지
     │   └ wait_until_reached()        pipe 피드백으로 도달 판정
     └ land_and_disarm() / trigger_emergency_land() / emergency_stop()
```

- **미션 정의**: 파일 상단 `MISSION_PLAN` 리스트 (지그재그 커버리지 경로 하드코딩).
- **좌표계**: `MOVE_FRAME="body"` (yaw 기준 상대이동) / `"world"` (NED 고정).
- **도달 판정**: MAVSDK telemetry가 아니라 `voxl-inspect-pose px4_vehicle_local_position` 값 사용.
- **속도 제어**: velocity 필드가 아니라 position setpoint를 ramp로 나눠 이동.

---

## 5. 안전장치 현황 (핵심 갭 분석)

### ✅ 이미 있는 것
- 로컬위치 health 손실 3연속 → `trigger_emergency_land`
- 셋포인트 스트림 5회 실패 → emergency land
- 이륙 전 VIO/위치 안정성 검증 (프리플라이트)
- `emergency_stop`, `aborted` 플래그로 미션 중단
- 정밀착륙 preland 하강 단계

### ✅ 추가 완료 (v14, 2026-07-27)
- **위치 편차 안전장치(지오펜스) 복구** — v13은 `MAX_HORIZONTAL_DEV`/`MAX_ALT_DEVIATION`이
  상수·CLI인자로만 있고 강제 로직이 없었다(CR 대비 회귀). v14에서 두 겹 펜스로 복구.
  단위테스트 25/25 + 기체 업로드·md5 검증 완료. **실비행 미검증.** → `docs/05`

### ❌ 없는 것 / 미완 (구현 목표)
- **장애물 회피 전무** — 코드에 obstacle/avoid/collision/tof/voa 관련 로직 0건.
  ToF·VOA 데이터가 흐르는데 **파이썬 비행 로직이 전혀 읽지 않음**.
  → PX4 Collision Prevention은 **Offboard에 개입 못 함이 3중 확증**됨(`docs/04`·`docs/06`·`docs/07`).
  **파이썬 레벨 회피가 유일한 경로.** `voa_pc_out` 와이어포맷은 확정됨(`docs/07` §1-A).
- **매핑 없음** — `voxl-mapper` 미설치. 소스는 GitLab에 있고 ToF-only 호환이나
  beta + 제어권 충돌 위험으로 **도입 보류** (`docs/07` §4).
- 배터리 저전압 failsafe (파이썬 레벨) 없음 — 단 PX4 `VOXLPM` 드라이버·`battery_helpers`
  파라미터가 있어 **파이썬 구현 전 PX4 레벨로 되는지 먼저 확인**할 것.

---

## 6. 폴더 구조

이 프로젝트는 애그테크연구실 `agtechresearch/Drone_project` 저장소의 하위 폴더다
(같은 레벨에 `bee35_programing/`, `advancedroneprograming/`, `drone_sim/` 등 다른 기체 프로젝트가 있다).

```
Drone_project/starling2_autonomy/
├── PROJECT_STRUCTURE.md   ← 이 문서 (전체 현황·구조)
├── README.md              ← 빠른 시작 / 접속법 / 규칙
├── .env.example           ← 접속 설정 템플릿 (.env.local 로 복사해서 사용)
├── worklog/               ← 날짜별 작업 기록 (YYYY-MM-DD.md)
├── docs/                  ← 가이드·설계 문서
│   ├── 01_remote_access_guide.md      원격 접속·제어·배포 방법
│   ├── 02_autonomy_roadmap.md         자율비행 구현 로드맵 (단계1-A 폐기 반영)
│   ├── 03_CR_vs_v13_comparison.md     CR 초기판 vs v13 비교
│   ├── 04_geofence_voa_investigation.md  지오펜스·VOA 조사(공식문서+기체실측)
│   ├── 05_v14_geofence.md             v14 지오펜스 복구 설계·검증
│   ├── 06_modalai_github_org.md       ModalAI GitHub 44개 전수 정리
│   └── 07_voxl_sdk_gitlab.md          VOXL SDK(GitLab) 93개 + docs.modalai.com
├── flight/                ← 비행 코드 원본 (기체에 배포되는 실체)
│   ├── path_flight_phase1_v14.py      현행 메인 코드
│   ├── legacy/                        v13·CR·초기판 (참조용, 배포 안 함)
│   └── tools_onboard/                 기체에서 돌리는 보조 스크립트
├── tools/                 ← 기체 동기화
│   ├── deploy.sh                      로컬 → 기체 (백업·md5·문법검증 포함)
│   └── pull.sh                        기체 ↔ 저장소 대조 (drift 감지)
└── analysis/              ← 단위테스트·분석 산출물
    └── test_geofence_v14.py           v14 지오펜스 25항목 검증
```

> **비행 코드의 원본은 이 저장소다.** 기체는 실행 사본이므로 기체에서 직접 편집하지 않는다.
> 버전 관리는 파일명 증분(v13→v14)이 아니라 **git 이력 + 태그**로 한다.
> 단, 기체에는 롤백을 위해 직전 안정판(v13)과 `deploy.sh` 자동 백업을 남겨둔다.
