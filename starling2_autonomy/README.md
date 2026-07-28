# Starling 2 Autonomy

> 애그테크연구실 [`Drone_project`](../) 의 하위 프로젝트. 다른 기체는 상위 폴더 참조.

ModalAI **Starling 2**(온보드 VOXL2) 드론에 **장애물 회피 · 안전장치 · 매핑**을 붙여
하드코딩 경로 비행에서 실제 자율비행으로 발전시키는 작업 기록.

조사 과정과 근거를 문서로 남기는 것을 목적에 포함한다. VOXL2 + PX4 + Offboard 조합에서
"공식 문서만 읽어서는 알 수 없었던 것들"이 `docs/` 에 정리돼 있다.

## 플랫폼

| 항목 | 내용 |
|---|---|
| 기체 | ModalAI Starling 2 (SKU `MRB-D0014-4-V1-C26-T8-M36-X0`) |
| 온보드 | VOXL2 (QRB5165, aarch64), Linux 4.19.125 |
| 비행 컨트롤러 | PX4 1.14.0 온보드 구동 (`voxl-px4`, Vendor 2.0.133) |
| 상태추정 | OpenVINS VIO — **GPS 없음** |
| 제어 | MAVLink ↔ **MAVSDK-Python** Offboard position setpoint (ROS2·pymavlink 미사용) |
| 회피용 센서 | **전방 ToF 1개**(106.5°×85.1°, 0.15–6m) + 하방 rangefinder. 스테레오 없음 |

## 현재 상태

| 항목 | 상태 |
|---|---|
| 경로 비행 (지그재그 커버리지) | 동작 — `flight/path_flight_phase1_v14.py` |
| 지오펜스 (위치 폭주 방어) | **v14에서 복구 완료.** 단위테스트 25/25, 기체 배포 완료, **실비행 미검증** |
| 장애물 회피 — 데이터 입력 | **`voa_pc_out` 파서 완료.** 단위테스트 60/60, **기체 실측 미검증** |
| 장애물 회피 — 정책 | 미착수. 선행조건인 **운용 환경(온실/실내/실외) 확인이 미해결** |
| 매핑 | 미도입 (`voxl-mapper` 는 beta + 제어권 충돌 위험으로 보류) |
| 배터리 저전압 failsafe | 없음 — PX4 레벨로 가능한지 먼저 확인 예정 |

### 조사에서 나온 핵심 결론

- **PX4 Collision Prevention 은 Offboard 모드에 개입하지 못한다.** 공식문서 · 기체 실제 펌웨어
  소스 · ModalAI 자체 파라미터 주석으로 3중 확증. Offboard setpoint 는 `flight_mode_manager` 를
  거치지 않고 `trajectory_setpoint` 로 `mc_pos_control` 에 직행하므로 CP 가 끼어들 지점이 없다.
  → **파이썬 레벨 회피가 유일한 경로.** (`docs/04`, `docs/06`, `docs/07`)
- **VOXL SDK 는 GitHub 에 없다. GitLab(`gitlab.com/voxl-public`)에 있다.**
  github.com/modalai 44개 중 37개가 upstream fork 다. (`docs/06` §0, `docs/07` §0)
- `voa_pc_out` 와이어포맷 확정: `point_cloud_metadata_t` = packed 60바이트,
  Python `struct` `"<IqIII32sI"`. `libmodal_pipe` 링크 없이 순수 파이썬으로 구독 가능. (`docs/07` §1-A)
- 설정 파일의 `enabled: true` 는 **하드웨어 존재 증거가 아니다.** `voa_inputs` 에 스테레오 3종이
  켜져 있지만 전부 죽은 템플릿이었다. (`docs/04`)

## 문서

| 문서 | 내용 |
|---|---|
| [`PROJECT_STRUCTURE.md`](PROJECT_STRUCTURE.md) | 전체 현황·구조·안전장치 갭 분석 |
| [`docs/01`](docs/01_remote_access_guide.md) | 원격 접속·제어·배포 방법 |
| [`docs/02`](docs/02_autonomy_roadmap.md) | 자율비행 구현 로드맵 |
| [`docs/03`](docs/03_CR_vs_v13_comparison.md) | CR 초기판 vs v13 비교 (지오펜스 회귀 발견) |
| [`docs/04`](docs/04_geofence_voa_investigation.md) | 지오펜스·VOA 조사 (공식문서 + 기체 실측) |
| [`docs/05`](docs/05_v14_geofence.md) | v14 지오펜스 복구 설계·검증 |
| [`docs/06`](docs/06_modalai_github_org.md) | ModalAI GitHub 저장소 44개 전수 정리 + 기체 펌웨어 커밋 특정 |
| [`docs/07`](docs/07_voxl_sdk_gitlab.md) | VOXL SDK(GitLab) 93개 + 와이어포맷 + 공식 회피 파라미터 |
| [`docs/08`](docs/08_voa_pc_parser.md) | `voa_pc_out` 파서 구현·검증·한계 + 기체 실측 절차 |

날짜별 작업 기록은 [`worklog/`](worklog/).

## 시작하기

```bash
git clone https://github.com/agtechresearch/Drone_project.git
cd Drone_project/starling2_autonomy

# 접속 설정 (IP·호스트명은 커밋되지 않는다)
cp .env.example .env.local
$EDITOR .env.local          # DRONE_HOST 를 ~/.ssh/config alias 로 지정

# 단위테스트 — 둘 다 기체 없이 실행 가능
python analysis/test_geofence_v14.py       # 지오펜스 25항목
python analysis/test_mpa_point_cloud.py    # voa_pc_out 파서 60항목
```

기체에 연결돼 있다면 포인트클라우드를 직접 볼 수 있다.

```bash
./tools/deploy.sh flight/mpa_point_cloud.py
ssh $DRONE_HOST "python3 /home/root/mpa_point_cloud.py voa_pc_out --seconds 5"
```

아래 명령은 모두 `starling2_autonomy/` 안에서 실행한다.

## 워크플로

**이 폴더가 비행 코드의 원본이다.** 기체(`/home/root/`)는 실행 사본이므로 기체에서 직접
편집하지 않는다.

```bash
# 로컬에서 수정 → 커밋 → 배포
git add -A && git commit -m "..."
./tools/deploy.sh                    # flight/path_flight_phase1_v14.py 를 기체로

# 기체와 저장소가 어긋났는지 확인
./tools/pull.sh                      # 차이 표시
./tools/pull.sh --apply              # 기체 내용 회수 (직후 반드시 커밋)
```

`deploy.sh` 는 **로컬 문법검증 → 기체 타임스탬프 백업 → scp → md5 대조 → 기체 python3
문법검증 → 배포기록** 을 순서대로 수행하고, 한 단계라도 실패하면 중단한다.

## 작업 규칙

- **분석·조회·빌드는 바로 실행.** 실제 비행, PX4 파라미터 변경 등 되돌리기 어려운 동작은
  **먼저 확인**받는다.
- 기체에는 **직전 안정판(v13)을 롤백용으로 보존**한다. 버전 관리 자체는 파일명 증분이 아니라
  git 이력 + 태그로 한다.
- 실비행으로 검증된 커밋에는 태그를 단다. 태그는 저장소 전체가 공유하므로
  **`starling2-` 접두사를 반드시 붙인다** (`starling2-v14-verified` 형식).
- 커밋 메시지도 다른 기체 작업과 섞이므로 `[starling2]` 로 시작한다.
- "정리"/"save" 입력 시 그날 내용을 `worklog/YYYY-MM-DD.md` 에 기록한다.

## 주의

이 저장소의 코드는 **실제 모터를 구동하는 비행 코드**다. 그대로 가져다 쓰기 전에
기체 형상·운용 환경·파라미터가 다르다는 점을 반드시 검토할 것. 실비행 검증이 끝나지 않은
코드가 포함돼 있다(현재 v14 지오펜스가 그렇다).
