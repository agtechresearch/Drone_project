# 12. 마커 드리프트 실험 — 진행 기록 (단일 문서)

이 실험의 **모든 진행 과정을 이 문서 하나에** 시간순으로 기록한다 (2026-11-10 결정: 실험 하나 = 기록 문서 하나).
설계서 `docs/09_marker_drift_experiment_plan.md` 와 랩미팅 요약 `_brief.md` 는 "무엇을 할 것인가", 이 문서는 "실제로 무엇이 일어났는가".
새 항목은 **§1 진행 기록** 끝에 날짜로 추가하고, 코드가 바뀌면 §2 패치노트, 문제를 겪으면 §3 트러블슈팅에도 한 줄씩 남긴다.

**현재 상태 (2026-11-12 오후):** 11-12 수동 비행(Position)에서 또 좌측 밀림·좌회전 → 원인 확정: **EKF 자이로 z 바이어스 -3.5 °/s**(부팅 31 s 만에 arm, 지상 정지에서도 EKF 헤딩이 3.5 °/s 회전). 기체·모터 정상.
v0.2.2b 배포됨(정지 헤딩 드리프트·부팅 경과·EKF 자이로 바이어스·yaw 샘플 0 게이트 + 비행 중 EKF-VIO yaw 발산 착륙) — 단위테스트 195/195, SITL A11 완료·V7 헤딩 발산 착륙 확인.
**다음 비행은 사용자 "시작" 뒤 0.4 m 점검 비행부터.** mavsdk_server·태그 검출기 기동됨(05:51 UTC).
남은 관문: 기체 재부팅 → 이륙 지점에 둔 채 손대지 않고 2~3 분 → `vio_check.py` PASS(헤딩 0 °/s·바이어스 < 1 °/s) → 사용자 "시작" → 0.4 m 점검 비행. `EKF2_EV_CTRL 15` 적용됨(05:30 UTC, 재부팅).

---

## 1. 진행 기록

### 2026-11-09 — 설계 확정, 코드 작성
- 실험 코드를 v14 파일 증분이 아니라 **전용 패키지 `flight/s2flight/`** 로 만들기로 함. v14·기존 파일 불변. 진입점 `flight/marker_drift_flight.py`.
- 기하 확정: 태그 ID 0~10, 간격 0.5 m(5.0 m), 이격 70 cm, 태그 20 cm(원문 "200센치" 는 20 cm 로 해석, 확인 필요).
- 제외: 프롭 탈착 지상 시험. 대신 **서버 SITL 선검증 → 실기 짧은 호버 → 본실험** 순.
- 정면 기준 변경: VIO 전원 투입 방향(v14) 대신 **이륙 직전 기체 방위**(`--initial-yaw current`). 전원을 다른 곳에서 켜면 이륙 후 돌아가던 문제 해소.
- 시퀀스 재설계(도달 판정 제거): 이륙 → 좌회전 → 태그 0 정렬·센터링(t0) → 우측 이동하며 태그별 자기추정 vs 실측 기록 → **태그 10 이 카메라 중앙이면 정지**
  → 복귀 → **태그 0 중앙이면 정지** → 착륙. 끝 태그 못 찾으면 상한(현수막 길이 + `--max-extra`)에서 **그 자리 착륙**. 자기 추정 이동량은 비행에 쓰지 않고
  기록만(그것이 드리프트). `Vehicle.move_until`, `s2flight.drift` 신설. 단위테스트 139 항목.
- 동료 의견 "v14 로 .sh 만 짜면 됐다": 원래 개방루프 설계(docs/09 초안)에는 맞는 말. 태그 정지·폐루프 정렬·센터링은 신규라 패키지 필요.
  `--stop-mode distance` 로 v14 동작 비교 가능하게 남김.
- 사용 가이드 아티팩트: https://claude.ai/artifact/7b1MW8JVCjHVKhb88LAFYg

### 2026-11-10 — 커밋·서버 접속·SITL 선검증·오류 주입
- 커밋·푸시 `f5f4763` (origin 에 drone_sim_gz 가 main 으로 병합돼 있어 리베이스).
- 서버 접속: Tailscale 노드 공유(구글 계정) + `ssh dronesim_tailscale`(키). 22번만 열림. TurboVNC 는 ssh 터널. 상세 §3.
- 배치 `sync_to_server.sh` → `~/starling_sim/marker_drift/`. 서버 레이아웃·컨테이너·의존성 모두 기본값과 일치.
- 월드 설치 스크립트 버그 1건 수정(§2 v0.1.1) 후 `drift_banner` 월드 설치.
- 비행 8회(인스턴스 2·3): 정상 T1·A1 통과, 오류 주입 D/R/S/N/M/J 모두 설계대로. 표는 §2, 수치는 `worklog/2026-11-10.md`.
- 발견: 횡방향 점프 0.8 m 가 지오펜스(2.5 m)에 안 걸림 → 통로 실험은 `--max-horizontal-dev 1.0` 권고. 큰 드리프트에서 센터링 미수렴은 경고 후 진행(유효).
- 추가 도구: `INJECT` 환경변수, `summarize_run.py`. 로그 사본 `C:\Users\ericj\sim_logs\marker_drift\`.
- 결정: 앞으로 실험 진행 과정은 **이 문서 하나**에 기록.

### 2026-11-10 (저녁) — 실험장 변경: 상단열도 ID 0~10, 설치 기준점 좌상단 꼭짓점
- 사용자: 마커는 **좌상단 꼭짓점 기준**으로 붙임. 상단열도 **ID 0~10** (당초 20~30 폐기). 20 cm 태그 배치 재검토 요청.
- 배치 결론: 중심 간격 0.5 m 유지(검은 사각형 사이 30 cm, 인쇄지 사이 25 cm). 좌상단 꼭짓점 X = 500·k mm(태그 0 기준), Z = 700 mm(하단)·2100 mm(상단),
  인쇄지 기준이면 Z +25. 현수막 바닥 350~850 mm. 비행 코드는 ID × 0.5 m 만 쓰므로 꼭짓점 간격·한 평면·수직이 정밀도의 전부. docs/09 §4.1 갱신.
- 같은 ID 두 열 → 비행 코드 영향 검토: 핀홀 보정(undistort_scale 0.6, fx≈277) 프레임에서 다른 열(1.4 m 위/아래)은 이격 0.7 m 에선 화각 밖이지만
  **이격 ≥ 1.0 m(센터링 전·정렬 중)면 들어온다**. `per_tag` 가 ID 키라 같은 ID 두 검출이 섞이면 센터링·정지 판정이 흔들릴 수 있음 →
  **상하 |y_cam| > 0.7 m 검출 배제** 필터 추가 (`--align-max-vertical`, v0.1.2). 단위테스트 144/144.
- 후처리 배치 파일은 ID 키라 열별로 분리: `lab_default.yaml`(하단) + `lab_upper.yaml`(상단).
- SITL 2열 월드(`make_banner_world.py --upper`, 상단열 ID 0~10) 재설치 후 A(0.6 m)·C(2.0 m) 각 1회: 결과 §2 v0.1.2 표.

### 2026-11-10 (밤) — 시뮬 파일럿 리허설 A·B·C × 3, offboard 시작 재시도
- 사용자 결정: 파일럿을 시뮬로 먼저 리허설. 의견 교환 결과 **시뮬 s 는 n 산정에 쓰지 않는다**(VIO 가 진실값 릴레이라 드리프트가 아니라 코드 재현성).
  목적은 절차·분석 파이프라인 예행과 파라미터 확정. 실기 파일럿 3회/조건은 계획대로.
- 실행: 인스턴스 2·3·4 병렬, 교대 순서 A B C → C B A → A B C (`pilot/batch.sh`). B 는 `--max-horizontal-dev 1.0`, C 는 `--altitude 2.0`(상단열 ID 0~10).
  서버는 타 사용자 Gazebo 작업과 겹쳐 load 20~28/32 코어.
- 결과 9/9 완료(A2 는 1차 실패 후 재실행). 통계 `sim/pilot_stats.py` → `pilot/pilot_sim_9.csv`(로컬 `C:\Users\ericj\sim_logs\marker_drift\`):

  | 조건 | n | 최종 드리프트 E(out_settled) mean / s | 복귀 드리프트(back_settled) mean / s |
  |---|---|---|---|
  | A 0.6 m | 3 | −0.005 / 0.005 m | +0.004 / 0.007 m |
  | B 0.6 m 통로 설정 | 3 | −0.007 / 0.006 m | +0.014 / 0.026 m (B2 +0.043 이상치) |
  | C 2.0 m 상단열 | 3 | −0.008 / 0.005 m | −0.003 / 0.004 m |

  조건 간 차이 |t| < 1 (당연: 시뮬은 조건이 같다). **코드 바닥 잡음 s ≈ 0.5 cm** → 실기 s 가 이보다 크면 그 초과분이 VIO 드리프트.
  Δ 3 cm 기준 n_req 2 는 시뮬 수치일 뿐 실기에 적용하지 않는다.
- 실패 1건: A2 1차(인스턴스 4) arm 직후 `Offboard FAILED: NO_SETPOINT_SET` → 비상정지(착륙·disarm, 안전 동작은 정상). 과부하로 setpoint 스트림이
  PX4 기준 시간 안에 안 닿은 것으로 판단. v14 와 같은 로직(3 s priming → arm → 1 s → start, 재시도 없음)이라 **재시도 3회** 추가(v0.1.3, §3).
- 관찰: B2 복귀 settled 드리프트 +0.043 m 는 정지 순간 dx +0.065 오버슈트 뒤 태그 0·1·2 기반 truth 추정과 자기 추정의 차. 시뮬 VIO 가 진실값이므로
  태그 추정 쪽 오차(가장자리 태그·2 cm 급)로 보인다. 실기에서 같은 크기가 반복되면 `center_tol`·hover_return 길이 재검토.
- VNC: Gazebo GUI 를 실행 중 인스턴스에 붙여 보는 방법 확립(§4, sim README). 마우스 회전·줌·Follow 가능.
- 서버 정리: dronesim 계정 고아 프로세스(Chrome 6, ugrep 4) 종료. 타 사용자 컨테이너·프로세스는 손대지 않음.

### 2026-11-10 (밤, 추가) — 실험장 실측: 현수막이 짧아 태그 0~9, 꼭짓점 간격 50 cm
- 사용자: 현수막 길이가 모자라 **태그 9 까지만** 부착(10장). 마커 좌상단 꼭짓점 간 거리 **50 cm** (= 중심 간격 0.5 m, 설계값과 같음).
- 비행 구간 5.0 → **4.5 m**. 끝 태그 10 이 없으므로 `--end-tag` 기본값을 **9** 로 변경(v0.1.4). 안 바꾸면 기체가 태그 10 을 찾아 상한 6.0 m 까지 가서
  그 자리 착륙하는 동작이 된다. 상한은 자동으로 4.5 + 1.0 = 5.5 m.
- 시뮬 월드 기본값도 0~9 로 맞추고 재설치, SITL A 1회 재검증(§2 v0.1.4). 배치 파일 `lab_default.yaml`·`lab_upper.yaml` 에서 id 10 제거.
- docs/09(§0·§1.2·§2·§4.1·§5.2·§10), brief, sim README, docs/10 권고(`tag_locations.conf` id 0~9 size 0.20) 갱신. 드리프트율 분모 4.5 m.
- 사용자(추가): 하단열 **검은 사각형 상단 75 cm / 하단 55 cm** (중심 65, 한 변 20 cm 확인). 설계 60 과 5 cm 차이. 비행 고도 0.6 유지 — 포즈 추정은
  상하 위치와 무관, |y_cam| ≈ 5 cm (필터 0.7 m 여유 충분). `lab_default.yaml` z 0.65, 시뮬 `--z` 기본 0.65, docs/09 §4.1 실측값 기록. 상단열 높이는 실측 전.

### 2026-11-10 (밤, 실험장 확정) — 하단열 완료, 상단열 보류, 통로 B = 마커 2~6
- 사용자: 하단열(태그 0~9, 중심 65 cm) 부착 완료. **상단열은 오늘 준비 안 됨 → 조건 C 보류.** 후방 장애물 환경(B)은 **마커 2~6 구간에만** 있다.
- 영향: B 비행 자체는 0~9 전 구간 그대로(코드 변경 없음). 분석에서 2→6 구간(1.0~3.0 m) err_along 증가분을 A 와 비교하도록
  `pilot_stats.py` 에 `seg_out`/`seg_back` 지표 추가(`--seg-from 2 --seg-to 6`, 각 지점 ±0.25 m 프레임 평균). 시뮬 9회: 구간 증가분 평균 ≈ 0, s ≈ 1 cm (이동 중 측정이라 settled 보다 잡음 크고, 이것이 구간 지표의 바닥 잡음).
- 오늘 실기 계획(상단열 없이): 손 검출 시험 → 점검 비행 1회 → 파일럿 A·B 교대 3회씩(A B B A A B) → s 산출. C 는 상단열 설치 후.
- 통로 폭(전방 현수막 ↔ 후방 장애물) **140 cm** 실측(사용자). 기체 이격 70 이면 후방도 70, 통로 중앙. docs/09 §0·§1.2·§4.1·§4.2 갱신. 통로 구간은 `--max-horizontal-dev 1.0` 권고 유지.

### 2026-11-10 (밤) — 기체 세션 1: 배포·태그 검출기 설정·손 검출 시험
- 기체 IP .139 → **192.168.45.203** (DHCP, 호스트키로 식별). `starling2_lab` 별칭 갱신, `.env.local` DRONE_HOST=starling2_lab. 부팅 3분 후 VIO 포즈 안정(±1 cm), CPU 50 °C.
- `tools/deploy_pkg.sh -y` 로 v0.1.5(git e78f5ad) 배포: 15 파일 md5 일치, 기체 py3.6.9 문법 통과. 백업 `s2flight.bak.20261110-205509.tgz`.
- **설정 변경(사용자 승인)**: `tag_locations.conf` id 0~10 size 0.20 + default 0.20 (당초 id 0 만 0.4), `voxl-tag-detector.conf` detector_0 skip_n_frames 5→1,
  detector_1(tracking_down) enable false. 백업 `*.conf.bak.20261110-120019`. `systemctl start voxl-tag-detector` (enable 안 함, 재부팅 시 꺼짐). 파이프 `tag_detections` 생성, 15 Hz.
- **손 검출 시험** (태그 10 을 전방 70 cm 에 고정, 30 s, `hand_test.py`): 태그 10 range **0.712 m** (실측 0.70 → 크기 0.20 설정 맞음), x −0.033 y −0.086 z 0.706,
  σ_pos < 1 mm, **σ_yaw 0.19° → align-tol = max(1°, 3σ) = 1.0°** (기본 1.5 대신 `--align-tol 1.0`). 현수막 태그 1·2 도 보임: x ≈ −0.7 (카메라 **왼쪽**),
  y ≈ −0.6 (위), z 0.57/1.03 (현수막이 전방으로 뛻어 있음) → 기체 왼쪽에 현수막, 전방 = ID 증가 방향이면 **x 부호 = 오른쪽 +** 확인. 태그 2(1.37 m, 비스듬한 각) 단일 yaw σ 14.6° —
  단일 태그 yaw 모호성 재확인(설계대로 2태그 직선 맞춤 사용).

### 2026-11-10 (낮) — **사고: 점검 비행 T1 이륙 중 천장 근접 상승 → 조종자 개입 → 책상 충돌**

**비행 금지.** 원인 분석이 끝나고 아래 대책이 코드·절차에 들어가 사용자가 다시 허락하기 전까지 arm 하지 않는다.

> **정정 (같은 날 저녁, 사용자 터미널 전문 확인 후):** 아래 타임라인의 콘솔 시각(12:28:17~12:29:04, 폴더 `122109_T1`)은 **사고 비행이 아니라 추락 직후의 재시도**다.
> 실제 사고 비행은 **12:27:47 arm, 폴더 `20261110_122731_T1`**, PX4 `log100.ulg`(이 로그는 처음부터 맞게 골랐으므로 물리적 결론은 그대로).
> 정정되는 사실: ① 사고 비행 **전** VIO 는 12:22:22 부터 5 분 넘게 GOOD 이었다("arm 전 리셋 2회·정지 드리프트 0.45 m" 는 추락 뒤 재시도 때 값) → **VIO 게이트만으로는 1차 사고를 못 막았을 것**,
> 핵심은 비행 중 0.5 s 감시와 MAVSDK 지연 수정(v0.2.1). ② 사고 비행 콘솔(사용자 터미널): 12:27:48 이륙 → 12:27:51.65 파이프 기준 0.58 m REACHED → 12:27:53.67 파이프 U **+1.90**(VIO 복귀 리셋) →
> 12:27:55 킬 스위치 → 코드는 그 뒤로도 회전 단계를 계속, 12:28:05 지오펜스(가짜 데이터) → "착륙". ③ 12:28:33 재시도에서 킬 스위치로 1 s 만에 disarm 됐는데
> **코드가 외부 disarm 을 감지하지 못해 30 s 동안 시퀀스를 계속**(그 콘솔이 `122109_T1`) → v0.2.1 `_armed_monitor` 추가. ④ mavsdk_server 의 `[Debug] MAVLink:` 출력은
> 터미널에만 나오고 파일에 안 남는다 → 치트시트에 로그 리다이렉트 추가. 폴더 `122731_T1` 과 voxl-logger `log0003` 은 기체가 꺼져 있어 다음 접속 때 회수.


**타임라인** (기체 시각; 로그 `marker_drift_logs/20261110_122109_T1`, PX4 `sess179/log100.ulg`, voxl-logger `log0004`, 사본 `C:\Users\ericj\sim_logs\marker_drift\real_20261110\`)

| 시각 | 사건 | 근거 |
|---|---|---|
| 12:24:40 | 첫 실행이 `Waiting for mavsdk_server` 에서 대기(서버 미기동). 사용자 Ctrl+C/Ctrl+Z (프로세스 7090 정지 상태로 남음, 연결 안 됨) | console 122440 |
| 12:27:25 | 사용자가 `/home/mavsdk_server -p 50051 udp://:14551 &` 기동 | ps lstart |
| 12:27:57 → 12:28:07 | **VIO GOOD→BAD → 하드 리셋**(Quality 0, Features 0, "System not stationary at initialization") | open-vins 저널 |
| 12:28:15 → 12:28:23 | VIO BAD → 하드 리셋 (두 번째) | 저널 |
| 12:28:17~28 | 스크립트 연결·프리플라이트 통과: `is_local_position_ok` 5회 OK, MAVSDK NED 10샘플 범위 D 1.6 cm | console |
| 12:28:25 | VIO INITIAL→GOOD (리셋 후 2 s) | 저널 |
| 12:28:28~29 | voxl-logger 시작(hires_small_color·ov·px4·imu), 포즈 리더 2개 시작 | console |
| 12:28:30.7~34 | **지상 정지 상태에서 VIO 고도가 0.13 m/s 로 드리프트**(ov z 0.02→0.41, quality 15, features 0); 파이프 `fb_rel_u` 0→−0.45 | ov.csv, sync.csv |
| 12:28:33.6 | arm. 12:28:34.6 offboard 시작, 이륙 램프(0.25 m/s, 파이프 기준 1.05 m 위) | console |
| 12:28:35 | VIO GOOD→**BAD**. 12:28:35.9~39.0 PX4 `vehicle_visual_odometry.quality = −1`(무효 3.0 s) → **EKF 관성 추측항법** | 저널, ulog |
| 12:28:35.9~37.4 | EKF z 0 → −0.59 m 로 상승 후 "도달" 유지(목표 0.6). MAVSDK `mav_rel_u` ≈ −0.2~0 | ulog, sync |
| 12:28:39.0 | VIO 복귀(quality 74) 위치 z **−2.0 m** → EKF xy/z 리셋 카운터 +1, z −2.0 으로 점프 = **실제로는 천장 높이(≈2 m)까지 올라가 있었다** | ulog 2130.32 |
| 12:28:40~42 | 수평 이동 x 0.25→3.1 m(조종자 개입 구간으로 추정), 12:28:42.1 **킬 스위치** → 낙하(vz +2~3 m/s) → 책상 충돌 | ulog, "Disarmed by kill-switch" |
| 12:28:55~12:29:04 | 스크립트는 파이프 기준 16.4 s 타임아웃 → 착륙 명령·disarm 기록(이미 지상) | console |
| 12:31:13 | 정지 상태에서 VIO 또 하드 리셋. 이후 안정 | 저널 |

**직접 원인**: 이륙 순간 OpenVINS VIO 가 끊겼고(quality −1, 3.0 s), PX4 EKF 가 관성 추측항법으로 고도를 0.6 m 로 **과소 추정**한 채 위치 제어를 계속해 실제로는 약 2 m 까지 상승했다.
2026-11-04 사고(리부팅 2 분 뒤 이륙, VIO 3 s 무효, z −2.2 리셋)와 **같은 메커니즘**이다.

**기여 요인**
1. **arm 전 VIO 가 이미 불안정**했다. 12:27:57~12:28:25 사이 하드 리셋 2회, arm 8 s 전에 막 재초기화된 상태. "리부팅 직후 VIO" 와 같은 조건이 리셋으로 재현됐다.
   정지 상태에서 고도가 3 s 에 0.45 m 흘렀는데 어떤 검사도 이를 보지 않았다.
2. **프리플라이트가 VIO 품질을 보지 않는다.** `is_local_position_ok`(PX4 EKF 관점) 와 MAVSDK 위치 범위만 본다. EKF 는 VIO 가 끊겨도 추측항법 중엔 OK 를 유지한다.
   파이프(`px4_vehicle_local_position`, vision-hub) 와 MAVSDK 위치가 시작부터 0.39 m 어긋나 있었고 그 뒤 1 m 이상 벌어졌지만 비교하지 않았다.
3. **지오펜스가 EKF(MAVSDK) 위치만 본다.** EKF 가 "0.6 m 도달" 이라고 믿었으므로 수직 편차 1.5 m 는 걸리지 않았다. 파이프 쪽은 −1.2 m 까지 벌어졌지만 지오펜스 입력이 아니다.
   **[2차 사고 분석 뒤 추가]** 게다가 그 MAVSDK 값은 소비 지연으로 6~8 s 과거 값이었다(v0.2.1 항목).
4. **비행 중 VIO 품질 감시 없음.** `vehicle_visual_odometry.quality`/open-vins 상태가 −1/BAD 로 바뀌어도 코드가 알 수 없다.
5. 부하: 같은 시점에 voxl-logger(hires 30 fps jpg) 시작, 태그 검출기 15 Hz(오늘 skip 5→1), 포즈 리더 2개. VIO 저하와 시간이 겹치나 인과는 미확정(CPU 총 28 %, 카메라 30 fps 유지).
   VIO 가 정지 상태에서도 features 0 으로 떨어진 것은 **전방 카메라 시야의 특징점 부족**(현수막·벽) 가능성이 크다.
6. 절차: 이륙 전 VIO 안정 확인(손으로 들어 움직여 보기, quality>0 확인)을 2026-11-04 메모에 적어 두고도 체크리스트에 넣지 않았다. mavsdk_server 수동 기동도 치트시트에 없었다.

**기체 상태 (원격 점검, 12:31~12:40)**: disarm, ESC 4개 online·오류 0, 배터리 7.99 V, IMU 정상(수평), tracking_front/down **30 fps** 정상, VIO 정지 상태 quality 96·드리프트 없음(12:31:15 이후),
CPU 28 %·35 °C. 서비스 모두 active. **프롭·암·카메라 렌즈·배터리 외관은 사용자가 직접 확인 필요.** 정지된 프로세스 7090 과 mavsdk_server 는 그대로 둠(비행 불가 상태).

**대책 (제안, 승인 후 구현·SITL 검증 → 재배포)**
- P1 프리플라이트 **VIO 게이트**: PX4 `vehicle_visual_odometry.quality` ≥ 임계(예 50)·open-vins 리셋 없이 ≥ 30 s 경과·정지 상태 파이프 고도 드리프트 < 3 cm/3 s·파이프 vs MAVSDK 위치 차 < 0.1 m. 하나라도 실패하면 arm 하지 않는다.
- P2 **비행 중 VIO 감시**: quality < 0 또는 파이프 stale 0.5 s 이상 → 즉시 착륙(현재 고도 유지 후 하강). 추측항법 구간을 비행 코드가 메워 주지 않는다.
- P3 지오펜스에 **파이프 vs EKF 불일치**(수직 0.3 m / 수평 0.5 m, 0.5 s 지속) 조건 추가. 둘이 갈라지면 어느 쪽이 맞든 안전하지 않다.
- P4 이륙 램프를 EKF(MAVSDK) 기준으로도 상한(목표 + 0.3 m) 검사하고, 파이프 기준 기대 이동량이 명령 고도의 1.5 배를 넘으면(여기선 1.05 m vs 0.6) 이륙 전에 중단.
- P5 절차: 체크리스트에 "mavsdk_server 기동 확인", "VIO 품질·리셋 확인(저널 `QUALITY` 최근 60 s 에 BAD 없음)", "기체를 들어 움직인 뒤 포즈 복귀 확인" 추가. 이륙 지점의 전방 카메라 시야에 특징점(텍스처) 확보. 로거는 이륙 후 hover_ref 에서 시작하는 옵션 검토.
- P6 RC: 조종자 개입 절차를 "Position 모드 전환 → 하강" 으로 명문화(킬은 최후). 천장까지 1.4 m 여유에서 상승 속도 1 m/s 면 반응 시간이 1.4 s 뿐이다 → 실내 점검 비행은 고도 0.4 m, `--takeoff-speed` 0.15 로.

### 2026-11-10 (오후) — 사고 대책 구현 v0.2.0 (사용자 승인 "그렇게 진행해줘"; 비행 없음)
- **P1 VIO 게이트** (`vehicle.preflight_vio_gate`, arm 직전): ① open-vins 저널 최근 30 s 에 BAD/리셋 없음 ② 3 s 창에서 `voxl-inspect-vio -n ov` 품질 최소 ≥ 50,
  상태 전부 OKAY, 특징점 최소 ≥ 3 ③ 같은 창에서 reach 파이프·MAVSDK 위치 변동 ≤ 3 cm ④ 파이프 vs MAVSDK 차 ≤ 0.15/0.25 m. 실패 → `PreflightAbort`(exit 3, arm 안 함).
  사고 당시 값으로 대입하면 ①(리셋 2회) ②(features 0, quality 15) ③(0.45 m) 세 군데서 걸린다.
- **P2 비행 중 VIO 감시** (`_vio_monitor`): 품질 < 1 또는 상태 ≠ OKAY 가 0.5 s 지속 → 비상 착륙. VIO 상태 샘플 1 s 끊김 → 비상 착륙. reach 파이프 1 s 끊김 → 비상 착륙(기존엔 경고만).
  사고 당시 quality −1 은 3.0 s 지속 → 0.5 s 에 착륙 명령이 나갔을 것.
- **P3 추정기 불일치** (`_check_estimator_disagreement`, MAVSDK 모니터 안): 파이프 vs MAVSDK 상대 위치 수직 0.3 / 수평 0.5 m 초과가 0.5 s → 비상 착륙. 사고 때 offboard 1 s 뒤 0.58 m.
- **P4 이륙 거부** (`takeoff`): 지상에서 파이프/MAVSDK 상대 위치가 원점에서 0.15 m 이상 → 이륙하지 않고 착륙·disarm. 사고 때 파이프 U −0.45.
- 새 모듈 `s2flight/vio.py`(voxl-inspect-vio 파서·리더·이력·저널 조회), `sim.sim_vio_task`(SITL 은 품질 100 고정, `S2_SIM_VIO_FILE` 로 상실 주입),
  `run_sim_drift.sh VIO_FAIL_AT=<s>`, `tools_onboard/vio_check.py`(P5 손 점검 스크립트: 정지 20 s 통계 + 저널 + PX4 quality → PASS/FAIL).
- CLI: `--no-vio-gate`(권장 안 함) 외 임계값 전부 옵션(`--vio-min-quality` 등). 배너에 `VIO gate` 줄. sync.csv 에 `vio_quality/vio_features/vio_state/est_dv/est_dh` 열.
- 테스트: 파서·이력 5 + 통합 9(게이트 PASS/품질/특징점/정지 드리프트 거부, 비행 중 상실·불일치 비상 착륙, 이륙 거부, --no-vio-gate) → (결과는 §2 v0.2.0).
- 절차(P5/P6): §4 치트시트에 실기 순서 추가 — mavsdk_server 기동 → `vio_check.py` 정지 PASS → 들어 움직인 뒤 다시 PASS → 비행. 실내 점검 비행은 `--altitude 0.4 --takeoff-speed 0.15`.
  조종자 개입은 Position 모드 전환 후 하강, 킬은 최후.
- 기체 배포: v0.2.0(`s2flight/` 16 파일 + `marker_drift_flight.py`) + `vio_check.py` 배포 완료, 온보드 0.2.0 확인. 비행 없음.
- **기체 현재 위치에서 `vio_check.py --seconds 8` 실행 → FAIL**: quality 최소 17 / 평균 71, features 6, 정지 드리프트 1 cm, 저널 리셋 0, PX4 quality 96.
  정지 상태에서도 품질이 17 까지 떨어지는 위치(충돌 후 책상 근처, 시야 특징점 부족 추정) — 게이트가 거부할 상태. 이륙 지점은 시야에 텍스처가 있는 곳으로.

### 2026-11-10 (오후) — **2차 사고: v0.2.0 점검 비행, 이륙 직후 가짜 '추정기 불일치' 비상 착륙 → 착륙 중 앞-왼쪽으로 밀려 충돌**

**타임라인** (`marker_drift_logs/20261110_131409_T1`, PX4 `sess179/log102.ulg`)

| 시각 | 사건 |
|---|---|
| 13:14:26 | VIO 게이트 PASS(quality 94, features 19, 드리프트 1 cm) → arm |
| 13:14:27 | offboard, 이륙 램프 0.15 m/s → 목표 0.40 m |
| 13:14:28~29 | PX4 EKF 는 0.43 m 까지 정상 상승, reach 파이프도 +0.31→+0.40. **MAVSDK 텔레메트리는 0.00 에 고정** |
| 13:14:29.5 | `_check_estimator_disagreement`: 파이프 0.40 vs MAVSDK 0.00 → **비상 착륙** (가짜 불일치) |
| 13:14:29.8~31 | PX4 Land 중 VIO quality 60→4 붕괴, EV 위치 1 m 점프, 기체 앞-왼쪽 ~1 m 이동 → 충돌. 13:14:31 킬 |

**직접 원인: MAVSDK 텔레메트리 소비 지연.** 기체에서 실측: `position_velocity_ned` 도착 30 msg/s, 비행 코드 루프(메시지마다 `sleep(0.05)`)는 18.5 msg/s 소비
→ gRPC 버퍼에 밀려 **초당 0.41 s 씩 과거로** (6 s 측정에 2.46 s 지연). 이륙 시점 MAVSDK 값은 2~3 s 전 "지상" 값이었다.
v14 부터 같은 구조였고, **1차 사고 때도 MAVSDK 가 6~8 s 지연**돼 있었다(파이프 점프 12:28:41/44 가 MAVSDK 에 12:28:49/54 에 나타남) → 지오펜스가 침묵한 또 하나의 이유.
SITL 에서는 전송률·소비 속도가 달라 드러나지 않았다.

**환경 요인**: 착륙 1.3 s 동안 VIO 가 붕괴(프롭 바람·바닥·책상 주변 시야). 사용자의 **수동(Position) 비행도 같은 방향으로 밀림** → 로그 비교:
11-05 정상 비행에도 모터 비대칭 22 %·롤 1.5° 바이어스가 있었으나 드리프트 8 cm/51 s 였으므로 추력 비대칭은 원인이 아니다.
오늘 Position 비행에서 EKF 는 "오른쪽 1.1 m 이동" 이라 믿었는데 VIO 속도는 ~0 → VIO 위치가 흘러가고 제어기가 반대로 밀어 실제로 왼쪽 이동.
VIO 불안정(BAD/리셋)은 1차 충돌 전 12:22 부터 있었다 → **오늘 이 장소의 시야 문제**(흰 현수막 70 cm, 책상, 조명) 가 유력. 카메라 손상은 다른 장소에서 손 검증 뒤 판단.

**대책 (v0.2.1)**
- `_mavsdk_position_monitor` 가 메시지마다 쉬지 않고 전부 소화, 무거운 처리만 0.05 s 간격. `set_rate_position_velocity_ned/odometry(20)` 요청.
- `_mavsdk_lag_monitor`: `odometry.time_usec`(PX4 시각) 와 수신 시각 차의 최소값을 기준선으로 지연을 잰다. 지연 > 0.3 s → MAVSDK 기반 판정(지오펜스·불일치) 보류·이벤트, 게이트 거부; offboard 중 > 2.0 s → 비상 착륙.
- 지오펜스 입력을 실시간 reach 파이프(명령 프레임)로 전환(없으면 신선한 MAVSDK). 불일치 판정은 MAVSDK 가 신선할 때만.
- `capture_initial_mavsdk_position` 도 쉬지 않고 소화(시간 창).
- 절차: 비행 전 **장소 검증** — 후보 지점에서 `vio_check.py --seconds 30` 을 기체를 들고 천천히 움직이며 실행, 이동 중에도 quality ≥ 50·features ≥ 10 유지되는 곳만 이륙 지점. 모든 곳에서 무너지면 카메라 점검.

### 2026-11-12 (오후) — **수동 비행 좌측 밀림·좌회전 재현 → EKF 자이로 바이어스 -3.5 °/s 확정, v0.2.2**
- 사용자 보고: 기체를 켜고 수동(Position)으로 띄우니 또 좌측으로 밀리고 천천히 좌회전. 비행 없음(코드·분석만). 기체 IP .18 로 변경.
- 로그 `sess180/log100.ulg`(7.3 MB): **arm = 부팅 31 s** (이전 비행들은 114·811·2124 s). POSCTL 38 s, 이륙 34 s.
  - 원시 자이로 z 적분 -2.3°(기체는 거의 안 돌았다) vs **EKF yaw +116.5°(+3.5 °/s)**. EKF `gyro_bias[2]` 첫 샘플 -3.04 → -3.46 °/s (11-05 +0.05, 11-10 -0.13, 11-04 -0.03).
    `vehicle_angular_velocity` z 평균 +3.34 °/s = 원시 -0.10 - 바이어스(-3.46): 컨트롤러는 가만히 있는 기체가 돈다고 믿었다.
  - EKF 위치 변화 5.4 m vs VIO 0.3 m, `ev_hpos` 이노베이션 최대 2.7 m → 틀어진 헤딩으로 EV 속도를 융합해 위치가 달아나고 기체가 그걸 쫓아 밀림. 조종자는 오른쪽 스틱 74 % 시간 — 보상 입력.
  - 모터 스프레드 7 %(11-05 10 %), 자세 평균 roll -0.17° → **기체 비대칭 아님**. 캘리브레이션 파라미터(CAL_GYRO0_*OFF, CAL_ACC0_*OFF) 11-04~11-12 동일 → 저장된 캘 문제 아님.
  - `EKF2_EV_CTRL = 7`(pos+vel+hgt, **yaw 비트 꺼짐**), `EKF2_MAG_TYPE 5` → `cs_yaw_align 0`, `cs_ev_yaw 0`: EKF 헤딩은 자이로 적분뿐이라 바이어스 오차가 그대로 헤딩 드리프트가 된다.
    ModalAI 실내 VIO 프리셋은 `EKF2_EV_CTRL 15`(yaw 융합 포함) — 포럼/문서 근거 §3. 변경은 기체 설정이라 승인 필요(미적용).
- 실시간 확인(부팅 10 분, 지상 정지, cs_in_air 0): `estimator_sensor_bias.gyro_bias[2] = -0.0616 rad/s (-3.53 °/s)`, 10 s 동안 heading -1.015 → -0.383 rad (**+3.6 °/s**). VIO yaw 는 멈춰 있음.
  부팅 30 분 뒤 재확인: 바이어스 +0.08 °/s, 헤딩 드리프트 0 — 그 사이 기체가 옮겨져(VIO 원점 리셋) EV 속도 융합으로 바이어스가 재학습된 것으로 보임. **움직여야만 관측되는 상태라 신뢰 불가 → 게이트로 막는다.**
- 가설: 전원을 켠 뒤 OpenVINS 초기화(부팅 +4 s GOOD)·EKF 초기화 중에 기체를 들고 옮기거나 돌린 뒤 바로 arm → EV 위치/속도 이노베이션이 자이로 바이어스 상태로 흡수. 로그가 arm 부터라 arm 전 31 s 는 미확인.
- 조치: **v0.2.2**(패치노트) + 절차 변경: 전원은 이륙 지점에 둔 채 켜고 2~3 분 손대지 않는다, `vio_check.py` 가 헤딩·바이어스·uptime 을 판정. v0.2.2 기체 배포·`vio_check` 가 실제 고장 상태에서 FAIL(-3.54 °/s) 확인.
- 회수: `20261110_122731_T1`(1차 사고 실제 콘솔), voxl-logger `log0003` csv(ov·imu·local_position; 이미지 제외) → `sim_logs/marker_drift/real_20261110/`. 오늘 로그 → `real_20261112/px4/sess180_log100.ulg`.
- 분석 스크립트(스크래치): `ulog_yawdrift.py`(원시 자이로 vs EKF yaw·바이어스·EV 비교), `ulog_params.py`(캘/EKF 파라미터·플래그 비교).

### 2026-11-12 (저녁) — **3차 사고: 수동 비행 천장 충돌** (EKF2_EV_CTRL 15 적용 뒤; 자동 비행 아님)
- 흐름: `EKF2_EV_CTRL 15` 적용·재부팅 → vio_check 3회 FAIL(특징점 6, 정지 위치 드리프트 0.07~0.20 m, 출력 15~25 Hz) → 사용자가 수동(Position) 비행 →
  ① `sess181/log101.ulg`(80 s): 이륙 뒤 10 s 간 VIO 유효/무효 깜빡임 → EKF 속도 ±0.16 m/s 요동("토일렛 보울링"), VIO 재초기화로 EKF yaw 10.8° 리셋, 11 s 뒤 회복.
  비행 중 "Preflight Fail: High Accelerometer Bias" 3회 — EKF 가속도계 z 바이어스 0.19 → 0.395 m/s²(한계 0.4). 재부팅 없이 ②로.
  ② `sess181/log102.ulg`(17 s): 이륙 2 s 전 VIO quality 73→22→-1 → EV 융합 중단·추측항법, z 리셋(-0.40→+0.04). 이륙(거리 센서 0→1.5→2.5 m)하는 동안 EKF 는
  **vz +0.4 → +6.5 m/s '하강'**(가속도계 바이어스 0.4 m/s² 적분) → Position 제어가 추력 상승. +2.5 s `invalid setpoints` → PX4 failsafe **blind land** 가 '추락 중' 추정 위에서
  하강을 시도해 추력을 더 올림 → 천장(2.5 m) 충돌, +13 s 킬 스위치. 종료 시 EKF z +23 m.
- 원인: (근본) 이륙 지점 전방 무늬 없음 → 특징점 6 → 이륙 순간 VIO 상실(저널 6 분간 GOOD→BAD→재초기화 3회). (증폭) 직전 비행이 남긴 가속도계 바이어스 0.4 m/s² 를 재부팅 없이 가져감
  (11-10 2차 사고와 같은 함정; 자동 비행이었다면 v0.2.1 `px4_msg` 게이트가 'Accelerometer Bias' 경고로 arm 거부). 수동 비행에는 게이트가 없다.
- 기체: 모터 편차·자세 정상. `EKF2_EV_CTRL 15` 는 정상 동작(①에서 헤딩이 VIO 에 재정렬). 사고 로그 → `sim_logs/marker_drift/real_20261112/px4/{manual2_latest,ceiling_crash}.ulg`.
- 조치: **v0.2.3** — 게이트에 EKF 가속도계 바이어스 ≤ 0.3 m/s², VIO 특징점 ≥ 8(종전 3), VIO 출력률 ≥ 20 Hz 추가; vio_check 동일. 절차: vio_check FAIL 이면 **수동 비행도 하지 않는다**,
  이륙 지점에 무늬(태그판·상자·바닥 종이) 확보, 어떤 비행이든 끝나면 다음 비행 전 vio_check 재실행(바이어스·경고 확인), 충돌·경고 뒤 재부팅.
- 분석 스크립트(스크래치): `ulog_bowl.py`(이륙 뒤 1 s 타임라인: EKF/EV yaw·품질·위치·속도·이노베이션), `ulog_ceiling.py`(z·vz·EV z·거리 센서·추력·setpoint 0.25 s).

### (다음 항목을 여기에 날짜로 추가)
- [x] 기체: `tag_locations.conf` 0.20, `skip_n_frames` 1, voxl-tag-detector 기동 (2026-11-10 승인·적용)
- [x] 손 검출 시험 (2026-11-10): x 오른쪽 +, σ_yaw 0.19° → `--align-tol 1.0`
- [x] v0.2.1 → v0.2.2 배포 (2026-11-12), `122731_T1`·`log0003` csv 회수
- [ ] 3차 사고 뒤: 전원 끄고 외관 점검(프롭·암·상판·렌즈) → **이륙 지점 무늬 보강**(태그 10 판 1~2 m 앞, 바닥 종이) → 전원 투입 3 분 정치 → `vio_check.py` PASS(features ≥ 8, rate ≥ 20 Hz, pos drift ≤ 3 cm) → v0.2.3 배포 확인 → 사용자 "시작" → 0.4 m 점검 비행 → 태그 모드
- [x] `EKF2_EV_CTRL 7 → 15` (2026-11-12 05:30 UTC, 사용자 승인 "15로 변경해줘"). 변경 전 7 확인, 백업 `/data/px4/param/parameters.bak.20261112-053038`, `px4-param set/save` 뒤 재부팅.
  근거: 기체는 `indoor_vio_missing_gps.params` 프리셋과 EV_CTRL 한 항목만 달랐다(인계 덤프 10-27 부터 7). 되돌리기: `px4-param set EKF2_EV_CTRL 7; px4-param save; reboot`.
  적용 뒤 확인 항목: `px4-listener estimator_status_flags` 의 `cs_ev_yaw True`, `cs_yaw_align True`; vio_check 의 EKF-VIO yaw 오프셋이 0 근처.
- [ ] 실험장: 태그 0~9 부착 마무리(좌상단 꼭짓점 기준 500 mm, §4.1 표), 통로 B 폭(140 cm?), 상단열 C(같은 ID 0~9)
- [ ] 태그 크기 20 cm 확인

---

## 2. 패치노트

### v0.2.3 — 가속도계 바이어스·특징점·출력률 게이트 (2026-11-12 저녁, 3차 사고 뒤)
- `vio.py`: `parse_px4_accel_bias`, `read_px4_sensor_bias`(자이로+가속도계; `read_px4_gyro_bias` 는 래퍼), `VioHistory.summarize` 에 `rate_hz`.
- 게이트 ⑨ 확장: 가속도계 바이어스 축별 ≤ `accel_bias_max_ms2`(0.3) ; ② 특징점 하한 `vio_min_features` 3 → **8** ; ⑩ VIO 출력률 ≥ `vio_min_rate_hz`(20, 실기만). CLI `--accel-bias-max --vio-min-rate-hz`.
- `vio_check.py`: 가속도계 바이어스(`--max-accel-bias` 0.3)·출력률(`--min-rate` 20)·특징점 8 판정, FAIL 문구에 조치(무늬 보강) 명시.
- 테스트 200/200 (+5: 가속도계 파서·rate_hz·특징점 6 거부·가속도계 0.395 거부·출력률 저하 거부). 가짜 기체 `vio_every_n` 로 출력률 저하 흉내.
**검증**: 단위테스트 **200/200**. SITL A12 `20261112_154211_i2` 게이트 PASS → 완료(출력률 검사는 실기만). 기체 배포됨(충돌 뒤 재부팅 전).

### v0.2.2 — EKF 헤딩 드리프트 방어 (2026-11-12 오후, 수동 비행 좌측 밀림 원인 확정 뒤)
- `vio.py`: `voxl-inspect-vio` RPY 파싱(`rpy`, PX4 EV 와 같은 부호 — 실기에서 -9.9° = -9.9° 확인), `VioHistory` yaw 보관·`yaw_range`·`latest_yaw`, `yaw_range_deg`(wrap), `read_uptime_s`, `parse_px4_gyro_bias`/`read_px4_gyro_bias`(`px4-listener estimator_sensor_bias`).
- `vehicle.py`: 연결 직후부터 `attitude_euler` 상시 스트림(실기; SITL 은 `sim_pose_task` 가 공급) → `mav_yaw_deg`. `capture_initial_yaw` 는 스트림 값을 샘플링(이중 구독 회피).
  게이트 추가 ⑦ 창 동안 지상 정지 EKF yaw 변동 ≤ `static_yaw_drift_deg`(1.0°) ⑧ `/proc/uptime` ≥ `min_uptime_s`(120 s) ⑨ EKF 자이로 바이어스 축별 ≤ `gyro_bias_max_dps`(1.0).
  비행 중 `_check_heading_disagreement`: offboard 시작 시 (EKF yaw − VIO yaw) 를 기준으로 잡고 `yaw_disagree_deg`(10°) 이상 `yaw_disagree_s`(0.5 s) 지속 → EMERGENCY LAND "HEADING DISAGREEMENT". sync.csv `yaw_ekf_vio_dev_deg`.
- `config.py`: `--min-uptime-sec --static-yaw-drift-deg --gyro-bias-max-dps --yaw-disagree-deg --yaw-disagree-sec`.
- `sim.py`: `sim_vio_task` 가 rpy(yaw = EKF yaw + 오버라이드 `yaw_offset_deg`) 를 낸다 → `VIO_FAIL_JSON='{"yaw_offset_deg": 30}'` 로 헤딩 발산 시나리오.
- `tools_onboard/vio_check.py`: uptime·EKF 자이로 바이어스·창 동안 EKF 헤딩 드리프트(`--max-heading-drift` 0.3 °/s)·VIO yaw 범위·EKF-VIO 오프셋 출력, FAIL 시 "재부팅 후 2~3 분" 안내.
- 테스트 194/194 (+13: 파서 rpy·yaw_range·VioHistory yaw·바이어스 파서 2·uptime·게이트 헤딩/uptime/바이어스 거부·비행 중 발산 착륙·기준 기록·정상 통과). 가짜 기체는 arm 전 setpoint 를 따르지 않도록 수정(실기와 동일).
**v0.2.2b (같은 날 저녁)**: SITL V6 에서 게이트 EKF yaw 샘플이 0개인 채 PASS → 헤딩 검사가 무음으로 꺼져 있었다(초기 헤딩 캡처가 `attitude_euler` 를 따로 구독·종료하면
뒤의 `sim_pose_task` 구독이 데이터를 못 받는 경쟁). 수정: ① yaw 샘플 0개면 게이트 FAIL("no EKF yaw samples") ② SITL 은 `sim_pose_task` 를 캡처 전에 띄우고, 캡처는 상시 공급(실기: 연결 직후 스트림)을
기다렸다 샘플링(별도 구독 없음). `run_sim_drift.sh` 의 `VIO_FAIL_JSON` 커스텀 JSON 에 `}` 가 하나 더 붙던 셸 기본값 버그 수정. 테스트 195/195.
**검증**: 단위테스트 **195/195**. 실기(비행 없음): 고장 상태에서 `vio_check` FAIL(-3.54 °/s), 정상화 뒤 heading 0.0004 rad/8 s; `EKF2_EV_CTRL 15` 뒤 EKF-VIO yaw 오프셋 -0.2°.
SITL(v0.2.2b): A11 `20261112_144213_i2` 게이트 PASS(yaw 30 샘플) → 완료; V7 `20261112_144233_i3` offboard +15 s 에 VIO yaw 를 +30° 떼어 놓자
**EMERGENCY LAND "HEADING DISAGREEMENT: EKF yaw drifted -30.0 deg vs VIO"** → 착륙. (V5·V6 는 각각 JSON 버그·yaw 샘플 0 으로 무효 — 트러블슈팅 참고.)

### v0.2.1 — MAVSDK 텔레메트리 지연 수정 (2026-11-10 오후, 2차 사고 대책)

**수정**: `vehicle._mavsdk_position_monitor` 소비 루프의 `sleep(0.05)` 제거(전부 소화, 처리만 20 Hz 제한). `capture_initial_mavsdk_position` 시간 창 방식.
**추가**: `_request_mav_rates`(PX4 전송률 20 Hz 요청), `_mavsdk_lag_monitor`(odometry time_usec 기준 지연, `mav_lag_s`), `mav_fresh()`, 게이트 조건 ⑤ 지연 ≤ 0.3 s,
지오펜스 입력 = reach 파이프 우선, 불일치 판정은 MAVSDK 신선할 때만. config `mav_rate_hz` 20, `mav_lag_warn_s` 0.3, `mav_lag_abort_s` 2.0, `mav_lag_calib_s` 2.0 (+CLI).
요약/sync 에 `mav_lag_s`. 테스트 6항목(60 Hz 스트림 추종, 지연 게이트 거부, 비행 중 지연 → 착륙 등).
**추가 2**: `_armed_monitor` — `telemetry.armed()` 를 감시해 우리가 arm 한 상태에서 PX4 가 disarm(킬 스위치·failsafe)이면 즉시 시퀀스 중단·이벤트 `external_disarm`
(1차 사고 재시도에서 킬 뒤 30 s 계속 진행한 결함). `vio_reader_task` 헤더 줄 오인(error_codes) 수정. 테스트 4항목 추가.
**추가 3**: `_status_text_monitor` — PX4 STATUSTEXT 를 모아 이벤트 `px4_msg` 로 기록, 게이트 조건 ⑥ 최근 `px4_msg_quiet_s`(30 s) 안에 'Preflight Fail'·'Accelerometer Bias'·
'estimate error'·'Attitude failure'·'Flight termination'·'Kill switch' 등이 있으면 arm 거부(추락 뒤 재부팅 없이 날았던 상황 대응, `--px4-msg-quiet-sec 0` 으로 끔). 테스트 3항목.
**구조 수정 (발견: SITL 에서 MAVSDK 위치 모니터가 한 번도 갱신되지 않았음)**: `sim_pose_task` 가 같은 `position_velocity_ned` 스트림을 먼저 구독하면 두 번째 구독(모니터)은
데이터를 못 받는다 → v0.1.4·v0.2.0 의 SITL 검증에서 지오펜스·불일치 감시는 **실제로 돌지 않았다**(sync.csv `mav_updates 0`). 스트림 소비(`_mavsdk_position_stream`, 실기만)와
주기 처리(`_mavsdk_position_monitor`, 0.05 s: 기록·지오펜스·불일치)를 분리하고, SITL 은 `sim_pose_task` 가 `update_mav_position` 을 호출. 초기 위치 캡처 뒤 구독 `aclose()`.

**검증**: 단위테스트 **181/181**. SITL(v0.2.1 최종): A9 `20261110_225503_i2` 게이트 PASS → 왕복 STOPPED → 완료, **MAVSDK 위치 모니터 갱신 확인**(mav_updates 515, mav_u 가 fb_u 추종, est_dv 최대 0.003 m);
V4 `20261110_225518_i3` offboard +20 s VIO 상실 → 0.5 s 뒤 EMERGENCY LAND → 착륙. 기체 배포는 다음 전원 투입 때(기체 꺼짐).

### v0.2.0 — 이륙 사고 대책: VIO 게이트·감시·추정기 불일치·이륙 거부 (2026-11-10 오후)

**추가**
- `s2flight/vio.py`: `parse_voxl_inspect_vio_line`, `vio_reader_task`(voxl-inspect-vio -n <pipe> 자식 프로세스), `VioHistory`(불량 지속·창 요약), `journal_recent_vio_events`.
- `s2flight/vehicle.py`: `PreflightAbort`, `update_vio`, `preflight_vio_gate`(P1/P4 정지 드리프트), `_vio_monitor`(P2), `_check_estimator_disagreement`(P3), `takeoff` 거부(P4),
  `offboard_active` 플래그, 요약에 `vio_gate/vio_samples/vio_bad_episodes`.
- `s2flight/config.py`: `vio_gate`, `vio_pipe`, `vio_min_quality` 50, `vio_min_features` 3, `vio_gate_window_s` 3, `vio_quiet_s` 30, `vio_static_drift_m` 0.03,
  `est_disagree_v_m` 0.3, `est_disagree_h_m` 0.5, `est_disagree_s` 0.5, `vio_inflight_min_quality` 1, `vio_bad_abort_s` 0.5, `vio_stale_abort_s` 1.0,
  `feedback_stale_abort_s` 1.0, `takeoff_fb_max_offset_m` 0.15 + CLI 그룹.
- `s2flight/sim.py` `sim_vio_task`; `sim/run_sim_drift.sh` `VIO_FAIL_AT/VIO_FAIL_JSON/VIO_RECOVER_AFTER`; `summarize_run.py` safety 줄.
- `flight/tools_onboard/vio_check.py`; `marker_drift_flight.py` PreflightAbort 처리(exit 3)·배너.

**검증**: 단위테스트 **166/166** (신규 14: 파서 3·이력 2·게이트 PASS/품질/특징점/정지 드리프트 거부·비행 중 상실·불일치·이륙 거부·--no-vio-gate).

| SITL | 인스턴스 | 시나리오 | 결과 |
|---|---|---|---|
| A7 | i2 (`20261110_215801_i2`) | 정상 | 게이트 PASS(quality 100, features 50, 드리프트 3 mm) → 왕복 STOPPED → 착륙, completed. VIO 샘플 722, 불량 0 |
| V2 | i3 (`20261110_215940_i3`) | offboard +20 s(hover_ref, 0.6 m)에 VIO 상실 주입 | 0.1 s 뒤 `[vio] BAD` → **0.5 s 뒤 EMERGENCY LAND** → 3 s 뒤 착륙(gt z 0.03). abort_reason 'VIO LOST' |
| G1 | i4 (`20261110_215831_i4`) | 시작부터 VIO 상실 | 게이트 FAIL(quality −1, not_ok 34/34) → **PREFLIGHT ABORT, arm 없음**, exit 3 |
| V1 | i3 (`20261110_215816_i3`) | (주입 타이밍 실수: 게이트 창에 상실) | 게이트가 잡아 arm 안 함 — 게이트 창 중 상실도 거부됨을 확인 |

사고 당시 조건을 대입하면: arm 전 저널 리셋 2회(①) + features 0(②) + 3 s 0.45 m 드리프트(③) 로 게이트 거부. 설령 통과했더라도 이륙 1.3 s 뒤 quality −1 → 0.5 s 뒤 착륙 명령,
파이프 vs EKF 0.58 m 불일치로도 0.5 s 뒤 착륙. 추정기 자체의 추측항법 오차는 비행 코드가 고칠 수 없으므로 '빨리 내려앉기' 가 대책의 전부다.

### v0.1.6 — 기체 ASCII 로케일 대응 (2026-11-10 밤, 실기 첫 실행에서 발견)

**수정**
- `s2flight/util.py` `force_utf8_stdio()`: stdout/stderr 가 UTF-8 이 아니면 UTF-8(errors=replace) 로 감싼다. `marker_drift_flight.py main()` 첫 줄에서 호출.
- `s2flight/recorder.py` CSV 열기에 `encoding="utf-8"` (한글 이벤트 문자열 저장 시 충돌 방지).
- `s2flight/__init__.py` `__version__` 0.1.6 (그동안 0.1.0 으로 방치됨).
- 테스트 2항목(ASCII stdout 감싸기, UTF-8 에서 no-op).

### v0.1.5 — 통로 구간 지표 (2026-11-10 밤)

**추가**: `sim/marker_drift_gz/pilot_stats.py` `--metric seg_out|seg_back`, `--seg-from/--seg-to`(기본 2→6): drift_log.csv 에서 두 태그 위치에 가장 가까운
프레임의 err_along 차. 비행별 표·CSV 에 `seg_out`, `seg_back` 열. 비행 코드 변경 없음.

### v0.1.4 — 태그 0~9 (2026-11-10 밤)

**변경**
- `marker_drift_flight.py` `--end-tag` 기본 10 → **9**, `s2flight/drift.py` `BannerLayout` 기본 end_tag 9. 길이·상한은 end_tag 에서 자동 계산(4.5 / 5.5 m).
- `sim/make_banner_world.py` `--ids`/`--upper-ids` 기본 `0-9`. `lab_default.yaml`·`lab_upper.yaml` id 10 제거. 문서 다수(§1 항목 참조).
- 테스트는 합성 배치(end_tag 명시)라 변경 없음.

**SITL**: 태그 0~9 2열 월드 재설치 후 A 1회 → (아래 표)

| 회차 | 인스턴스 | 결과 |
|---|---|---|
| A5 | i2 (`20261110_202332_i2`) | completed. ALIGNED 0.06°, center ok, out STOPPED **태그 9**(truth 4.547), back STOPPED 태그 0, settled drift_along −0.006, 상한 5.5 m |
| A6 | i2 (`20261110_202626_i2`) | 태그 중심 z 0.65 월드, 고도 0.6. completed, ALIGNED 0.03°, out STOPPED 태그 9, back STOPPED 태그 0, settled drift_along −0.005. 자기 열 |y_cam| 평균 0.08 m(이상 중 최대 0.61) → 0.7 필터 안 |

### v0.1.3 — offboard 시작 재시도, 파일럿 통계 (2026-11-10 밤)

**추가**
- `s2flight/config.py` `offboard_start_retries = 3`, `offboard_retry_sec = 0.5` (CLI `--offboard-start-retries`, `--offboard-retry-sec`).
  `vehicle.start_monitors_and_offboard`: `offboard.start()` 가 `OffboardError` 로 거부되면 스트림을 유지한 채 재시도, 이벤트 `offboard_retry` 기록.
  소진 시 기존과 같이 비상정지. `--offboard-start-retries 0` 이면 v14 동작.
- `sim/marker_drift_gz/pilot_stats.py`: 여러 결과 폴더 → 비행별 표, 조건별 mean/s/n_req(16(s/Δ)²), Welch t, `--csv`. 표준 라이브러리.
- 테스트 4항목(재시도 성공·이벤트·소진·0회) → 148/148.

**수정**
- `summarize_run.py`: `initial_yaw=None(None)` 표시 버그 (잘못된 키 경로) 수정.

**SITL**: 파일럿 9회(§1 표) + A2 재실행 통과. 재시도 패치 적용 후 실행에서는 재시도가 발생하지 않았다(정상 경로 불변 확인).

### v0.1.2 — 2열 동일 ID 대응 (2026-11-10 저녁)

**추가**
- `s2flight/align.py` `DEFAULT_MAX_VERTICAL_M = 0.7`, `measure_alignment(..., max_vertical_m)`: 카메라 기준 상하 |y| 가 이보다 큰 검출 배제.
  `s2flight/drift.py` `estimate_frame`/`DriftMonitor`, `align_stage.AlignParams` 에 같은 인자. CLI `--align-max-vertical`(기본 0.7, 음수면 끔).
- `sim/make_banner_world.py`: `--upper-ids` 기본 `0-10`, 링크 이름 `tag_NN_rR`, layout 키 `NN_r1` 로 같은 ID 두 열 생성 가능.
- `analysis/marker_drift/layouts/lab_upper.yaml` 신설(상단열 ID 0~10, z 2.0). `lab_default.yaml` 은 하단열만.
- 테스트 5항목 추가(다른 열만 → no_tags, 혼합 → 자기 열만, 끄기, estimate_frame 2건) → 144/144.

**문서**: docs/09 §4.1(상단열 ID·설치 기준점 표), brief, sim README, `marker_drift_flight.py` 도움말(상단열 20/30 문구 제거).

**SITL 검증 (2열 월드, 상단열 ID 0~10)**

| 회차 | 인스턴스 | 조건 | 결과 |
|---|---|---|---|
| A3 | i2 (`20261110_192012_i2`) | 0.6 m, 하단열 | completed. ALIGNED 0.08°, center ok, out/back STOPPED(태그 10/0), settled drift_along −0.002 / +0.034 m. 원시 검출 1227 중 상단열(|y| 1.40) 6건 → 필터로 배제, 판정 영향 없음 |
| C1 | i3 (`20261110_192032_i3`) | 2.0 m, 상단열 | completed. ALIGNED 0.04°, center ok, out/back STOPPED, settled drift_along −0.009 / −0.001 m. 원시 검출 1419 중 하단열(|y| 최대 1.48, 이착륙 중 0.98) 43건 → 배제. `--start/end-tag` 기본값 그대로 |

### v0.1.1 — 서버 첫 실행 (2026-11-10)

**수정**
- `sim/marker_drift_gz/install_banner_world.sh`: `gz sdf -k` 월드 검사가 `model://drift_banner` 를 못 찾아 중단되던 문제. 검사 중에만
  `GZ_SIM_RESOURCE_PATH`/`SDF_PATH` 에 `<out>/models` 와 PX4 `Tools/simulation/gz/models` 를 준다. 복사 로직은 그대로.

**추가**
- `sim/marker_drift_gz/run_sim_drift.sh`: `INJECT='<json>'` 환경변수. drone_sim_gz `vio_relay.py` 의 오차 주입 파일(`/tmp/wk_inject_N.json`)에
  그대로 써 주고 산출물 폴더에 `inject.json` 으로 남긴다. 기본 `{}`(주입 없음). 키: `rot_deg`, `drift_n`/`drift_e`(m/s), `step_e`/`step_at`,
  `noise_pos_m`/`noise_vel_m_s`/`noise_seed`.
- `sim/marker_drift_gz/summarize_run.py`: 결과 폴더 요약(주입값, 완료/그 자리 착륙, 정렬·센터링, 레그 정지 사유, `legs_settled` 드리프트,
  drift_log 레그별 err 통계, 주목 이벤트, 진실값 범위). 표준 라이브러리만 쓴다.
- `sim/marker_drift_gz/README.md`: 검증 상태, 오류 주입 절. `worklog/2026-11-10.md` 신규.

**변경 없음** — `flight/s2flight/*`, `marker_drift_flight.py` 는 서버에서 수정 없이 그대로 통과했다. v14·drone_sim_gz 파일도 불변.

**검증** 인스턴스 2·3, 8회 비행 (정상 2 + 주입 6). 모두 `exception: -`, 안전장치 발동 없음.

| 회차 | 인자 / 주입 | 결과 |
|---|---|---|
| T1 | `--no-align --stop-mode distance --distance 0.5` | completed, 전 단계 OK |
| A1 | `--condition A --run 1` | ALIGNED → center ok → out STOPPED(태그 10) → back STOPPED(태그 0) → 착륙. settled drift −0.004 m |
| D1 | `{"drift_n":-0.02}` | STOPPED 양쪽. 기록된 drift_along 기울기 −0.020 m/s(settled −0.56 / −0.81). 센터링 3회 미수렴 → 경고 후 진행 |
| R1 | `{"rot_deg":3}` | STOPPED. 진실 경로 직선, 드리프트 ≈ 0 (프레임 회전은 body 기준 이동에 무영향) |
| S1 | `{"step_e":-0.3,"step_at":25}` | STOPPED. 실제 이격 0.70→1.00, drift_standoff −0.30 |
| N1 | `{"noise_pos_m":0.02,"noise_vel_m_s":0.05}` | STOPPED. err sd 2 cm. preland 도달 4 s |
| M1 | `--end-tag 11 --max-extra 0.5` | out MAX_DIST 6.00 m → 그 자리 착륙(`landed_in_place=True`) |
| J1 | `{"step_e":-0.8,"step_at":25}` | STOPPED. 실제 이격 0.70→1.50(순간 1.70), drift_standoff −0.80. 지오펜스 무반응 |

### v0.1.0 — 패키지 신설 (2026-11-09, 커밋 `f5f4763`)
- `flight/s2flight/` 12 모듈, `marker_drift_flight.py`, `sim/marker_drift_gz/` 5 스크립트, `tools/deploy_pkg.sh`, `analysis/test_s2flight.py` 139 항목.
  상세는 `flight/s2flight/README.md`, `worklog/2026-11-09.md`.

---

## 3. 트러블슈팅

형식: 증상 → 원인 → 조치. 서버·네트워크 항목은 이 실험 외에도 재사용된다.

### 네트워크 / 서버 접속

**Tailscale: 서버 노드가 보이는데 ssh 포함 모든 TCP·ICMP 가 time out**
- 증상: `tailscale ping` 은 pong(직접 연결 12 ms), `tailscale status` 에 서버 뜸. `Test-NetConnection :22` False, ssh "Connection timed out".
- 원인: 공유(share)를 수락한 계정이 상대 테일넷 ACL 에서 허용되지 않음. 여기서는 GitHub 계정으로 로그인한 상태에서 받은 공유였고,
  관리자가 구글 계정으로 다시 초대하자 바로 열렸다. 노드가 "보이는 것"(공유 성립)과 "패킷이 통과하는 것"(ACL)은 별개.
- 조치: 관리자에게 초대받은 계정을 확인하고 그 계정으로 Tailscale 재로그인. 여전히 막히면 ACL 에 `autogroup:shared` 허용 규칙 또는 서버 ufw
  `allow in on tailscale0`. 공유받은 쪽은 **22번만** 열려 있을 수 있다(여기서 그랬다).

**ssh 가 되는데 `Permission denied (publickey,password)`**
- 원인: 비대화형(BatchMode) 이라 비밀번호 입력 불가, 키 미등록.
- 조치: 비밀번호로 한 번 들어간 서버 셸에서 이 PC 공개키를 `~/.ssh/authorized_keys` 에 추가. (PowerShell용 `type ... | ssh` 한 줄을 **서버 셸**에서
  치면 `$env:` 가 bash 에 안 먹고 alias 도 없어 실패한다 — 어느 쪽 터미널인지 확인.)

**VNC 5901 이 Tailscale 로 안 닿음**
- 원인: 위와 같은 ACL(22번만 허용).
- 조치: `ssh -f -N -L 5901:localhost:5901 dronesim_tailscale` 후 뷰어에서 `localhost:5901`. 서버 쪽은 TurboVNC `:1`(XFCE, `~/.vnc/passwd` 비밀번호).
  재부팅하면 터널을 다시 연다. 비밀번호는 서버 관리자.

**`! ssh host` 를 Claude Code 세션 안에서 치면 "Pseudo-terminal will not be allocated"**
- 원인: 세션 셸은 TTY 가 아니라 대화형 로그인 불가.
- 조치: 대화형은 별도 PowerShell 창. 세션 안에서는 `ssh host '명령'` 비대화형만.

### 시뮬 설치 / 실행

**`install_banner_world.sh`: `Unable to find uri[model://drift_banner]`, 검사 실패로 중단**
- 원인: `gz sdf -k` 가 월드의 `<include><uri>model://…` 을 해석하려면 모델 검색 경로가 필요한데 주지 않았다. 설치 전 검사이므로 PX4 트리에도
  아직 모델이 없었다.
- 조치: 검사 중 `GZ_SIM_RESOURCE_PATH=<out>/models:<PX4>/Tools/simulation/gz/models` (+ `SDF_PATH`). 수정 후 `sdf ok: world`.

**tag_detect 로그에 `Error, more than one new minima found` 가 반복**
- 원인: AprilTag 라이브러리(pupil_apriltags 내부 C)가 자세 추정에서 찍는 stderr 경고. 검출·좌표에는 영향 없음.
- 조치: 무시. 검출 수는 `tags.csv` 의 `n_det` 또는 `summarize_run.py` 의 `tags:` 줄로 본다.

**`tags.csv` 첫 행 `true_x..true_t` 가 nan**
- 원인: 첫 이미지가 첫 odometry 보다 먼저 도착. 그 뒤 행은 전부 채워진다.
- 조치: 분석 때 첫 행 제외. (`sim_tag_detect.py` 수정 불필요.)

**콘솔에 `MAV_STALE!` 이 계속 붙음 (시뮬)**
- 원인: 보조 파이프(mav)는 실기에서 MAVSDK 텔레메트리가 채우는데 `--sim` 에서는 같은 소스라 갱신 표시가 안 된다. 경고 문자열일 뿐 판정·지오펜스에
  쓰이지 않는다(`vehicle.py` `mav_stale_warn_sec`).
- 조치: 무시. 실기에서 뜨면 MAVSDK 연결 점검.

**`summary.json` 에 `initial_yaw_deg: None`**
- 원인: 요약 도구가 최상위 키로 읽는데 실제 값은 다른 위치(콘솔 `Heading ref` 줄에는 정상 출력). 도구 쪽 표시 문제.
- 조치: `summarize_run.py` 가 `sequence` 가 아니라 `vehicle.initial_yaw_deg` 를 읽도록 수정 (v0.1.3, 2026-11-10 밤). 이제 `initial_yaw=359.999(current heading at arm ...)` 로 표시.

**Windows 에서 `python3 - <<EOF` 로 한글 포함 스크립트를 넘기면 `unicodeescape` / cp949 오류**
- 원인: stdin 이 cp949 로 읽힌다. 경로의 `\U` 도 이스케이프로 해석.
- 조치: 스크립트는 파일로 저장해 실행하거나 Write 도구로 직접 쓴다. 서버(리눅스)에서는 문제 없음.

**SITL 에서 MAVSDK 위치 모니터가 전혀 갱신되지 않음 (`mav_updates 0`, 2026-11-10 저녁 발견)**
- 증상: 모든 SITL sync.csv 에서 `mav_rel_*` 0 고정, `mav_updates` 0. 실기에서는 갱신됨.
- 원인: SITL 에서는 `sim_pose_task` 가 `position_velocity_ned` 를 먼저 구독한다. MAVSDK-Python 0.12 + mavsdk_server 1.4 에서 같은 스트림의 두 번째 구독이 데이터를 받지 못했다.
  → 지오펜스·불일치 감시가 SITL 검증에서 꺼진 채 "통과" 로 보였다.
- 조치: v0.2.1 구조 수정(위). 검증 기준에 **sync.csv `mav_updates` > 0, `mav_rel_u` 가 `fb_rel_u` 를 따라감** 을 추가. 같은 스트림을 두 곳에서 구독하지 말 것.

**게이트가 'yaw drift … (0 samples)' 로 PASS — 헤딩 검사가 꺼진 채 통과 (2026-11-12 SITL V6)**
- 증상: mission.log 게이트 줄에 `(0 samples)`, `[heading] EKF-VIO yaw offset …` 줄 없음, sync.csv 에 `yaw_ekf_vio_dev_deg` 열 없음. 같은 코드로 바로 전 실행(V5·A10)은 30 샘플.
- 원인: `capture_initial_yaw` 가 `attitude_euler` 를 따로 구독하고 `aclose()` 한 뒤, 뒤에 시작한 `sim_pose_task` 의 같은 스트림 구독이 데이터를 못 받는 경우가 있다(MAVSDK-Python 0.12 이중 구독 문제의 변형, 타이밍 의존).
- 조치: v0.2.2b — 샘플 0개면 게이트 FAIL, SITL 포즈 태스크를 캡처 전에 시작, 캡처는 상시 공급 값을 샘플링. 규칙: **감시 입력이 비어 있으면 통과가 아니라 실패**로 다룬다.

**`VIO_FAIL_JSON` 으로 넣은 커스텀 JSON 이 안 먹음 (`{"yaw_offset_deg": 30}}`, 2026-11-12 SITL V5)**
- 원인: `echo "${VIO_FAIL_JSON:-{\"quality\": …0}}"` — 기본값 안의 `}` 가 확장을 먼저 닫아, 변수가 설정돼 있으면 뒤 `}` 가 그대로 붙는다. 기본 JSON 일 때만 우연히 맞았다.
- 조치: 기본값을 변수에 한 번 대입(`VIO_FAIL_JSON=${VIO_FAIL_JSON:-'…'}`) 뒤 `echo "$VIO_FAIL_JSON"`. 주입 결과는 산출물 폴더 `sim_vio.json` 으로 확인한다.

**이륙하자마자 멈추지 않고 상승 → 천장 (2026-11-04, 11-10 1차, 11-12 3차 — 세 번 다 같은 기전)**
- 기전: 이륙 순간 VIO 상실 → EKF 가 EV 높이 융합을 끊고 관성 추측항법 → 가속도계 바이어스 오차가 그대로 적분돼 '하강 중' 으로 추정 → Position/blind-land 제어가 추력을 올림.
  바이어스가 이미 커져 있으면(직전 비행 'High Accelerometer Bias' 경고, 재부팅 안 함) 수 초 만에 수 m/s 가 된다(11-12: 10 s 에 4 m/s).
- 판별: ulog `vehicle_local_position.vz` 가 이륙 중 양수(하강)로 커지는데 `distance_sensor.current_distance` 는 증가; `vehicle_visual_odometry.quality` -1; `estimator_status_flags.cs_inertial_dead_reckoning` 1.
- 예방: 이륙 지점 특징점 ≥ 8·출력 30 Hz(vio_check), 비행 사이 바이어스 확인(accel ≤ 0.3 m/s²), 경고·충돌 뒤 재부팅. 조종자: 상승이 멈추지 않으면 즉시 **Manual(Stabilized) 로 내리고 스로틀을 줄인다** — Position 모드는 틀린 추정을 믿고 계속 올린다.

**지상 정지인데 EKF 헤딩이 돈다 / 수동·Position 비행에서 한쪽으로 밀리며 천천히 회전 (2026-11-12)**
- 증상: 기체는 가만히 있는데 `px4-listener vehicle_local_position` heading 이 초당 수 ° 변함; 비행하면 한쪽(이번엔 왼쪽)으로 밀리고 같은 쪽으로 천천히 yaw, 조종자가 계속 보상.
- 원인: EKF2 자이로 z 바이어스 상태가 틀어짐(-3.5 °/s). 이 기체는 `EKF2_EV_CTRL 7`(yaw 비트 꺼짐)·자력계 없음이라 헤딩 보정원이 없고, 정지 상태에선 바이어스가 관측되지 않아 스스로 안 낫는다. 부팅 31 s 만에 arm 한 비행에서 발생.
- 판별: `px4-listener estimator_sensor_bias -n 1` 의 `gyro_bias` 세 번째 값 ×57.3 (°/s) 이 1 넘으면 비정상; heading 두 번 읽어 변화율. `vio_check.py` 가 둘 다 본다. 모터 스프레드·자세 평균은 정상이므로 기체 탓으로 보지 말 것.
- 조치: 재부팅 → 이륙 지점에 둔 채 2~3 분 손대지 않음 → `vio_check.py` PASS 뒤 비행. 코드 게이트 ⑦⑧⑨(v0.2.2) 가 거부한다. 근본 대책 후보: `EKF2_EV_CTRL 15`(VIO yaw 융합, ModalAI 실내 프리셋; 포럼: "yaw estimate will track VIO unless the Yaw bit in EKF2_EV_CTRL is off and EKF2_MAG_TYPE 0/5") — 승인 뒤 적용.
- 근거: https://docs.modalai.com/flying-with-vio/ , https://forum.modalai.com/post/12384 , https://forum.modalai.com/post/22352

**MAVSDK 위치가 몇 초 전 값이다 / 파이프와 MAVSDK 가 갈라진다 (2026-11-10 1·2차 사고)**
- 증상: `_sync.csv` 에서 `mav_rel_*` 가 `fb_rel_*` 를 수 초 늦게 따라감. 실기 측정: 30 msg/s 입력, `sleep(0.05)` 루프 18.5 msg/s 소비, 지연 +0.41 s/s.
- 원인: MAVSDK-Python 의 async 스트림은 소비가 느리면 버퍼링(드롭 아님). 메시지마다 sleep 하는 루프는 입력률보다 느리면 반드시 밀린다.
- 조치: v0.2.1. 스트림 루프엔 sleep 을 두지 말 것. 지연은 `odometry.time_usec` 로 잰다(`mav_lag_s`). SITL 은 시뮬 시계라 지연 감시 생략.

**arm 직후 `Offboard FAILED: NO_SETPOINT_SET` → 비상정지 (2026-11-10, SITL 인스턴스 4, 서버 load 28)**
- 증상: priming 3 s 동안 20 Hz setpoint 스트림이 돌았는데도 `offboard.start()` 가 NO_SETPOINT_SET. 기체는 지상 arm 상태에서 착륙 명령·disarm 으로 정리됨.
- 원인: 과부하로 lockstep 시뮬·mavsdk_server 가 지연돼 PX4 가 최근 setpoint 를 못 본 것으로 판단(병렬 3 인스턴스 + 타 사용자 Gazebo). 같은 코드가
  다른 8회에서는 문제없었다. 실기에서는 mavsdk_server 가 기체 안에서 돌아 가능성이 낮지만 0 은 아니다.
- 조치: v0.1.3 재시도 3회(0.5 s 간격). 실행 중 재발하면 콘솔의 `Offboard start rejected (...) retry k/3` 로 보인다. 재시도도 소진되면 비상정지.
  재실행 결과(instance 2) 는 재시도 없이 통과.

**실기 첫 실행: `UnicodeEncodeError: 'ascii' codec can't encode character '\u2014'` 로 배너 출력에서 중단 (2026-11-10, arm 전)**
- 증상: `ssh starling2_lab "python3 marker_drift_flight.py ..."` 직후 Traceback. 모터 안 돌았음.
- 원인: 비대화형 ssh 셀은 LANG 이 비어 기체 python3 의 stdout 이 ASCII (`locale` LC_CTYPE=POSIX). 콘솔·로그의 '—'·한글이 모두 예외.
  SITL 컨테이너는 UTF-8 이라 드러나지 않았다. 비행 중 log() 에서 터졌다면 예외 → 비상정지 경로였을 것.
- 조치: v0.1.6 `force_utf8_stdio()` + CSV utf-8. 실행 명령에도 `PYTHONIOENCODING=utf-8` 을 같이 준다(이중 안전).

### 비행 로직에서 확인된 동작 (버그 아님, 알아 둘 것)

**큰 등속 드리프트(2 cm/s) 에서 센터링 3회 미수렴**
- 동작: `!!! centering not converged … continuing — reference uses measured tag position`. 태그는 보이므로 착륙하지 않고 진행. 결과(정지·드리프트 기록)는
  유효했다.
- 선택지: `--center-tol 0.08`, `--no-center`. 실기 호버 드리프트는 이보다 작을 것으로 기대.

**횡방향 VIO 점프 0.8 m 가 안전장치에 걸리지 않음**
- 동작: 추적 오차 지오펜스 상한이 `--max-horizontal-dev 2.5` 라 0.8 m 는 정상 추적으로 처리. 기체는 실제로 0.8 m 옆으로 이동 후 비행 계속.
- 권고: 통로 실험(조건 B)에서는 `--max-horizontal-dev 1.0` 으로 조인다. 태그 이격 vs 자기 이격 차이로 멈추는 조건은 미구현(필요 시 `DriftMonitor` 에
  predicate 추가).

**잡음 2 cm 에서 preland 도달이 느림**
- 동작: `h_tol 0.02` 가 잡음과 같은 크기라 `stable_time 0.8 s` 를 채우는 데 4 s. TIMEOUT(15 s) 뒤에도 현재 위치 고정 후 진행하므로 멈추진 않는다.
- 선택지: 실기 VIO 잡음이 크면 `--h-tol 0.04`.

**VIO 프레임 회전(`rot_deg`)이 경로에 영향 없음**
- 이유: 정렬 뒤 이동 방향을 **현재 body yaw 기준**으로 잡기 때문에 추정 프레임과 추정 yaw 가 같이 돌면 상쇄된다. v14 처럼 VIO 0° 를 경로 heading 으로
  쓰는 방식에서만 횡드리프트가 생긴다(`drone_sim_gz` 의 `rot_deg` 설명이 그 경우). 실기에서 "전원 켠 방향으로 돌아가는" 문제가 이 코드에서 사라진 것과
  같은 이유.

---

## 4. 치트시트

### 4a. 실기 비행 절차 (2026-11-10 사고 뒤 확정; 순서를 건너뛰지 않는다)

```bash
# 0) 전원은 **이륙 지점에 둔 채** 켠다. 켠 뒤 2~3 분 손대지 않는다(옮기거나 돌리지 않는다). 추락·킬·충격 뒤엔 반드시 재부팅. (2026-11-12: 부팅 31 s arm → EKF 자이로 바이어스 -3.5 °/s)
ssh starling2_lab                                              # IP 바뀌면 메모리 절차로 스윕 (hostkey IIh6B2Siw…)
pgrep -af mavsdk_server || (/home/mavsdk_server -p 50051 udp://:14551 > /home/root/mavsdk_server.log 2>&1 &)   # 재부팅마다 수동. 출력은 파일로(터미널에만 남던 MAVLink 메시지 보존)
systemctl is-active voxl-tag-detector || systemctl start voxl-tag-detector  # 태그 모드일 때만. conf: id 0~10 size 0.20, skip 1, down 끔
cd /home/root && PYTHONIOENCODING=utf-8 python3 vio_check.py --seconds 20   # ① 바닥에 둔 채 PASS 필요 (uptime ≥ 120 s, EKF 자이로 바이어스 < 1 °/s, 헤딩 드리프트 < 0.3 °/s 포함)
#   → 기체를 손으로 천천히 들어 30 cm 움직였다 제자리에 둔다
cd /home/root && PYTHONIOENCODING=utf-8 python3 vio_check.py --seconds 20   # ② 다시 PASS (복귀 확인). FAIL 이면 30~60 s 뒤 재시도
#   조종기: 위(Offboard). 이상하면 가운데(Position) 로 내리고 스틱으로 하강. 킬은 최후.
PYTHONIOENCODING=utf-8 python3 marker_drift_flight.py --no-align --stop-mode distance --distance 0.5 \\
    --altitude 0.4 --takeoff-speed 0.15 --hover-ref 3 --hover-far 2 --hover-return 2 --condition T -y   # 실내 점검 비행 (낮게·느리게)
#   비행 코드가 arm 직전 VIO 게이트를 다시 본다(⑦ 정지 헤딩 ⑧ uptime ⑨ 자이로 바이어스 포함). 'vio-gate FAIL' 이면 arm 하지 않고 exit 3.
#   비행 중 EKF-VIO yaw 가 10° 넘게 벌어지면 'HEADING DISAGREEMENT' 로 스스로 착륙한다.
```
이륙 지점: 전방 카메라 시야에 텍스처(가구·마커·벽 무늬)가 있어야 한다. 흰 벽·현수막만 보이면 특징점 0 → 게이트 FAIL.
**추락·킬 스위치·강한 충격 뒤에는 반드시 기체 재부팅** → 3~5 분 대기 → vio_check (EKF 가속도계 바이어스·VIO 상태가 남아 다음 비행을 밀어낸다, 2026-11-10 2차 사고).
장소 검증: 후보 지점에서 `vio_check.py --seconds 30` 을 기체를 들고 50 cm 쯤 천천히 움직이며 실행 — 이동 중에도 quality ≥ 50·features ≥ 10 유지되는 곳만 이륙 지점.

### 4b. 서버 실행 (이 PC 에서)

```bash
ssh dronesim_tailscale                                   # 키 로그인
SIM_HOST=dronesim_tailscale ./sim/marker_drift_gz/sync_to_server.sh   # 작업 중 파일 밀어넣기 (커밋 없이)
ssh dronesim_tailscale 'cd ~/starling_sim/marker_drift && INSTANCE=2 sim/run_sim_drift.sh --condition A --run 2'
ssh dronesim_tailscale 'cd ~/starling_sim/marker_drift && python3 sim/summarize_run.py logs/<시각>_i2'
scp -r dronesim_tailscale:~/starling_sim/marker_drift/logs/<시각>_i2 C:/Users/ericj/sim_logs/marker_drift/
ssh -f -N -L 5901:localhost:5901 dronesim_tailscale      # 화면 보기 (RealVNC Viewer → localhost:5901)
ssh dronesim_tailscale 'docker exec -d -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 -e GZ_PARTITION=starling_2 starling-gz gz sim -g'  # 실행 중 인스턴스 2 에 Gazebo GUI 붙이기
ssh dronesim_tailscale 'cd ~/starling_sim/marker_drift && python3 sim/pilot_stats.py logs/20261110_19*_i*'   # 여러 비행 통계 (조건별 s, n)
ssh dronesim_tailscale 'cd ~/starling_sim/marker_drift && VIO_FAIL_AT=25 INSTANCE=3 sim/run_sim_drift.sh --condition V --run 1'   # 비행 중 VIO 상실 → 비상 착륙 검증
ssh dronesim_tailscale 'cd ~/starling_sim/marker_drift && VIO_FAIL_AT=15 VIO_FAIL_JSON="{\"yaw_offset_deg\": 30}" INSTANCE=3 sim/run_sim_drift.sh --condition V --run 2'   # EKF-VIO 헤딩 발산 → HEADING DISAGREEMENT 착륙 검증
```
인스턴스 2·3 병렬 가능(각 ~3.5 분). 끝난 뒤 `pgrep -af "px4|gz sim"` 가 비어 있어야 정리된 것. 서버의 `~/Drone_project` 클론은 뒤처져 있으니 건드리지 않는다.
