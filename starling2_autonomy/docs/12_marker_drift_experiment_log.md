# 12. 마커 드리프트 실험 — 진행 기록 (단일 문서)

이 실험의 **모든 진행 과정을 이 문서 하나에** 시간순으로 기록한다 (2026-11-10 결정: 실험 하나 = 기록 문서 하나).
설계서 `docs/09_marker_drift_experiment_plan.md` 와 랩미팅 요약 `_brief.md` 는 "무엇을 할 것인가", 이 문서는 "실제로 무엇이 일어났는가".
새 항목은 **§1 진행 기록** 끝에 날짜로 추가하고, 코드가 바뀌면 §2 패치노트, 문제를 겪으면 §3 트러블슈팅에도 한 줄씩 남긴다.

**현재 상태 (2026-11-10):** 코드 완성 · 서버 SITL 선검증 통과(정상 2 + 오류 주입 6) · 실기 비행 전 단계.
남은 관문: 기체 태그 검출기 설정(승인) → 손 검출 부호 확인 → 짧은 호버 비행 → 본실험.

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

### (다음 항목을 여기에 날짜로 추가)
- [ ] 기체: `tag_locations.conf` 0.20, `skip_n_frames`, voxl-tag-detector 기동 (승인 후)
- [ ] 손 검출 시험: x_cam 부호(오른쪽 +)·yaw 오차 부호 확인, 30 s 정지 σ → `--align-tol`
- [ ] `tools/deploy_pkg.sh` 배포 → 실기 `--no-align --stop-mode distance --distance 0.5` → 태그 모드
- [ ] 실험장: 태그 0~10 제작, 통로 B 폭(140 cm?), 상단열 C
- [ ] 태그 크기 20 cm 확인

---

## 2. 패치노트

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
- 조치: 콘솔 로그/`events.csv` 의 `initial_yaw` 이벤트로 확인. (도구 수정 예정 — 사소.)

**Windows 에서 `python3 - <<EOF` 로 한글 포함 스크립트를 넘기면 `unicodeescape` / cp949 오류**
- 원인: stdin 이 cp949 로 읽힌다. 경로의 `\U` 도 이스케이프로 해석.
- 조치: 스크립트는 파일로 저장해 실행하거나 Write 도구로 직접 쓴다. 서버(리눅스)에서는 문제 없음.

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

## 4. 서버 실행 치트시트 (이 PC 에서)

```bash
ssh dronesim_tailscale                                   # 키 로그인
SIM_HOST=dronesim_tailscale ./sim/marker_drift_gz/sync_to_server.sh   # 작업 중 파일 밀어넣기 (커밋 없이)
ssh dronesim_tailscale 'cd ~/starling_sim/marker_drift && INSTANCE=2 sim/run_sim_drift.sh --condition A --run 2'
ssh dronesim_tailscale 'cd ~/starling_sim/marker_drift && python3 sim/summarize_run.py logs/<시각>_i2'
scp -r dronesim_tailscale:~/starling_sim/marker_drift/logs/<시각>_i2 C:/Users/ericj/sim_logs/marker_drift/
ssh -f -N -L 5901:localhost:5901 dronesim_tailscale      # 화면 보기 (RealVNC Viewer → localhost:5901)
```
인스턴스 2·3 병렬 가능(각 ~3.5 분). 끝난 뒤 `pgrep -af "px4|gz sim"` 가 비어 있어야 정리된 것. 서버의 `~/Drone_project` 클론은 뒤처져 있으니 건드리지 않는다.
