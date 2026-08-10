# HANDOFF — 인수인계 (다음 담당자 먼저 읽기)

이 문서는 **처음 맡는 사람이 가장 먼저 읽는 진입점**이다. 상세 내용은 각 문서를 가리키기만 하고, 여기서는 **어디서 시작하고 / 지금 무엇을 하며 / 무엇을 조심할지**만 정리한다.

- **최종 인수인계 시점**: 2026-08-10 (직전 담당자까지의 작업은 [`worklog/`](worklog/)에 날짜별로 있음)
- **프로젝트**: 애그테크연구실 드론 프로젝트의 **이동형 드론 스테이션 + 딸기 적재용 모바일 로봇(myAGV) 자율주행**. 메카넘 휠, Raspberry Pi 4B, ROS Noetic.

---

## 1. 문서 지도 (읽는 순서)
1. **이 문서(HANDOFF.md)** — 전체 그림과 함정
2. [`PROJECT_STRUCTURE.md`](PROJECT_STRUCTURE.md) — 폴더 구조·코드 역할·데이터 흐름
3. [`ROADMAP.md`](ROADMAP.md) — ✅완료 / 🔴할 일 (당장 할 일은 여기 **P1**)
4. [`README.md`](README.md) · [`GUIDE.md`](GUIDE.md) · [`HOW_TO_RUN.md`](HOW_TO_RUN.md) — 실제 실행/운용 방법(매핑·주행·명령어)
5. [`worklog/`](worklog/) — 날짜별 상세 이력(왜 그렇게 됐는지의 근거 원본). **07-13**과 **08-10**을 특히 볼 것.

---

## 2. 지금 상태 한눈에
- **동작하는 것**: gmapping 매핑 → AMCL → move_base → 터치 UI 웨이포인트 주행. **첫 목표 주행 성공**. 최신 맵 `maps/map_20260713_111502`, 웨이포인트 2개(Room1, 예냉실).
- **막힌 지점(다음 담당자 본업)**: **AMCL 위치추정 드리프트.** 첫 목표는 되는데 2번째부터 엉뚱한 경로/도달 실패. 이동 후 정합이 느슨(≤15cm 50%)하고 AMCL이 과확신. **아직 "확정 진단"까지 못 감**(마지막 run은 스택 조기 종료로 미측정).

## 3. 당장 할 일 (ROADMAP P1) — 여기서 시작
스택을 **켠 채로** 진단한다(측정 전에 `stop_all.sh` 하지 말 것 — 그래서 지난번 측정을 놓쳤다):
1. `scripts/robot_ui_launch.sh` → "준비됨" 뜬 뒤 UI `⌂ 원점 초기화`
2. **목표 하나(예냉실)만** 전송
3. 주행 중/직후, **스택 살아있는 상태에서** 측정:
   - 라이브 정합(≤15cm 비율)
   - `rostopic echo -n1 /move_base/GlobalPlanner/plan` 끝점이 실제 목표와 맞는지
   - `/amcl_pose` vs 목표 좌표 거리
4. 드리프트로 확정되면 ROADMAP **P2** 순서대로: `escape_vel -0.0→-0.1` → `odom_alpha1`↑/파티클↑ → 목표 간 재초기화 → **AprilTag 재국소화**(근본 해결).

> ⚠️ **유실된 도구**: 위 정합 측정에 쓰던 `loc_check.py`(및 `map_quality.py`, `scan_vs_map.py`)는 이전 세션의 임시 파일이라 **지금 저장소에 없다.** 재작성해야 하며, 재작성 시 scratchpad가 아니라 **추적되는 위치(예: `python/` 또는 새 `tools/`)에 두어 커밋**할 것.
> - `loc_check.py` 로직: tf2로 라이브 `/scan`을 `map` 프레임에 투영 → 정적맵 벽까지 거리의 중앙값/≤15cm 비율 + `/amcl_pose` 공분산. 잘 맞으면 중앙값 ~5cm·≤15cm 90%+, 드리프트면 급락.

---

## 4. 반드시 알아야 할 함정 (모르면 시간 날림)

1. **경로 구조 = 심볼릭 링크.** 실제 파일은 `~/Drone_project/mobile_robot_station/`. `~/autonomous_nav`는 이곳을 가리키는 **심볼릭 링크**다. 스크립트/런치 다수가 `/home/er/autonomous_nav` 절대경로를 하드코딩하므로 로봇 명령은 예전처럼 `~/autonomous_nav/...`로 쓰면 된다.
   - **다른 기기/계정에서 clone 하면 이 링크가 없어 로봇이 안 된다** → clone 후 `ln -s ~/Drone_project/mobile_robot_station ~/autonomous_nav` 재생성 필수(경로가 정확히 `/home/er/...`여야 함).

2. **메카넘 y축(횡이동)을 자율주행에 다시 켜지 말 것.** 벤더는 일부러 diff-drive(`holonomic_robot:false`)로 운용한다. 과거 y축을 켰다가 저속 다축 mix가 **모터 데드밴드**에 걸려 "버튼 눌러도 안 움직임"이 났고, 벤더 `TrajectoryPlannerROS`로 되돌려 해결했다. (근거: worklog 07-13)

3. **"도착 시 waypoint 좌표로 AMCL 리셋"은 무효(자기참조 함정).** 도착 판정 자체가 드리프트된 AMCL 기준이라, 리셋해도 실제 오차가 안 준다. 근본 해결은 **외부 기준(AprilTag/마커, 360° 라이다)** 뿐. (근거: worklog 07-13)

4. **`/Voltage`(배터리)·오도가 안 나오면** MCU `auto_report`가 OFF일 수 있다 → pymycobot로 `set_auto_report_state(1)`. (상세: `README.md`/메모리, worklog 07-13)

5. **RViz는 Pi4에서 무겁다** → 초기위치는 UI `⌂ 원점 초기화` 버튼 또는 `scripts/set_initialpose.sh`(CLI)로. RViz 단독 실행은 roscore 선행 필수.

6. **진단 순서**(주행 안 될 때): `/move_base/status` 확인 → status 4면 배터리 아님(계획 실패) → `/amcl_pose`가 원점 근처면 초기위치 미설정 → 특정 먼 목표만 실패하면 맵/좁은 길목 → 그래도면 데드밴드/드리프트.

---

## 5. 개발 / git 워크플로우
- 저장소: `agtechresearch/Drone_project` (모노레포). 이 프로젝트는 `mobile_robot_station/` 컴포넌트.
- 일상 흐름: `cd ~/Drone_project` → 수정 → `git add …` → `git commit` → **`git push origin main`** (팀 관례상 PR 없이 main 직접. 큰 변경은 브랜치+PR 권장).
- 인증: 현재 이 Pi에는 직전 담당자 계정의 **SSH 키**가 설정돼 있어 로그인 없이 push된다.
  - ⚠️ **담당자가 실제로 바뀌면**: 본인 GitHub 계정으로 커밋 귀속을 남기려면 (a) 본인 SSH 키를 `~/.ssh`에 두고 GitHub 계정에 등록, (b) `git config --local user.name/user.email`을 본인 것으로 변경할 것. 안 바꾸면 직전 담당자 이름으로 커밋된다.
- "정리"/"save" 규칙: 작업 끝에 `worklog/YYYY-MM-DD.md`에 (수행 작업 / 주요 수치 / 결론 / 다음 할 일) 기록 후 커밋.

## 6. 환경 요약
| 항목 | 값 |
|---|---|
| 보드/OS | Raspberry Pi 4B, Ubuntu 20.04, ROS Noetic |
| 구동 | 메카넘 휠 4 · MCU `/dev/ttyAMA2` (115200) |
| 센서 | YDLidar X2 `/dev/ttyAMA0` · 카메라 `/dev/video0` |
| 벤더 ROS 패키지(저장소 밖, 기기 로컬) | `~/myagv_ros` (elephantrobotics/myagv_ros) |
| SDK(저장소 밖) | `~/pymycobot` (MCU 명령·전압) |
| 비상 정지 | `bash ~/autonomous_nav/scripts/stop_all.sh` |
