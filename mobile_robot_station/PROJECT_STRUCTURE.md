# PROJECT_STRUCTURE — mobile_robot_station

애그테크연구실 드론 프로젝트의 **이동형 드론 스테이션 + 딸기 적재용 모바일 로봇 자율주행** 모듈.
메카넘 휠 myAGV(Raspberry Pi 4B, ROS Noetic) 기반. 실행/운용 방법은 [`README.md`](README.md)·[`GUIDE.md`](GUIDE.md)·[`HOW_TO_RUN.md`](HOW_TO_RUN.md), 진행 현황은 [`ROADMAP.md`](ROADMAP.md) 참고. 이 문서는 **구조·코드 역할·데이터 흐름**만 정리한다.

## 하드웨어 / 소프트웨어 스택
- 보드 Raspberry Pi 4B · 구동 메카넘 휠 4 · MCU myAGV(`/dev/ttyAMA2`, 115200)
- 센서 YDLidar X2(`/dev/ttyAMA0`) · 라즈베리파이 카메라(`/dev/video0`)
- OS Ubuntu 20.04 + ROS Noetic

## 디렉토리 구조와 역할
| 경로 | 역할 |
|---|---|
| `scripts/*.sh` | 운용 진입점(셸). 매핑·주행·텔레오프·상태·정지 등 단계별 실행 래퍼 |
| `python/` | 주행 로직 노드. 아래 "코드 역할" 참고 |
| `config/` | 파라미터. `navigation_overrides.yaml`(현행 오버라이드), `waypoints.yaml`(목표점), `line_calib.json`(라인 색), `mecanum_dwa_params.yaml`(레거시, 미사용) |
| `launch/` | `navigation.launch`(벤더 TrajectoryPlannerROS + AMCL 튜닝), `auto_explore.launch` |
| `maps/` | gmapping 산출 맵(`map_YYYYMMDD_HHMMSS.pgm/.yaml`) + `current_map.*`(최신 맵 상대 심볼릭 링크) |
| `worklog/` | 날짜별 작업 기록(가장 상세한 이력 원본) |
| `logs/` | 런타임 로그(gitignore) |

### 코드 역할 (`python/`)
- **맵 주행(move_base/DWA 계열, 장애물 회피 O)**: `waypoint_ui.py`(터치 UI 버튼+원점 초기화), `nav_goal_sender.py`, `move_relative.py`, `patrol_navigator.py`
- **move_base 우회(데드밴드 회피, 장애물 회피 X)**: `go_simple.py`(AMCL 위치 기반 회전→직진)
- **매핑/조작**: `map_teleop.py`(단일키 드라이버), `map_drive.py`, `move_direct.py`
- **라인추종/카메라**: `line_follow.py`, `line_calibrate.py`, `corridor_follow.py`, `camera_obstacle.py`
- **보조**: `save_waypoint.py`, `set_initialpose.py`, `robot_status.py`, `lidar_power_daemon.py`, `indoor_run*.py`

## 데이터 흐름
```
YDLidar(/scan) + 휠오도(/odom)
        │
   [매핑] gmapping ──▶ maps/*.pgm,yaml ──▶ current_map 링크
        │
   [주행] AMCL(/amcl_pose, 맵↔오도 정합) ──▶ move_base
                                             (global: NavfnROS / local: TrajectoryPlannerROS = 벤더 diff-drive식)
        │
      /cmd_vel(Twist) ──▶ myAGV MCU ──▶ 메카넘 모터
```
- 웨이포인트 주행: `waypoint_ui.py` 버튼 → `/move_base_simple/goal` 또는 액션 → move_base가 경로계획·추종.
- 초기위치: UI `⌂ 원점 초기화` 또는 `set_initialpose.py` → `/initialpose` 발행(AMCL 미설정 시 주행 실패).

## 외부 의존성 (이 저장소에 미포함, 기기 로컬)
- `~/myagv_ros` — **벤더 공식 ROS 패키지**(github.com/elephantrobotics/myagv_ros). `myagv_odometry`·`myagv_navigation`·`ydlidar_ros_driver`·`myagv_teleop`. 로컬 플래너 3종(TrajectoryPlannerROS 기본/DWA/TEB) 제공. 현재 주행은 벤더 `TrajectoryPlannerROS`(holonomic_robot:false) 사용.
- `~/pymycobot` — Elephant Robotics SDK(MCU 명령: 전압·auto_report 등).

## 경로 주의
이 모듈은 기존에 `/home/er/autonomous_nav`에 있었고 스크립트/런치 다수가 그 절대경로를 참조한다. 저장소로 이동 후 **`/home/er/autonomous_nav` → `mobile_robot_station` 심볼릭 링크**로 호환을 유지한다(같은 기기 한정). 다른 기기로 clone 시 이 링크 재생성 필요.
