# myAGV 자율주행 프로젝트

메카넘 휠 기반 myAGV (Raspberry Pi 4B) 자율주행 시스템

## 하드웨어 구성

| 구성요소 | 세부 사항 |
|---------|---------|
| 보드 | Raspberry Pi 4B |
| 구동 | 메카넘 휠 4개 |
| LiDAR | YDLidar X2 (`/dev/ttyAMA0`, 115200 baud) |
| 카메라 | 라즈베리파이 카메라 (`/dev/video0`) |
| OS | Ubuntu 20.04 + ROS Noetic |

## 기존 ROS 패키지 (myagv_ros)

- `myagv_odometry` — 휠 오도메트리 + IMU EKF
- `myagv_navigation` — gmapping SLAM + AMCL + move_base
- `ydlidar_ros_driver` — YDLidar X2 드라이버
- `myagv_teleop` — 키보드 원격 조종

## 자율주행 단계

```
[1단계] SLAM 지도 작성 → [2단계] 지도 저장 → [3단계] 자율 네비게이션
```

---

## 터미널 구성 (tmux 권장)

자율주행은 여러 터미널(또는 tmux 창)이 필요합니다.

```
터미널 1: roscore
터미널 2: 오도메트리 + 라이다 (myagv_active.launch)
터미널 3: SLAM 또는 네비게이션 launch
터미널 4: 텔레오퍼레이션 (지도 작성 시)
터미널 5: 자율주행 Python 스크립트
```

---

## 1단계: SLAM 지도 작성

**터미널 1** — roscore 시작:
```bash
source /opt/ros/noetic/setup.bash
roscore
```

**터미널 2** — 오도메트리 + YDLidar 시작:
```bash
source ~/myagv_ros/devel/setup.bash
roslaunch myagv_odometry myagv_active.launch
```

**터미널 3** — SLAM (gmapping) 시작:
```bash
bash ~/autonomous_nav/scripts/1_slam_mapping.sh
```

**터미널 4** — 키보드로 로봇 조종하며 지도 완성:
```bash
bash ~/autonomous_nav/scripts/start_teleop.sh
```

> RViz에서 지도가 완성되면 2단계로 이동

---

## 2단계: 지도 저장

**터미널 5** — SLAM이 실행 중인 상태에서:
```bash
bash ~/autonomous_nav/scripts/2_save_map.sh
```

저장 위치: `~/autonomous_nav/maps/map_YYYYMMDD_HHMMSS.yaml`  
자동으로 `maps/current_map.yaml` 심볼릭 링크 생성

---

## 3단계: 자율주행 네비게이션

**터미널 1** — roscore (계속 실행)

**터미널 2** — 오도메트리 (계속 실행)

**터미널 3** — 네비게이션 시작 (지도 자동 로드):
```bash
bash ~/autonomous_nav/scripts/3_start_navigation.sh
# 특정 지도 지정 시:
bash ~/autonomous_nav/scripts/3_start_navigation.sh ~/autonomous_nav/maps/my_map.yaml
```

**터미널 4** — 메카넘 파라미터 적용 (측면 이동 활성화):
```bash
bash ~/autonomous_nav/scripts/apply_mecanum_params.sh
```

**터미널 5** — 자율주행 실행:
```bash
# 단일 목표 지점 이동
python3 ~/autonomous_nav/python/nav_goal_sender.py 1.5 2.0 90

# 다중 웨이포인트 순찰 (무한 반복)
python3 ~/autonomous_nav/python/patrol_navigator.py

# 3바퀴만 순찰
python3 ~/autonomous_nav/python/patrol_navigator.py --loops 3

# 카메라 장애물 감지 (선택사항, 별도 터미널)
python3 ~/autonomous_nav/python/camera_obstacle.py
```

---

## 웨이포인트 설정

`config/waypoints.yaml` 편집:

```yaml
waypoints:
  - name: "거실"
    x: 2.5      # RViz에서 확인한 X 좌표
    y: 1.0      # RViz에서 확인한 Y 좌표
    yaw: 0.0    # 도착 시 방향 (도)
    wait: 3.0   # 도착 후 대기 시간 (초)
```

**좌표 확인 방법**: RViz → `Publish Point` 도구 선택 → 지도에서 클릭  
터미널에서 `rostopic echo /clicked_point` 실행 시 좌표 출력

---

## 상태 모니터링

```bash
# 실시간 상태 (위치, 속도, LiDAR, 네비게이션 상태)
python3 ~/autonomous_nav/python/robot_status.py

# 주요 토픽 직접 확인
rostopic echo /odom            # 오도메트리
rostopic echo /scan            # LiDAR 데이터
rostopic echo /move_base/status # 네비게이션 상태
rostopic echo /obstacle_detected # 카메라 장애물 감지

# 카메라 영상 확인
rqt_image_view /camera/obstacle_view
```

---

## 비상 정지

```bash
bash ~/autonomous_nav/scripts/stop_all.sh
```

또는 키보드로 `Ctrl+C` → `k` 키 (텔레오퍼레이션 창)

---

## 주요 ROS 토픽

| 토픽 | 타입 | 설명 |
|-----|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 로봇 속도 명령 |
| `/scan` | sensor_msgs/LaserScan | YDLidar 데이터 |
| `/odom` | nav_msgs/Odometry | 오도메트리 |
| `/map` | nav_msgs/OccupancyGrid | 지도 |
| `/obstacle_detected` | std_msgs/Bool | 카메라 장애물 감지 |
| `/camera/obstacle_view` | sensor_msgs/Image | 카메라 시각화 |

## 메카넘 휠 속도 구성

```
linear.x  = 전진/후진
linear.y  = 좌/우 횡이동 (메카넘 특수 기능)
angular.z = 제자리 회전
```

기존 DWA 설정은 `max_vel_y: 0.0`으로 횡이동이 비활성화되어 있습니다.  
`apply_mecanum_params.sh` 실행으로 활성화하세요.
