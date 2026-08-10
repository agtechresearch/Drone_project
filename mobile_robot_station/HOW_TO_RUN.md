# 지도 작성 실행 방법

> RViz는 VNC 화면(:0)에 표시됨. SSH 터미널에서 실행해도 스크립트 내 `DISPLAY=:0` 설정으로 자동 연결.

---

## 방법 1 — 수동 조종 (키보드 텔레오퍼레이션)

### 터미널 1 — roscore

```bash
source /opt/ros/noetic/setup.bash && roscore
```

### 터미널 2 — SLAM (gmapping)

```bash
cd /home/er/autonomous_nav && bash scripts/1_slam_mapping.sh
```

### 터미널 3 — 키보드 텔레오퍼레이션

```bash
cd /home/er/autonomous_nav && bash scripts/start_teleop.sh
```

- `u i o / j k l / m , .` 키로 조종
- `k` = 정지, `q/e` = 속도 조절

### 터미널 4 — 지도 저장 (완성 후 실행)

```bash
cd /home/er/autonomous_nav && bash scripts/2_save_map.sh
```

---

## 방법 2 — 자동 탐색 (explore_lite frontier 탐색)

로봇이 스스로 미탐색 영역을 찾아다니며 지도를 완성함.

### 터미널 1 — roscore

```bash
source /opt/ros/noetic/setup.bash && roscore
```

### 터미널 2 — 라이다 + 오도메트리

```bash
roslaunch myagv_odometry myagv_active.launch
```

### 터미널 3 — 자동 탐색 + gmapping

```bash
cd /home/er/autonomous_nav && bash scripts/auto_explore.sh
```

### 터미널 4 — 지도 저장 (완성 후 실행)

```bash
cd /home/er/autonomous_nav && bash scripts/2_save_map.sh
```

- 저장 위치: `maps/map_YYYYMMDD_HHMMSS.pgm/.yaml`
- `maps/current_map.yaml` 심볼릭 링크 자동 생성
