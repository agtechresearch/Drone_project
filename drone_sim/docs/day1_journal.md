# Day 1 Journal — 2026-07-28

시뮬레이션 베이스라인 구축.

## 목표

- Container A(SITL): PX4 v1.14.0 + Gazebo Classic 11 정상 실행
- Container B(flight_code): Python 3.6.9 + MAVSDK 0.12.0
- 두 컨테이너 UDP 연결
- 실기 flight_code를 수정 없이 SITL에서 실행 (실패 지점 파악)

## 결과

**모든 목표 달성.** 유일한 하드웨어 의존(voxl-inspect-pose)이 어디서 어떻게 걸리는지 정확히 확인됨. Day 3에서 해당 부분만 수정하면 미션 20단계가 그대로 실행 가능한 상태.

## 진행 순서

1. 서버 환경 조사 → 스택 확정 (Ubuntu 22.04, ROS2 Humble, Gazebo 3종, RTX 4090)
2. 호스트 직접 설치 시도 → **실패** (ROS-Gazebo 자동 후킹으로 iris 스폰 안 됨)
3. Docker 격리 방향 전환
4. Container A Dockerfile 작성 및 빌드
5. Container A 안에서 PX4 SITL 실행 → 성공
6. VNC + noVNC로 Gazebo GUI 원격 접근 구축
7. Container B Dockerfile 작성 및 빌드 (Ubuntu 18.04 베이스)
8. mavsdk_server 백그라운드 실행 + flight_code 실행
9. 예상된 실패 지점 확인 (voxl-inspect-pose 없음)

## 삽질 로그 (같은 문제 재발 방지)

### 1. ROS-Gazebo 자동 후킹으로 iris 스폰 실패

**증상**: `make px4_sitl gazebo-classic` 실행 시 gzserver는 뜨는데 iris가 스폰되지 않음. `simulator_mavlink: Waiting for simulator to accept connection on TCP port 4560`에서 무한 대기.

**원인**: `ros-humble-gazebo-ros-pkgs`가 시스템에 깔려 있고, `.bashrc`에서 자동으로 ROS2 환경을 source하면서 `LD_LIBRARY_PATH`에 ROS2의 OGRE 라이브러리 경로가 들어감. Gazebo Classic 11이 자기 OGRE 대신 ROS2 것을 잘못 로드하여 모델 스폰 절차가 조용히 실패.

**진단 로그**:
```
LD_LIBRARY_PATH /opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:...
```

**시도했으나 안 된 것**:
- `env -i` 로 환경변수 완전 제거 → 여전히 실패 (시스템 레벨 오염)
- `bash --noprofile --norc` → 여전히 LD_LIBRARY_PATH 남음
- `xvfb-run` 로 가상 디스플레이 → libGL 에러는 사라졌지만 스폰 실패 지속

**해결**: Docker 컨테이너로 완전 격리. 컨테이너 안에서는 gazebo-ros-pkgs를 설치하지 않아 충돌 원인 자체가 없음.

### 2. Container A에서도 ROS 자동 source 재발

**증상**: Docker 컨테이너 안에서도 `make px4_sitl gazebo-classic` 시 같은 무한 대기 발생.

**원인**: Dockerfile에서 편의를 위해 `.bashrc`에 `source /opt/ros/humble/setup.bash`를 넣어둠. 컨테이너 안에서도 호스트와 같은 오염 상황.

**해결**: `.bashrc`에서 자동 source 제거하고, 필요 시 명시적으로 실행하는 별칭으로 대체.
```bash
sed -i '/source \/opt\/ros\/humble\/setup.bash/d' ~/.bashrc
echo "alias ros2_env='source /opt/ros/humble/setup.bash'" >> ~/.bashrc
```

### 3. VNC hostname 해결 실패

**증상**: `vncserver` 실행 시 `hostname: Temporary failure in name resolution`.

**원인**: 컨테이너의 hostname(`starling-sitl`)이 `/etc/hosts`에 없어서 자기 자신을 IP로 해석 못 함.

**해결**:
```bash
echo "127.0.0.1 starling-sitl" | sudo tee -a /etc/hosts
```

컨테이너 재생성 시마다 필요. Dockerfile에 넣으면 이미지 빌드 시점에는 컨테이너 hostname이 정해지지 않아 소용없음.

### 4. VNC 무단 접속 거부

**증상**: `-SecurityTypes None`으로 실행 시 `YOU ARE TRYING TO EXPOSE A VNC SERVER WITHOUT ANY AUTHENTICATION` 에러.

**원인**: TigerVNC의 안전장치.

**해결**: `vncpasswd`로 비밀번호 설정 후, `-SecurityTypes None` 옵션 제거하고 실행.

### 5. Xauthority 인증 실패 (fluxbox가 X에 붙지 못함)

**증상**: `vncserver`는 성공적으로 뜨는데 로그에 `Authorization required, but no authorization protocol specified` → `xsetroot: unable to open display ':1'` → `Error: Couldn't connect to XServer:1`.

**원인**: run.sh에서 호스트 `~/.Xauthority`를 읽기 전용(:ro)으로 마운트해뒀는데, Xvnc가 여기에 인증 토큰을 쓰려다 실패.

**해결**: 
1. run.sh에서 Xauthority 마운트 줄 제거
2. `/tmp/.Xauthority-dronesim`을 새로 만들어 쓰기 가능한 인증 파일로 사용
3. xstartup에 `export XAUTHORITY=/tmp/.Xauthority-dronesim` 명시
4. `vncserver -auth /tmp/.Xauthority-dronesim` 로 시작
5. Gazebo 실행 시에도 `DISPLAY=:1 XAUTHORITY=/tmp/.Xauthority-dronesim` 명시

### 6. websockify가 localhost만 바인딩

**증상**: 컨테이너 안 `curl -I http://localhost:6080/vnc.html`은 200 OK인데, Mac에서 `nc -zv 서버IP 6080`은 timeout.

**원인**: `websockify -D --web=/usr/share/novnc/ 6080 localhost:5901`에서 첫 인자 `6080`이 IPv6 `[::1]`에만 바인딩되어 외부 접근 불가.

**해결**: `0.0.0.0:6080`으로 명시.
```bash
websockify -D --web=/usr/share/novnc/ 0.0.0.0:6080 localhost:5901
```

### 7. Python 3.6에서 asyncio.run() 없음

**증상**: 첫 MAVSDK 연결 테스트 시 `AttributeError: module 'asyncio' has no attribute 'run'`.

**원인**: `asyncio.run()`은 Python 3.7+ API.

**해결**: 3.6 호환 방식으로 이벤트 루프 명시.
```python
loop = asyncio.get_event_loop()
loop.run_until_complete(main())
```

flight_code 원본은 이미 3.6 호환으로 작성되어 있음.

### 8. flight_code가 mavsdk_server 없이 무한 대기

**증상**: `[1/7] Connecting...`에서 무한 대기.

**원인**: flight_code가 `System(mavsdk_server_address="localhost", port=50051)`로 **외부에서 이미 실행 중인** mavsdk_server를 기대. 인자 없이 `System()`을 부르면 자동으로 임시 서버가 뜨지만, 명시적 주소를 주면 외부 서버가 있어야 함.

**해결**: mavsdk_server를 백그라운드로 먼저 실행.
```bash
MAVSDK_BIN=/usr/local/lib/python3.6/dist-packages/mavsdk/bin/mavsdk_server
$MAVSDK_BIN -p 50051 udp://:14540 > ~/mavsdk_server.log 2>&1 &
```

## flight_code 실행 결과 (Day 1 종료 시점)

```
[20:20:16.334] [1/7] Connecting...
                     Connected!
[20:20:21.845]       Local position health OK.
[20:20:26.906]       Local position health stable.
[20:20:29.654]       Initial MAVSDK NED: N=+0.005 E=-0.008 D=+0.027
[20:20:29.657] [voxl] voxl-inspect-pose not found. Pipe 'px4_vehicle_local_position' disabled.
[20:20:38.704] Exception: Reach feedback pipe 'px4_vehicle_local_position' did not produce valid pose within 8.0s
[20:20:38.704] !!! EMERGENCY LAND TRIGGERED
```

정확히 예상한 실패 지점. Day 3에서 `voxl_pose_reader` 함수를 MAVSDK 버전으로 대체하면 그다음 단계로 진행 가능.

## Day 2 준비 사항

- 간소화된 QVIX 월드 SDF 작성 (벽/기둥/통로 대략 구조 + AprilTag 위치만 정확히)
- Starling 2 airframe 정의 (질량, 관성, 모터 배치)
- `indoor_vio_missing_gps.params` SITL에 적용

## 참고 자료

- `/home/dronesim/starling_sim/flight_test.log` — 전체 실행 로그
- `/home/dronesim/starling_sim/mavsdk_server.log` — mavsdk_server 로그
