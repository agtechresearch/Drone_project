# Setup

랩 서버(Ubuntu 22.04 + Docker + GPU)에서 Container A/B를 처음부터 구축하는 절차다.

## 사전 조건

- Ubuntu 22.04 이상 (호스트)
- Docker 20.10 이상, dronesim 계정이 docker 그룹 소속
- Nvidia GPU (선택, Gazebo 렌더링 가속용)
- 디스크 여유 20GB 이상
- 랩 네트워크에서 서버 접근 가능

## 1. 작업 폴더 준비

```bash
mkdir -p ~/starling_sim
cd ~/starling_sim
git clone https://github.com/agtechresearch/Drone_project.git .
# 또는 이미 clone된 상태라면 drone_sim 폴더 안 파일들만 배치
```

## 2. PX4-Autopilot 소스 확보

Container A가 마운트해서 빌드할 PX4 소스다. 호스트에 두면 컨테이너를 재생성해도 소스가 남아있다.

```bash
cd ~/starling_sim
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.14.0
git submodule update --init --recursive
```

클론 5-10분, submodule 초기화 추가 5분 정도 걸린다.

## 3. Container A (SITL) 빌드

```bash
cd ~/starling_sim/docker
./build.sh
```

첫 빌드 20-40분 소요. 이미지 크기 약 3.6GB.

성공 확인:
```bash
docker images | grep starling-sitl
```

## 4. Container A 실행 및 첫 검증

```bash
./run.sh
# 프롬프트가 dronesim@starling-sitl:~$ 로 바뀜
```

컨테이너 안에서 hosts 파일 설정 (VNC용, 컨테이너 재생성 시마다 필요):
```bash
echo "127.0.0.1 starling-sitl" | sudo tee -a /etc/hosts
```

PX4 SITL 첫 실행 (Gazebo GUI 없이):
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

`pxh>` 프롬프트가 뜨면 성공. 콘솔에서 이륙/착륙 테스트:
```
pxh> commander takeoff
pxh> commander land
pxh> shutdown
```

## 5. VNC 셋업 (Gazebo GUI를 원격에서 볼 때)

Container A 안에서:

```bash
# VNC 패키지는 이미지에 포함되어 있음
which vncserver fluxbox websockify

# 비밀번호 설정
mkdir -p ~/.vnc
vncpasswd
# 짧은 비밀번호 (예: starling), view-only는 n

# xstartup 스크립트 (Xauthority 우회 버전)
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XKL_XMODMAP_DISABLE=1
export DISPLAY=:1
export XAUTHORITY=/tmp/.Xauthority-dronesim
xsetroot -solid grey &
exec fluxbox
EOF
chmod +x ~/.vnc/xstartup

# 편의를 위한 시작 스크립트
cat > ~/start_vnc.sh << 'EOF'
#!/bin/bash
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
touch /tmp/.Xauthority-dronesim
vncserver -localhost no :1 -geometry 1600x900 -depth 24 -auth /tmp/.Xauthority-dronesim
pkill -9 websockify 2>/dev/null
sleep 1
websockify -D --web=/usr/share/novnc/ 0.0.0.0:6080 localhost:5901
echo "VNC 시작. 브라우저: http://<서버IP>:6080/vnc.html"
EOF
chmod +x ~/start_vnc.sh

# 실행
~/start_vnc.sh
```

호스트에서 방화벽 허용:
```bash
sudo ufw allow 6080/tcp
sudo ufw allow 5901/tcp
```

브라우저에서 `http://<서버IP>:6080/vnc.html` 접속 → Connect → 비밀번호 입력 → 회색 데스크톱.

## 6. Gazebo GUI로 SITL 실행

Container A 안에서:
```bash
cd ~/PX4-Autopilot
DISPLAY=:1 XAUTHORITY=/tmp/.Xauthority-dronesim make px4_sitl gazebo-classic
```

브라우저 VNC 화면에 iris 쿼드콥터가 있는 월드가 뜬다.

편의를 위해 .bashrc에 환경변수를 넣어두면 이후 명시 불필요:
```bash
cat >> ~/.bashrc << 'EOF'
export DISPLAY=:1
export XAUTHORITY=/tmp/.Xauthority-dronesim
EOF
```

## 7. ROS2 자동 source 무력화 (중요)

`ros-humble-gazebo-ros-pkgs`가 자동으로 붙으면 iris 스폰이 실패한다. Dockerfile에서 자동 source하도록 되어 있는 걸 제거:

```bash
sed -i '/source \/opt\/ros\/humble\/setup.bash/d' ~/.bashrc

# 필요할 때만 명시적으로 source하도록 별칭 추가
cat >> ~/.bashrc << 'EOF'
alias ros2_env='source /opt/ros/humble/setup.bash'
EOF
```

## 8. Container B (flight_code) 빌드

호스트에서:
```bash
cd ~/starling_sim/docker
./build_flight.sh
```

5-10분 소요. Container A보다 훨씬 가볍다 (Ubuntu 18.04 베이스 + MAVSDK만).

```bash
docker images | grep starling-flight
```

## 9. Container B 실행 및 검증

```bash
./run_flight.sh
# 프롬프트가 dronesim@starling-flight:~/workspace$ 로 바뀜
```

Container B 안에서 환경 확인:
```bash
python3 --version                              # 3.6.9
python3 -c "import mavsdk; print(mavsdk.__version__)"  # 0.30.1 (라이브러리 버전)
ls ~/workspace/                                # flight_code 파일이 보여야 함
```

## 10. 두 컨테이너 통신 검증

**전제**: Container A에서 SITL이 실행 중이어야 한다.

Container B에서 mavsdk_server 백그라운드 실행:
```bash
MAVSDK_BIN=/usr/local/lib/python3.6/dist-packages/mavsdk/bin/mavsdk_server
$MAVSDK_BIN -p 50051 udp://:14540 > ~/mavsdk_server.log 2>&1 &

sleep 2
cat ~/mavsdk_server.log | tail -20
```

`System discovered [UUID: 1]` 메시지가 있으면 SITL과 연결됨.

flight_code 실행:
```bash
cd ~/workspace
python3 path_flight_phase1_v13.py --csv auto --csv-sample-sec 1.0
```

**현재 시점(Day 1)의 예상 실패 지점**: `[5/7] Starting monitors...` 단계에서 voxl-inspect-pose 없음으로 인해 `wait_reach_pipe_ready`가 8초 후 RuntimeError로 종료. 이는 예상된 실패이며, Day 3 작업(voxl_pose_reader의 SITL 대응)으로 해결한다.

## 11. 이미지 스냅샷 (안전 백업)

셋업이 정상 작동함을 확인한 후, 재현 시간을 아끼려면 이미지 태그를 저장한다:

```bash
docker commit starling-sitl starling-sitl:day1-complete
docker commit starling-flight starling-flight:day1-complete
docker images | grep starling
```

## 12. 재시작 절차

컨테이너를 종료해도 Docker가 상태를 유지한다. 재시작:

```bash
~/starling_sim/docker/start_all.sh
```

또는 개별:
```bash
docker start starling-sitl starling-flight
docker exec -it starling-sitl bash
```

## 트러블슈팅

`docs/day1_journal.md`에 오늘 겪은 문제와 해결 방법을 정리해두었다.
