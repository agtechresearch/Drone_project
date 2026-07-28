#!/bin/bash
set -e

# X11 forwarding 준비
xhost +local:docker > /dev/null 2>&1 || true

# 이미 컨테이너가 있으면 재사용, 없으면 새로 생성
if docker ps -a --format '{{.Names}}' | grep -q '^starling-sitl$'; then
    echo "기존 컨테이너에 attach 중..."
    docker start starling-sitl > /dev/null
    docker exec -it starling-sitl /bin/bash
else
    echo "새 컨테이너 생성 중..."
    docker run -it \
        --name starling-sitl \
        --hostname starling-sitl \
        --gpus all \
        --network host \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/starling_sim/PX4-Autopilot:/home/dronesim/PX4-Autopilot \
        --workdir /home/dronesim \
        starling-sitl:latest
fi
