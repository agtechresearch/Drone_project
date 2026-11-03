#!/bin/bash
# Harmonic SITL 컨테이너 (GPU + X11). starling-gui 와 같은 방식.
set -e
xhost +local:docker > /dev/null 2>&1 || true
if docker ps -a --format '{{.Names}}' | grep -q '^starling-gz$'; then
    docker start starling-gz > /dev/null; docker exec -it starling-gz /bin/bash
else
    # --init (2026-11-01): PID 1 reaps exited children; without it ~800 zombies piled up over the batches
    docker run -it --init --name starling-gz --hostname starling-gz \
        --gpus all --device /dev/dri --network host \
        --group-add $(getent group render | cut -d: -f3) --group-add $(getent group video | cut -d: -f3) \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e DISPLAY=${DISPLAY:-:1} -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/starling_sim/PX4-Autopilot:/home/dronesim/PX4-Autopilot \
        -v $HOME/starling_sim:/home/dronesim/starling_sim \
        -v /tmp/vgl_tvnc_pkgs:/vgl:ro \
        --workdir /home/dronesim starling-sitl-gz:latest
fi
