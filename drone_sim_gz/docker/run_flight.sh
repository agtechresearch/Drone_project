#!/bin/bash
set -e

if docker ps -a --format '{{.Names}}' | grep -q '^starling-flight$'; then
    echo "기존 컨테이너에 attach 중..."
    docker start starling-flight > /dev/null
    docker exec -it starling-flight /bin/bash
else
    echo "새 컨테이너 생성 중..."
    docker run -it \
        --name starling-flight \
        --hostname starling-flight \
        --network host \
        -v $HOME/starling_sim:/home/dronesim/workspace \
        --workdir /home/dronesim/workspace \
        starling-flight:latest
fi
