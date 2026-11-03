#!/bin/bash
set -e
cd "$(dirname "$0")"
docker build --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) --build-arg USERNAME=dronesim \
    -t starling-sitl-gz:latest -f Dockerfile.gz .
echo "✓ 빌드 완료: starling-sitl-gz:latest   다음: ./run_gz.sh"
