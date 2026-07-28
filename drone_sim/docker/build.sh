#!/bin/bash
set -e

cd "$(dirname "$0")"

# 호스트 UID/GID로 이미지 빌드 (볼륨 마운트 시 권한 문제 방지)
docker build \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    --build-arg USERNAME=dronesim \
    -t starling-sitl:latest \
    -f Dockerfile \
    .

echo ""
echo "✓ 빌드 완료: starling-sitl:latest"
echo "  다음: ./run.sh 실행"
