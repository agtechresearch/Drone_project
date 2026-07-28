#!/bin/bash
set -e
cd "$(dirname "$0")"

docker build \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    --build-arg USERNAME=dronesim \
    -t starling-flight:latest \
    -f Dockerfile.flight \
    .

echo ""
echo "✓ 빌드 완료: starling-flight:latest"
