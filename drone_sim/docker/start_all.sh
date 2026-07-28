#!/bin/bash
# Container A와 B 둘 다 시작
echo "Starting Container A (SITL)..."
docker start starling-sitl > /dev/null
echo "Starting Container B (flight_code)..."
docker start starling-flight > /dev/null
docker ps --filter "name=starling" --format "table {{.Names}}\t{{.Status}}"
echo ""
echo "다음 단계:"
echo "  Container A 진입:  docker exec -it starling-sitl bash"
echo "  Container B 진입:  docker exec -it starling-flight bash"
