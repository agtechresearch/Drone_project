#!/bin/bash
# 비상 정지 - 모든 ROS 노드 종료 및 로봇 정지

source "$(dirname "$0")/../setup.sh"

echo "[비상 정지] cmd_vel 제로 명령 발행..."
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' 2>/dev/null || true

echo "[비상 정지] 모든 ROS 노드 종료..."
rosnode kill --all 2>/dev/null || true

sleep 1
pkill -f "roslaunch"            2>/dev/null || true
pkill -f "rosrun"               2>/dev/null || true
pkill -f "myagv_odometry_node"  2>/dev/null || true
pkill -f "ydlidar_ros_driver"   2>/dev/null || true
pkill -f "patrol_navigator"     2>/dev/null || true
pkill -f "camera_obstacle"      2>/dev/null || true
pkill -f "line_follow"          2>/dev/null || true
pkill -f "map_drive"            2>/dev/null || true
pkill -f "waypoint_ui"          2>/dev/null || true
pkill -f "lidar_power_daemon"   2>/dev/null || true  # GPIO 20 LOW → 모터 정지

sleep 1
echo "[완료] 모든 프로세스 종료"
