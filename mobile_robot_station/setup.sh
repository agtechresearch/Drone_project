#!/bin/bash
# ROS 환경 및 myagv_ros 워크스페이스 소스
source /opt/ros/noetic/setup.bash
source /home/er/myagv_ros/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

echo "[OK] ROS Noetic + myagv_ros 환경 로드 완료"
echo "     ROS_MASTER_URI: $ROS_MASTER_URI"
