#!/bin/bash
# Launch PX4 SITL (gz Harmonic) with the starling2 model, then set the global origin (VIO-only vehicle has no GPS).
# usage: run_sitl.sh <world: default|qvix> [headless] [x,y,z,roll,pitch,yaw]     (run inside the starling-gz container)
# Examples:
#   run_sitl.sh default headless                       # empty world, spawn at origin
#   run_sitl.sh qvix                                   # GUI, spawn inside QVIX (aisle 2 of the cross-aisle compartment, facing north)
# GPU rendering (2026-10-28): the container has libEGL_nvidia but no GLVND vendor file, so EGL fell back to Mesa llvmpipe
# (GUI ~6 fps, camera sensor on CPU). Point GLVND at NVIDIA when the GPU is reachable. If /dev/nvidiactl gives
# "Operation not permitted" (NVML "Unknown Error"), the container lost its GPU cgroup access: docker restart starling-gz.
if python3 -c 'open("/dev/nvidiactl","rb")' 2>/dev/null; then export __EGL_VENDOR_LIBRARY_FILENAMES=$HOME/starling_sim/repo_export/gz/glvnd/10_nvidia.json; fi
WORLD=${1:-default}; [ "$2" = "headless" ] && export HEADLESS=1
export PX4_GZ_WORLD="$WORLD"   # explicit: the airframe file has its own default and the make target without a world suffix does not set it
# Spawn facing NORTH (gz yaw 1.5708 = PX4 NED yaw 0) in every world: the mission script assumes yaw 0 = north at start,
# otherwise it yaws 90 deg left during takeoff (gz default yaw 0 faces +X = east). z 0.035 = legs just above the ground (0.18 on the QVIX floor).
if [ "$WORLD" = "qvix" ]; then export PX4_GZ_MODEL_POSE="${3:-2.05,-0.8,0.18,0,0,1.5708}"; else export PX4_GZ_MODEL_POSE="${3:-0,0,0.035,0,0,1.5708}"; fi
# origin setter: waits for the heartbeat on the GCS link (14550), sends SET_GPS_GLOBAL_ORIGIN (Seoul 37.5N 127.0E)
# over the OFFBOARD link (PX4 sends it to udp 14540; mavsdk_server is started only after "New NED origin" and after this
# script released the port). Not the GCS port 14550: with a parallel instance (bench.sh) every PX4 sends its GCS link
# to 14550 and the origin went to the wrong vehicle (2026-10-28 17:52).
( sleep 3; python3 "$(dirname "$0")/set_global_origin.py" udpin:0.0.0.0:14540 37.5 127.0 0 ) &
cd ~/PX4-Autopilot
echo "[run_sitl] world=$WORLD headless=${HEADLESS:-0} pose=${PX4_GZ_MODEL_POSE:-origin}"
if [ "$WORLD" = "default" ]; then tail -f /dev/null | make px4_sitl gz_starling2; else tail -f /dev/null | make px4_sitl gz_starling2_${WORLD}; fi
echo "EXIT=$?"
