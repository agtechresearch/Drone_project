#!/bin/bash
# v3_marker copy (2026-10-29): launches the WALL-KEEP vehicle starling2_tof (airframe 4202, front ToF, relayed VIO with
# error injection: vio_relay.py reads $WK_INJECT, default /tmp/wk_inject_<N>.json) as instance N (use 2). Model name starling2_tof_N.
# Launch ONE independent PX4 + Gazebo instance (run INSIDE starling-gz), for parallel headless experiments.
#   run_sitl_instance.sh <instance N> <world: default|qvix> [headless] [x,y,z,roll,pitch,yaw] [real_time_factor]
# Instance N gets: gz transport partition "starling_N" (its own Gazebo world, invisible to the others), PX4 "-i N"
# (MAVLink offboard udp 14540+N, GCS link local port 18570+N, MAV_SYS_ID N+1), its own working dir
# build/px4_sitl_default/instance_N (parameters, logs), model name starling2_N, front camera topic /front_camera (per partition).
# N=0 is equivalent to run_sitl.sh (default partition, make target) and should be started with run_sitl.sh instead.
# The global origin is set through the offboard port (udpin 1454N) before mavsdk_server binds it; shell commands for the
# instance go through the local PX4 client: build/px4_sitl_default/bin/px4-<module> --instance N ... (no MAVLink needed).
# GPU rendering (2026-10-28): the container has libEGL_nvidia but no GLVND vendor file, so EGL fell back to Mesa llvmpipe
# (GUI ~6 fps, camera sensor on CPU). Point GLVND at NVIDIA when the GPU is reachable. If /dev/nvidiactl gives
# "Operation not permitted" (NVML "Unknown Error"), the container lost its GPU cgroup access: docker restart starling-gz.
if python3 -c 'open("/dev/nvidiactl","rb")' 2>/dev/null; then export __EGL_VENDOR_LIBRARY_FILENAMES=$HOME/starling_sim/repo_export/gz/glvnd/10_nvidia.json; fi
N=${1:?instance}; WORLD=${2:-qvix}; [ "${3:-}" = "headless" ] && export HEADLESS=1
export GZ_PARTITION="starling_$N"
export PX4_SIM_MODEL=gz_starling2_tof PX4_GZ_WORLD="$WORLD"
if [ "$WORLD" = "qvix" ]; then export PX4_GZ_MODEL_POSE="${4:-2.05,-0.8,0.18,0,0,1.5708}"; else export PX4_GZ_MODEL_POSE="${4:-0,0,0.035,0,0,1.5708}"; fi
RTF=${5:-1}
V2=$HOME/starling_sim/v3_marker
"$V2/scripts/install_tof.sh" || { echo "!! install_tof failed - PX4 not started"; echo "EXIT=1"; exit 1; }   # additive, idempotent; no-op when identical (2026-10-31)
WK_INJECT=${WK_INJECT:-/tmp/wk_inject_$N.json}; [ -f "$WK_INJECT" ] || echo '{}' > "$WK_INJECT"
# VIO relay: the model publishes its odometry on /wallkeep/vio_raw; PX4 gets it (with optional injected errors) from here
python3 "$V2/scripts/vio_relay.py" "starling2_tof_$N" "$WK_INJECT" > "/tmp/vio_relay_$N.log" 2>&1 &
cd ~/PX4-Autopilot/build/px4_sitl_default
# WK_MODEL_DIR (2026-11-01, model variant test, e.g. v3_marker/gz/variants/m2 from make_model_m2.py): a folder with another
# starling2_tof/ goes FIRST on THIS instance's resource path; PX4 spawns "starling2_tof/model.sdf" by resource path
# (GZBridge.cpp:75), so only this instance gets the variant, under the same model name. Unset = the installed model, as before.
if [ -n "${WK_MODEL_DIR:-}" ]; then
  [ -f "$WK_MODEL_DIR/starling2_tof/model.sdf" ] || { echo "!! WK_MODEL_DIR=$WK_MODEL_DIR has no starling2_tof/model.sdf - PX4 not started"; echo "EXIT=1"; exit 1; }
  export GZ_SIM_RESOURCE_PATH="$WK_MODEL_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
  echo "[run_sitl_instance] model variant: $WK_MODEL_DIR/starling2_tof/model.sdf sha256 $(sha256sum "$WK_MODEL_DIR/starling2_tof/model.sdf" | cut -c1-64)"
fi
. ./rootfs/gz_env.sh            # PX4_GZ_WORLDS / GZ_SIM_RESOURCE_PATH etc. (the make target exports these; appended after)
mkdir -p "instance_$N"
# origin over the instance's OFFBOARD link (PX4 sends it to udp 14540+N; mavsdk_server is started only after "New NED origin").
# Not the GCS link: every instance's GCS link sends to the same remote port 14550 and PX4 locks onto the first sender.
( sleep 3; python3 "$V2/scripts/set_global_origin.py" "udpin:0.0.0.0:$((14540 + N))" 37.5 127.0 0
  if [ "$RTF" != "1" ]; then sleep 5; gz service -s "/world/$WORLD/set_physics" --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 3000 --req "real_time_factor: $RTF" >/dev/null && echo "[run_sitl_instance] real_time_factor $RTF"; fi ) &
echo "[run_sitl_instance] N=$N partition=$GZ_PARTITION world=$WORLD headless=${HEADLESS:-0} pose=$PX4_GZ_MODEL_POSE rtf=$RTF"
# same launch as the make target (rootfs = CWD) but with -i and a private working dir
# -w = private working dir (params, dataman, logs); the positional rootfs path is symlinked into it as etc/
cd rootfs && tail -f /dev/null | ../bin/px4 -i "$N" -w "../instance_$N" -d "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/etc"
echo "EXIT=$?"
