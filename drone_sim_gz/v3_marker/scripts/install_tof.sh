#!/bin/bash
# Install the wall-keep model starling2_tof + airframe 4202 into the PX4 tree WITHOUT a rebuild and without touching
# starling2 / 4201 (instances 0 and 1 keep running). Additive only:
#   Tools/simulation/gz/models/starling2_tof/            (model, found through PX4_GZ_MODELS)
#   build/px4_sitl_default/etc/init.d-posix/airframes/4202_gz_starling2_tof   (rcS looks airframes up by name here)
#   ROMFS/.../airframes/4202_gz_starling2_tof            (source copy; not in CMakeLists -> a rebuild may drop the build copy,
#                                                         run_wk_instance.sh re-installs it on every start)
# 2026-10-31 (codex model-install-atomicity): run_wk_instance.sh calls this on EVERY boot, also for parallel instances. Sources AND
# the airframe target directories are checked before anything changes; one lock (shared with install_into_px4_gz.sh); the model
# is a versioned directory behind a symlink swapped with one rename (repo_export/scripts/px4_model_swap.sh), nothing is touched when
# the source is unchanged; any failure exits non-zero (run_wk_instance.sh then does not start PX4).
# Limit: a gz instance that is already running keeps the model it loaded; a change applies to the next start.
set -eu
PX4=${PX4:-$HOME/starling_sim/PX4-Autopilot}
V2=$(cd "$(dirname "$0")/.." && pwd)
GZ=$PX4/Tools/simulation/gz
SRC_M=$V2/gz/models/starling2_tof; SRC_A=$V2/gz/airframes/4202_gz_starling2_tof
AFS="$PX4/build/px4_sitl_default/etc/init.d-posix/airframes $PX4/ROMFS/px4fmu_common/init.d-posix/airframes"
SWAP=$(cd "$V2/.." && pwd)/repo_export/scripts/px4_model_swap.sh
for p in "$SRC_M/model.sdf" "$SRC_M/model.config" "$SRC_M/meshes" "$SRC_A" "$GZ/models" "$SWAP" $AFS; do
  [ -e "$p" ] || { echo "!! install_tof: missing $p - nothing changed"; exit 1; }
done
. "$SWAP"
exec 9>"$GZ/models/.install.lock"; flock -w 60 9 || { echo "!! install_tof: lock timeout"; exit 1; }
ARC=$HOME/starling_sim/archive/$(date +%Y%m%d)/px4_installed_models
model=$(swap_model "$SRC_M" "$GZ/models" starling2_tof) || { echo "!! install_tof: model swap failed"; exit 1; }
for d in $AFS; do
  cmp -s "$SRC_A" "$d/4202_gz_starling2_tof" || { cp "$SRC_A" "$d/.4202.$$"; chmod +x "$d/.4202.$$"; mv "$d/.4202.$$" "$d/4202_gz_starling2_tof"; }
done
echo "installed: $GZ/models/starling2_tof -> $(readlink "$GZ/models/starling2_tof") ($model), airframe 4202_gz_starling2_tof"
