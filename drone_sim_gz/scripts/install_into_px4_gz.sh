#!/bin/bash
# Install repo_export/gz (starling2 model, qvix world, airframe) into a PX4 v1.15 tree and register the gz targets.
# 2026-10-31 (codex model-install-atomicity): every source and target is checked BEFORE anything in the PX4 tree is touched; one
# lock shared with v3_marker install_tof.sh; each model is a versioned directory behind a symlink swapped with one rename
# (px4_model_swap.sh; a real directory from an older install is moved once to ~/starling_sim/archive/<date>/px4_installed_models/).
# Single files (world, airframe) are copied next to the target and renamed over it.
# Limits: the two CMakeLists registrations are in-place sed edits (idempotent, only added when missing) and need a PX4 rebuild;
# the install as a whole is not one transaction (models, world and airframe switch one after the other); running gz instances
# keep what they loaded.
set -e
PX4=${1:-$HOME/starling_sim/PX4-Autopilot}
RE=$(cd "$(dirname "$0")/.." && pwd)
GZ=$PX4/Tools/simulation/gz; AF=$PX4/ROMFS/px4fmu_common/init.d-posix/airframes
CM=$PX4/src/modules/simulation/gz_bridge/CMakeLists.txt
for p in "$GZ/models" "$GZ/worlds" "$AF" "$AF/CMakeLists.txt" "$CM" "$RE/gz/models/starling2/model.sdf" "$RE/gz/models/qvix_r_module" \
         "$RE/gz/worlds/qvix.sdf" "$RE/gz/airframes/4201_gz_starling2" "$RE/scripts/px4_model_swap.sh"; do
  [ -e "$p" ] || { echo "!! missing $p - nothing installed"; exit 1; }
done
. "$RE/scripts/px4_model_swap.sh"
exec 9>"$GZ/models/.install.lock"; flock -w 60 9 || { echo "!! lock timeout - nothing installed"; exit 1; }
ARC=$HOME/starling_sim/archive/$(date +%Y%m%d)/px4_installed_models
# the helper's status is checked before anything is printed (2026-11-01 codex reply-4: echo "$(...)" hid a failed swap)
for m in starling2 qvix_r_module; do
  r=$(swap_model "$RE/gz/models/$m" "$GZ/models" $m) || { echo "!! $m: model swap failed - $m keeps its previous version; stopping"; exit 1; }
  echo "$m: $r"
done
# every step checked on its own (2026-11-01 codex reply-6: in an `a && b` list set -e does not stop on a failing a); a failed
# copy leaves the installed target untouched (the staging file is removed)
put_file(){ # src dst [mode]
  local tmp; tmp="$(dirname "$2")/.$(basename "$2").$$"
  cp "$1" "$tmp" || { rm -f "$tmp"; echo "!! cannot stage $2 - it keeps its previous content; stopping"; exit 1; }
  if [ -n "${3:-}" ]; then chmod "$3" "$tmp" || { rm -f "$tmp"; echo "!! chmod $3 failed for $2 - unchanged; stopping"; exit 1; }; fi
  mv -f "$tmp" "$2" || { rm -f "$tmp"; echo "!! cannot replace $2 - unchanged; stopping"; exit 1; }
}
put_file "$RE/gz/worlds/qvix.sdf" "$GZ/worlds/qvix.sdf"
put_file "$RE/gz/airframes/4201_gz_starling2" "$AF/4201_gz_starling2" +x
grep -q "4201_gz_starling2" "$AF/CMakeLists.txt" || sed -i "/^\t4001_gz_x500$/a\\\t4201_gz_starling2" "$AF/CMakeLists.txt"
grep -qE "^\s*qvix\s*$" "$CM" || sed -i "/^\t\tdefault$/a\\\t\tqvix" "$CM"
echo "== registered:"; grep -n "4201_gz_starling2" "$AF/CMakeLists.txt"; grep -n -A5 "set(gz_worlds" "$CM"
ls "$GZ/models" | grep -E "starling2|qvix"; ls "$GZ/worlds" | grep qvix
echo "previous real directories (first migration only) -> $ARC; versions in $GZ/models/.versions"
echo "build+run:  make px4_sitl gz_starling2_qvix"
