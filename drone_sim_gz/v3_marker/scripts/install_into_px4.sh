#!/bin/bash
# Install the Starling 2 / QVIX customisations from repo_export into a PX4-Autopilot (v1.14, Gazebo Classic) tree.
# usage: install_into_px4.sh [PX4_DIR]   (default ~/starling_sim/PX4-Autopilot)
set -e
PX4=${1:-$HOME/starling_sim/PX4-Autopilot}
RE=$(cd "$(dirname "$0")/.." && pwd)
SG=$PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic
AF=$PX4/ROMFS/px4fmu_common/init.d-posix/airframes
[ -d "$SG/models" ] || { echo "sitl_gazebo-classic not found under $PX4 (submodules missing?)"; exit 1; }

echo "== models"
cp -r "$RE/airframe/starling2"        "$SG/models/"
cp -r "$RE/airframe/iris_starling"    "$SG/models/"          # legacy Iris-shaped model (kept for comparison)
cp -r "$RE/qvix_integration/qvix_r_module" "$SG/models/"
echo "== world"
cp "$RE/qvix_integration/qvix.world"  "$SG/worlds/"
echo "== airframes"
cp "$RE/airframe/4200_gazebo-classic_iris_starling" "$RE/airframe/4201_gazebo-classic_starling2" "$AF/"
chmod +x "$AF/4200_gazebo-classic_iris_starling" "$AF/4201_gazebo-classic_starling2"
# register airframes in CMakeLists (insert after the iris entry)
CM=$AF/CMakeLists.txt
for a in 4200_gazebo-classic_iris_starling 4201_gazebo-classic_starling2; do
  grep -q "$a" "$CM" || sed -i "/10015_gazebo-classic_iris$/a\\\t$a" "$CM"
done
# register qvix world in the gazebo-classic target list
TG=$PX4/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake
grep -q "^\s*qvix$" "$TG" || sed -i "/^\t\tempty$/a\\\t\tqvix" "$TG"
echo "== verify"
grep -n "iris_starling\|starling2" "$CM"
grep -n -B2 -A2 "qvix" "$TG"
ls "$SG/models" | grep -E "starling|qvix"
ls "$SG/worlds" | grep qvix
echo "done. build with:  make px4_sitl gazebo-classic_starling2__qvix"
