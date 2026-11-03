#!/bin/bash
# Draw / clear the QVIX flight grid in the running Gazebo GUI and aim the camera at the room (run on the HOST).
#   grid_overlay.sh R1          # draw R1 cells (blue = aisle cell, green = crossing row) + labels
#   grid_overlay.sh R1 clear    # remove the overlay
#   grid_overlay.sh R1 camera   # only move the GUI camera inside R1 (north end, looking south)
# Needs the GUI running (markers live in the GUI's MarkerManager; a headless server has no /marker service).
set -u
ROOM=${1:-R1}; MODE=${2:-draw}
SIM=$HOME/starling_sim; GRID=$SIM/repo_export/scripts/qvix_grid.py
cam(){ # x y z qx qy qz qw  (world frame)
  docker exec starling-gz bash -c "gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 --req 'pose: {position: {x: $1, y: $2, z: $3}, orientation: {x: $4, y: $5, z: $6, w: $7}}'" >/dev/null
}
case "$MODE" in
  clear) python3 -I "$GRID" --room "$ROOM" --clear-markers | docker exec -i starling-gz bash ;;
  camera) ;;
  *) python3 -I "$GRID" --room "$ROOM" --markers | docker exec -i starling-gz bash
     echo "markers registered: $(docker exec starling-gz bash -c "gz service -s /marker/list --reqtype gz.msgs.Empty --reptype gz.msgs.Marker_V --timeout 3000 --req ''" 2>/dev/null | grep -c '^marker {')" ;;
esac
# camera presets: inside the room just under the ceiling, pitched 40 deg down, looking along the aisles
if [ "$ROOM" = "R1" ]; then cam 3.4 -1.5 4.7 0.2425 0.2425 -0.6642 0.6642      # north end, looking south
else                        cam 3.4 -0.3 4.7 0.2425 -0.2425 0.6642 0.6642; fi  # R2: south end, looking north
