#!/bin/bash
# 현수막 모델·월드를 PX4 gz 트리에 넣는다 (starling-gz 컨테이너 안에서 실행. 빌드 불필요).
#   install_banner_world.sh [gz 출력 폴더 = ../gz] [PX4 트리 = ~/PX4-Autopilot]
# 복사 대상 (이 이름만 건드린다; 다른 모델·월드는 손대지 않음)
#   $PX4/Tools/simulation/gz/models/drift_banner/      ← <out>/models/drift_banner/
#   $PX4/Tools/simulation/gz/worlds/drift_banner.sdf   ← <out>/worlds/drift_banner.sdf
# run_wk_instance.sh 는 PX4_GZ_WORLD=<이름> 으로 px4 를 직접 띄우므로 CMake 등록 없이 worlds/ 에 파일만 있으면 된다
# (v3_marker 의 qvix_sv*.sdf 와 같은 방식).
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-$HERE/../gz}; PX4=${2:-$HOME/PX4-Autopilot}
NAME=${NAME:-drift_banner}
GZ=$PX4/Tools/simulation/gz
for p in "$OUT/models/$NAME/model.sdf" "$OUT/models/$NAME/model.config" "$OUT/worlds/$NAME.sdf" "$GZ/models" "$GZ/worlds"; do
  [ -e "$p" ] || { echo "!! 없음: $p"; exit 1; }
done
if command -v gz >/dev/null 2>&1; then
  # model:// URI 해석을 위해 검사 중에만 모델 경로를 준다 (현수막 = $OUT/models, qvix 등 = PX4 트리)
  export GZ_SIM_RESOURCE_PATH="$OUT/models:$GZ/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
  export SDF_PATH="$GZ_SIM_RESOURCE_PATH${SDF_PATH:+:$SDF_PATH}"
  gz sdf -k "$OUT/models/$NAME/model.sdf" >/dev/null && echo "sdf ok: model" || { echo "!! model.sdf 검사 실패"; exit 1; }
  gz sdf -k "$OUT/worlds/$NAME.sdf" >/dev/null && echo "sdf ok: world" || { echo "!! $NAME.sdf 검사 실패"; exit 1; }
fi
rm -rf "$GZ/models/$NAME.new"; cp -r "$OUT/models/$NAME" "$GZ/models/$NAME.new"
rm -rf "$GZ/models/$NAME"; mv "$GZ/models/$NAME.new" "$GZ/models/$NAME"
cp "$OUT/worlds/$NAME.sdf" "$GZ/worlds/.$NAME.sdf.tmp"; mv "$GZ/worlds/.$NAME.sdf.tmp" "$GZ/worlds/$NAME.sdf"
echo "installed: $GZ/models/$NAME ($(ls "$GZ/models/$NAME/materials/textures" | wc -l) textures), $GZ/worlds/$NAME.sdf"
echo "run     : run_wk_instance.sh <N> $NAME headless 0,0,0.035,0,0,1.5708   (starling2_tof, front_camera_real)"
