#!/bin/bash
# PC → 연구실 시뮬 서버로 패키지·진입 스크립트·시뮬 도구를 올린다 (tar over ssh, deploy_pkg.sh 와 같은 방식).
#   SIM_HOST=user@server ./sim/marker_drift_gz/sync_to_server.sh [원격 폴더 = ~/starling_sim/marker_drift]
# 올라가는 것 (원격 폴더 아래)
#   flight/s2flight/               ← starling2_autonomy/flight/s2flight/
#   flight/marker_drift_flight.py  ← starling2_autonomy/flight/marker_drift_flight.py
#   sim/                           ← starling2_autonomy/sim/marker_drift_gz/  (이 폴더)
# 원격의 gz/ (생성물) 과 logs/ 는 건드리지 않는다. 기존 flight/, sim/ 은 덮어쓴다 (백업 tgz 하나 남김).
set -eu
: "${SIM_HOST:?SIM_HOST=user@host 를 줄 것 (.env.local 의 서버 항목)}"
REMOTE=${1:-'~/starling_sim/marker_drift'}
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT"
python3 -m compileall -q flight/s2flight flight/marker_drift_flight.py sim/marker_drift_gz >/dev/null || python -m compileall -q flight/s2flight flight/marker_drift_flight.py sim/marker_drift_gz
echo "→ $SIM_HOST:$REMOTE"
tar czf - --exclude='__pycache__' --exclude='*.pyc' flight/s2flight flight/marker_drift_flight.py sim/marker_drift_gz \
  | ssh "$SIM_HOST" "set -e; mkdir -p $REMOTE; cd $REMOTE; [ -d flight ] && tar czf flight.bak.\$(date +%Y%m%d_%H%M%S).tgz flight sim 2>/dev/null || true; rm -rf flight/s2flight sim; tar xzf -; mkdir -p flight; mv sim/marker_drift_gz sim_tmp && rm -rf sim && mv sim_tmp sim; chmod +x sim/*.sh; python3 -m compileall -q flight/s2flight flight/marker_drift_flight.py || true; echo remote: \$(pwd); ls flight sim"
echo "다음: ssh $SIM_HOST  →  cd $REMOTE && python3 sim/make_banner_world.py  →  docker exec starling-gz $REMOTE/sim/install_banner_world.sh  →  sim/run_sim_drift.sh --no-align --distance 0.5"
