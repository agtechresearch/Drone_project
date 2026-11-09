#!/bin/bash
# Copy the comparison runner to the VOXL (2026-11-04), run on the HOST while the reverse tunnel is open (127.0.0.1:22422).
# Writes ONLY /data/claude_collect/tools_safe/{ctl_patch.py, real_ctl_safe.py, ctl_run.sh, ctl_flight.sh, policy_runtime.py,
# policies/pd_imit.json, policies/sur_bcrand_s2.json}; then sha256 compare + python3 -m py_compile on the vehicle. No install,
# no config / param change, nothing started.   KH=<known_hosts file> bash real/deploy_ctl.sh
cd "$(dirname "$0")/.."; KH=${KH:?KH=<known_hosts file>}
S="ssh -o BatchMode=yes -o ConnectTimeout=5 -o UserKnownHostsFile=$KH -i $HOME/.ssh/id_ed25519_voxl -p 22422 root@127.0.0.1"
C="scp -o BatchMode=yes -o UserKnownHostsFile=$KH -i $HOME/.ssh/id_ed25519_voxl -P 22422"
T=/data/claude_collect/tools_safe
F="real/ctl_patch.py real/real_ctl_safe.py real/ctl_run.sh real/ctl_flight.sh rl_demo/policy_runtime.py"
P="surrogate/selected/pd_imit.json surrogate/selected/sur_bcrand_s2.json"
$S "mkdir -p $T/policies" || exit 1
$C $F root@127.0.0.1:$T/ || exit 1
$C $P root@127.0.0.1:$T/policies/ || exit 1
loc=$( (sha256sum $F | sed 's#  .*/#  #'; sha256sum $P | sed 's#  .*/#  policies/#') | sort -k2)
rem=$($S "cd $T && sha256sum ctl_patch.py real_ctl_safe.py ctl_run.sh ctl_flight.sh policy_runtime.py policies/pd_imit.json policies/sur_bcrand_s2.json" | sort -k2)
[ "$loc" = "$rem" ] && echo "sha256 match (7 files)" || { echo "!! sha256 MISMATCH"; diff <(echo "$loc") <(echo "$rem"); exit 2; }
$S "cd $T && python3 -m py_compile ctl_patch.py real_ctl_safe.py policy_runtime.py && bash -n ctl_run.sh && bash -n ctl_flight.sh && echo compile ok \
  && mkdir -p /data/claude_collect/ctl_selftest && cd /data/claude_collect/ctl_selftest \
  && python3 $T/real_ctl_safe.py --ctrl ppo --policy $T/policies/sur_bcrand_s2.json --log-dir /data/claude_collect/ctl_selftest/ctl \
     --v13 /home/root --result /data/claude_collect/ctl_selftest/mission_result.json -- --help 2>&1 | head -2"
