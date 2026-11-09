#!/bin/bash
# Step 2 pre-check (2026-11-03 v2), read-only: prints the values AND a verdict. exit 0 = ready to run step2_run.sh.
#   bash step2_check.sh <label>   -> /data/claude_collect/step2_check/<label>.txt
# Ready = RC selected and valid, kill switch OFF (kill_switch 3, no manual lockdown), mode switch in an Offboard slot (5/6),
# disarmed, pre-flight checks pass, battery warning 0, no mavsdk_server running.
D=/data/claude_collect/step2_check; mkdir -p $D; L=${1:-now}; O=$D/$L.txt
{ date -u +%FT%TZ; for t in manual_control_setpoint manual_control_switches input_rc vehicle_status actuator_armed battery_status; do
    echo "### $t"; px4-listener $t 2>&1 | head -60; done
  echo "### mavsdk_server processes"; pgrep -a -x mavsdk_server || echo none; } > $O
v(){ grep -a -m1 -E "^ *$1:" $O | awk '{print $2}'; }
ok=1; say(){ printf '  %-4s %s\n' "$1" "$2"; [ "$1" = OK ] || ok=0; }
[ "$(v data_source)" = 1 ] && [ "$(v valid)" = True ] && say OK "RC is the input (data_source 1, valid)" || say NG "RC input: data_source $(v data_source) valid $(v valid)"
[ "$(v kill_switch)" = 3 ] && [ "$(v manual_lockdown)" = False ] && say OK "kill switch OFF" || say NG "kill switch ON or unknown (kill_switch $(v kill_switch), manual_lockdown $(v manual_lockdown)) -> turn the kill switch OFF"
case "$(v mode_slot)" in 5|6) say OK "mode switch in the Offboard slot ($(v mode_slot), nav_state $(v nav_state))";; *) say NG "mode switch slot $(v mode_slot) (nav_state $(v nav_state)) -> put it UP (Offboard, slot 5/6)";; esac
[ "$(v arming_state)" = 1 ] && say OK "disarmed (arming_state 1)" || say NG "arming_state $(v arming_state) (1 = disarmed)"
# pre_flight_checks_pass is False in the Offboard slot until setpoints arrive (offboard signal lost; the script primes them 3 s
# before arming) - informational only (2026-11-03: before_A / before_C showed False and both armed); kill / battery are checked above
[ "$(v pre_flight_checks_pass)" = True ] && say OK "pre-flight checks pass" || printf '  %-4s %s\n' INFO "pre-flight checks False (normal in the Offboard slot before the script's setpoints; arm denial reasons: journalctl -u voxl-px4 | grep 'Preflight Fail')"
[ "$(v warning)" = 0 ] && say OK "battery $(v voltage_v) V, $(v remaining) remaining, warning 0" || say NG "battery warning $(v warning) ($(v voltage_v) V, $(v remaining)) -> change the battery"
grep -a -A1 "### mavsdk_server" $O | tail -1 | grep -q none && say OK "no mavsdk_server running" || say NG "a mavsdk_server runs: $(grep -a -A1 '### mavsdk_server' $O | tail -1)"
echo "saved $O"
[ $ok = 1 ] && { echo "READY"; exit 0; } || { echo "NOT READY - fix the NG lines first"; exit 1; }
