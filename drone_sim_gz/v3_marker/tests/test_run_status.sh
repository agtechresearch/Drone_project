#!/bin/bash
# Offline check of run_status.sh (2026-10-31, codex runner-exit-status): fake run dirs, no flight.
SC=$(cd "$(dirname "$0")/../scripts" && pwd); T=$(mktemp -d); F=0
case_(){ # name expect_rc mission_rc verify_rc world archive log_text [truth verdict]
  mkdir -p "$T/$1"; printf '%s\n' "$7" > "$T/$1/mission.log"
  "$SC/run_status.sh" "$T/$1" "$3" "$4" "$5" "$6" "${8:-}" > /dev/null; rc=$?
  if [ "$rc" = "$2" ]; then echo "PASS  $1 -> $rc  $(cat "$T/$1/status.json")"; else echo "FAIL  $1 -> $rc (expected $2)"; F=$((F+1)); fi
}
case_ ok_qvix            0  0 0 qvix     ok      "Done!"
case_ ok_turned_world    0  0 1 qvix_rot ok      "Done!" PASS
case_ turned_no_verdict  14 0 1 qvix_rot ok      "Done!"
case_ turned_incomplete  14 0 0 qvix_rot ok      "Done!" INCOMPLETE
case_ turned_error       14 0 0 qvix_sv1 ok      "Done!" error
case_ turned_contact     11 0 0 qvix_rot ok      "Done!" FAIL
case_ mission_rc         10 1 0 qvix_rot ok      "Done!" PASS
case_ traceback          10 0 0 qvix_rot ok      "Traceback (most recent call last):" PASS
case_ stop_on_fail       10 0 0 qvix_rot ok      "!!! stop_on_fail: s01 did not pass." PASS
case_ no_yaw             10 0 0 qvix_rot ok      "[wk] s00: no yaw feedback -> turn NOT confirmed (failure) NO_YAW_FEEDBACK" PASS
case_ safety_landing     13 0 0 qvix_rot ok      "!!! EMERGENCY LAND TRIGGERED: ToF blind for 5 s !!!" PASS
case_ landing_aborts_stage 13 0 0 qvix_rot ok "$(printf '%s\n' '!!! EMERGENCY LAND TRIGGERED: ToF blind !!!' '[yaw scan6] [END] YAW_TIMEOUT reason=ABORTED' 'scan6 YAW YAW_TIMEOUT')"
case_ crash_then_landing 10 0 0 qvix_rot ok "$(printf '%s\n' '[wk] correction/safety task crashed: ValueError' 'Traceback (most recent call last):' '!!! EMERGENCY LAND TRIGGERED: correction task crashed !!!')"
case_ verify_failed      11 0 1 qvix     ok      "Done!"
case_ archive_failed     12 0 0 qvix_rot failed  "Done!" PASS
case_ landing_not_confirmed 10 0 0 qvix_rot ok "$(printf '%s\n' '!!! EMERGENCY LAND TRIGGERED: ToF blind !!!' '!!! TOUCHDOWN NOT CONFIRMED: landing step not finished within 40 s (no in_air message?) - not disarmed !!!')" PASS
case_ land_never_sent     10 0 0 qvix_rot ok "$(printf '%s\n' '!!! EMERGENCY LAND TRIGGERED: stop requested !!!' '       land NOT sent after 3 tries - motors NOT killed (WK_ALLOW_KILL=0)')" PASS
# 2026-11-01 codex decisions reply-7 S3: the safety layer's result file not written
case_ result_file_not_written 16 0 0 qvix_rot ok "$(printf '%s\n' 'Done!' '[result] !!! RESULT FILE NOT WRITTEN: result file x not written: PermissionError !!!')" PASS
case_ result_file_with_safety_landing 13 0 0 qvix_rot ok "$(printf '%s\n' '!!! EMERGENCY LAND TRIGGERED: ToF blind !!!' '[result] !!! RESULT FILE NOT WRITTEN: x !!!')" PASS
grep -q '"result_file": "not_written"' $T/result_file_with_safety_landing/status.json && echo "PASS    status.json keeps result_file not_written under exit 13" \
  || { echo "FAIL    result_file field lost under exit 13"; F=$((F+1)); }
mkdir -p "$T/ro"; echo "Done!" > "$T/ro/mission.log"; chmod 555 "$T/ro"; "$SC/run_status.sh" "$T/ro" 0 0 qvix ok > /dev/null 2>&1; rc=$?; chmod 755 "$T/ro"
[ $rc = 15 ] && echo "PASS  status.json cannot be written -> 15" || { echo "FAIL  status.json cannot be written -> $rc"; F=$((F+1)); }
mkdir -p "$T/no_log"; "$SC/run_status.sh" "$T/no_log" 0 0 qvix_rot ok >/dev/null; rc=$?
[ $rc = 10 ] && echo "PASS  missing mission.log -> 10" || { echo "FAIL  missing mission.log -> $rc"; F=$((F+1)); }
rm -rf "$T"; # the result file checked against the logged run id (codex decisions reply-9)
rid_case(){ # name expect_rc json(content or -) extra_log
  mkdir -p "$T/$1"; printf '%s\n' "[result] run id R1, result file x" "Done!" "$4" > "$T/$1/mission.log"
  [ "$3" != - ] && printf '%s' "$3" > "$T/$1/mission_result.json"
  "$SC/run_status.sh" "$T/$1" 0 0 qvix_rot ok PASS > /dev/null; rc=$?
  if [ "$rc" = "$2" ]; then echo "PASS  $1 -> $rc  $(grep -o '"result_file": "[^"]*"' "$T/$1/status.json")"; else echo "FAIL  $1 -> $rc (expected $2)"; F=$((F+1)); fi; }
rid_case result_json_ok          0  '{"run_id": "R1"}'  ""
rid_case result_json_missing     16 -                   ""
rid_case result_json_other_run   16 '{"run_id": "R0"}'  ""
rid_case result_json_unreadable  16 '{"run_id": '       ""
rid_case result_json_ok_but_not_written_line 16 '{"run_id": "R1"}' "[result] !!! RESULT FILE NOT WRITTEN: end step interrupted !!!"
echo "$F failure(s)"; exit $F
