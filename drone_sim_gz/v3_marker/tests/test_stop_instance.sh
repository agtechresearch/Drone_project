#!/bin/bash
# Offline checks of stop_sitl_instance.sh exit codes (2026-10-31, codex mission-process-cleanup reply-2). A fake `docker` on PATH
# stands in for the containers, so no real process is touched. exit code = number of failures.
SC=$(cd "$(dirname "$0")/../scripts" && pwd); S=$(mktemp -d); F=0
t(){ # name expect_rc shim_body instance
  printf '#!/bin/bash\n%s\n' "$3" > "$S/docker"; chmod +x "$S/docker"
  out=$(STOP_LOG=$S/stop.log PATH=$S:$PATH timeout 60 "$SC/stop_sitl_instance.sh" "$4" 2>&1); rc=$?
  if [ $rc = "$2" ]; then echo "PASS  $1 -> $rc"; else echo "FAIL  $1 -> $rc (expected $2): $out"; F=$((F+1)); fi; }
t "docker exec fails"            2 'echo "Error: No such container" >&2; exit 1' 5
t "query gives no OK line"       2 'for a in "$@"; do [ "$a" = python3 ] && { cat >/dev/null; exit 0; }; done; exit 0' 5
t "process survives KILL"        1 'for a in "$@"; do [ "$a" = python3 ] && { cat >/dev/null; echo "OK 1"; exit 0; }; done; echo OK' 5
t "flight process survives"      1 'for a in "$@"; do [ "$a" = python3 ] && { cat >/dev/null; echo "OK 0"; exit 0; }; done; echo 4242; echo OK' 5
t "everything gone"              0 'for a in "$@"; do [ "$a" = python3 ] && { cat >/dev/null; echo "OK 0"; exit 0; }; done; echo OK' 5
t "gz /proc scan could not read a process" 2 'for a in "$@"; do [ "$a" = python3 ] && { cat >/dev/null; echo "QUERY_ERROR 1: 4242: Permission denied"; exit 0; }; done; echo OK' 5
t "flight pgrep failed (rc > 1)"  2 'for a in "$@"; do [ "$a" = python3 ] && { cat >/dev/null; echo "OK 0"; exit 0; }; done; echo "QUERY_ERROR pgrep rc 2"' 5
t "instance 1 protected"         4 'exit 0' 1
t "instance 0 refused"           4 'exit 0' 0
t "instance 6 refused"           4 'exit 0' 6

# --- internal query paths (2026-11-01 codex mission-process-cleanup reply-8) -------------------------------------------------
# The docker stand-in now RUNS the command it is given (python3 /proc scan, bash flight loop) on a made-up process table
# (STOP_PROC) and with a stand-in pgrep, so the script's own ENOENT / EACCES / pgrep rc branches decide the result.
# PIDs are above pid_max (4194304): a kill reaches nobody.
cat > "$S/docker" <<'SH'
#!/bin/bash
[ "$1" = exec ] && shift; while [ "${1#-}" != "$1" ]; do [ "$1" = -e ] && shift; shift; done; shift   # options, container
echo "$*" >> "$S_CALLS"; exec "$@"
SH
chmod +x "$S/docker"
cat > "$S/pgrep" <<'SH'
#!/bin/bash
[ -n "$FAKE_PGREP_OUT" ] && echo "$FAKE_PGREP_OUT"; exit ${FAKE_PGREP_RC:-1}
SH
chmod +x "$S/pgrep"
export S_CALLS=$S/calls
mkp(){ # pid state environ cmdline [noenv|noaccess|nocmd|dironly]
  local d=$P/$1; mkdir -p $d
  [ "$5" = dironly ] && return
  echo "$1 (x) $2 1 1" > $d/stat
  [ "$5" = noenv ] || printf '%s\0' $3 > $d/environ
  [ "$5" = noaccess ] && chmod 000 $d/environ
  [ "$5" = nocmd ] || printf '%s\0' $4 > $d/cmdline; }
ti(){ # name expect_rc pgrep_rc pgrep_out ; process table already in $P
  out=$(STOP_PROC=$P STOP_LOG=$S/stop.log FAKE_PGREP_RC=$3 FAKE_PGREP_OUT="$4" PATH=$S:$PATH timeout 60 "$SC/stop_sitl_instance.sh" 5 2>&1); rc=$?
  if [ $rc = "$2" ]; then echo "PASS  internal: $1 -> $rc"; else echo "FAIL  internal: $1 -> $rc (expected $2): $out"; F=$((F+1)); fi
  chmod -R u+rwx $P 2>/dev/null; rm -rf $P; mkdir -p $P; }
P=$S/proc; mkdir -p $P
mkp 9000001 S "PATH=/bin GZ_PARTITION=starling_3" "gz sim other"; ti "other instance only, pgrep rc 1 (no match)" 0 1 ""
mkp 9000002 S "" "" noenv;                                ti "gz: environ ENOENT (process gone while read)" 0 1 ""
[ "$(id -u)" = 0 ] && echo "SKIP  internal: EACCES needs a non-root user" || {
mkp 9000003 S "GZ_PARTITION=starling_3" "gz sim" noaccess; ti "gz: environ EACCES -> QUERY_ERROR" 2 1 ""; }
[ "$(id -u)" = 0 ] || { mkp 9000004 Z "" "" noaccess;     ti "gz: zombie with unreadable environ is skipped" 0 1 ""; }
mkp 9000005 S "GZ_PARTITION=starling_5" "gz sim -s";       ti "gz: instance 5 process stays after TERM+KILL" 1 1 ""
mkp 9000006 S "PATH=/bin" "../bin/px4 -i 5 -w ../instance_5"; ti "gz: px4 -i 5 by command line stays" 1 1 ""
mkp 9000007 S "PATH=/bin" "../bin/px4 -i 55 -w x";         ti "gz: px4 -i 55 is not instance 5" 0 1 ""
ti "flight: pgrep rc 2 -> QUERY_ERROR" 2 2 ""
ti "flight: pgrep rc 3 -> QUERY_ERROR" 2 3 ""
mkp 9000010 S "PATH=/bin" "mavsdk_server -p 50056 udp://:14545"; ti "flight: mavsdk_server -p 50056 stays" 1 0 "9000010"
mkp 9000011 S "PATH=/bin" "mavsdk_server -p 50053 udp://:14542"; ti "flight: other port 50053 ignored" 0 0 "9000011"
mkp 9000012 S "PATH=/bin" "mavsdk_server -p 500561 x";      ti "flight: port 500561 is not 50056" 0 0 "9000012"
ti "flight: listed pid gone before its cmdline was read" 0 0 "9000013"
mkp 9000014 S "" "" nocmd;                                  ti "flight: cmdline unreadable, process still there -> QUERY_ERROR" 2 0 "9000014"
grep -q "QUERY_ERROR" "$S/stop.log" && echo "PASS  internal: QUERY_ERROR lines kept in the stop log" || { echo "FAIL  internal: no QUERY_ERROR in the stop log"; F=$((F+1)); }

# --- caller: wk_fly.sh must not boot when the stop query fails (QUERY_ERROR -> rc 2 -> exit 2, no run_wk_instance) ----------
H=$S/home; mkdir -p $H/starling_sim/v3_marker/logs; ln -s "$SC" $H/starling_sim/v3_marker/scripts
[ "$(id -u)" = 0 ] || { mkp 9000020 S "GZ_PARTITION=starling_5" "gz sim" noaccess
  : > "$S_CALLS"; out=$(HOME=$H INSTANCE=5 STOP_PROC=$P FAKE_PGREP_RC=1 PATH=$S:$PATH timeout 120 "$SC/wk_fly.sh" t_stop on 0 0 "R1C9:W@0.68 R1C1 R1C1@1.265 R1C9" 2>&1); rc=$?
  boots=$(grep -c "run_wk_instance" "$S_CALLS")
  if [ $rc = 2 ] && [ "$boots" = 0 ] && echo "$out" | grep -q "정리 실패 (rc 2)"; then echo "PASS  caller: wk_fly stop QUERY_ERROR -> exit $rc, boot calls $boots"
  else echo "FAIL  caller: wk_fly rc $rc, boot calls $boots: $(echo "$out" | tail -3 | tr '\n' ' ')"; F=$((F+1)); fi
  chmod -R u+rwx $P; }
rm -rf "$S"; echo "$F failure(s)"; exit $F
