#!/bin/bash
# Change a PX4 parameter of SITL instances for a batch and ALWAYS put the old values back (2026-11-01, codex
# experiment-provenance reply-4: the phase 9 scripts restored only on a normal end, without checks).
#   px4_param_guard.sh recover                      before a batch: a pending restore left by a DEAD batch is done first; a pending
#                                                   record whose owner batch still runs is refused (exit 1) - two batches overlap
#   px4_param_guard.sh set <param> <value> "2 3 4 5"  reads every old value first (any read failure: nothing is changed,
#                                                   exit 1), writes logs/.px4_param_pending, then sets + saves + reads back
#   px4_param_guard.sh restore                      sets the saved old values, saves, reads back; exit 1 on any mismatch
#   px4_param_guard.sh recover-legacy               like recover for a record WITHOUT the owner line (old format). Procedure
#                                                   (codex experiment-provenance / decisions reply-9): (1) no batch may run -
#                                                   the command itself refuses while a process matches BATCH_PAT (run_phase*.sh,
#                                                   wk_tofh_batch.sh, run_real_*sitl.sh); (2) start no batch until it printed
#                                                   "restored" for every row and exited 0 - the guard lock covers the command,
#                                                   not a batch's lifetime, so this ordering is the operator's job; (3) keep the
#                                                   .done_<time> record it leaves
# Use in a batch script (logs/run_phase12.sh): the exit / INT / TERM trap is installed BEFORE "set", so every ending after the
# change restores; a batch killed with SIGKILL is restored by the next batch's "recover". The guard itself holds a lock while it
# runs; set refuses while an unrestored change is pending.
# Owner (2026-11-01 codex experiment-provenance reply-8): the pending record's first line is "# owner <pid> <start> <cmd>", the
# batch process that ran "set" (PX4_GUARD_OWNER, default the caller = $PPID; <start> = /proc/<pid>/stat field 22, so a reused
# pid is not the owner). restore / recover act only when the owner is this caller or no longer runs. The record is checked
# BEFORE anything is restored (readable, rows "<instance 2..5> <PARAM> <number>", at least one) and closed (.done_<time>) only
# after every row was restored and read back.
LOGS=${PX4_GUARD_LOGS:-$(cd "$(dirname "$0")/.." && pwd)/logs}; PEND=$LOGS/.px4_param_pending
INST_OK=" 2 3 4 5 "                 # instances a batch may change (0 / 1 belong to other work)
# one owner at a time (2026-11-01 codex experiment-provenance reply-6): the whole command runs under this lock
exec 8>"$LOGS/.px4_param_guard.lock"; flock -w 30 8 || { echo "!! px4_param_guard: another batch holds the lock"; exit 1; }
BIN="cd /home/dronesim/PX4-Autopilot/build/px4_sitl_default/bin"
OWNER=${PX4_GUARD_OWNER:-$PPID}
PROC=${PX4_GUARD_PROC:-/proc}       # tests point this at a fake tree (an owner whose start time can not be read)
pstart(){ awk '{print $22}' "$PROC/$1/stat" 2>/dev/null; }
# the docker / px4-param rc is checked: a failed query is no value (codex reply-8)
get(){ local out; out=$(docker exec starling-gz bash -c "$BIN; ./px4-param --instance $1 show $2" 2>/dev/null) || return 1
       echo "$out" | grep -o "$2 .*: *[-0-9.]*" | grep -o '[-0-9.]*$'; }
# set and save checked on their own: a value that is set but not saved would come back after a restart
put(){ docker exec starling-gz bash -c "$BIN; ./px4-param --instance $1 set $2 $3 >/dev/null 2>&1" || { echo "!! px4_param_guard: set i$1 $2 $3 failed"; return 1; }
       docker exec starling-gz bash -c "$BIN; ./px4-param --instance $1 save >/dev/null 2>&1" || { echo "!! px4_param_guard: save i$1 failed"; return 1; }; }
ROWS=(); LEGACY=0
# owner state of the record (2026-11-01 codex experiment-provenance reply-10): alive (same pid AND same start time) | gone (no
# such pid, or the pid now belongs to a process with another start time = reused) | unknown (/proc/<pid> exists but its start
# time can not be read) -> unknown is NOT taken as gone: the record is kept
owner_state(){ local pid=$1 start=$2
  [ -d "$PROC/$pid" ] || { echo gone; return; }
  local now; now=$(pstart "$pid"); [ -n "$now" ] || { echo unknown; return; }
  [ "$now" = "$start" ] && echo alive || echo gone; }
# read + check the whole record first; owner check: rc 0 = may restore, 1 = invalid / unreadable / owner unknown, 3 = owned by a
# running batch. The first line must be the one "# owner <pid> <start> <cmd>" line; a record without it (written before the
# owner line existed) is restored only by the explicit "recover-legacy" command (LEGACY=1), with every other check.
load(){
  ROWS=(); [ -r "$PEND" ] || { echo "!! px4_param_guard: $PEND exists but cannot be read - nothing restored, record kept"; return 1; }
  local lines; mapfile -t lines < "$PEND" || { echo "!! px4_param_guard: reading $PEND failed - nothing restored, record kept"; return 1; }
  local l k=0 nown=0 op="" os=""
  for l in "${lines[@]}"; do
    k=$((k+1))
    if [[ $l == "#"* ]]; then
      if [[ $l =~ ^#\ owner\ ([0-9]+)\ ([0-9]+)(\ .*)?$ ]] && [ $k = 1 ]; then
        nown=$((nown+1)); op=${BASH_REMATCH[1]}; os=${BASH_REMATCH[2]}; continue
      fi
      echo "!! px4_param_guard: $PEND line $k is not a valid first-line owner record: '$l' - nothing restored, record kept"; return 1
    fi
    set -- $l
    if [ $# != 3 ] || [[ " $INST_OK " != *" $1 "* ]] || ! [[ $2 =~ ^[A-Z0-9_]+$ ]] || ! [[ $3 =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
      echo "!! px4_param_guard: $PEND line $k is not '<instance 2..5> <PARAM> <number>': '$l' - nothing restored, record kept"; return 1
    fi
    ROWS+=("$l")
  done
  [ ${#ROWS[@]} -gt 0 ] || { echo "!! px4_param_guard: $PEND has no value to restore - record kept"; return 1; }
  if [ $nown = 0 ]; then
    [ $LEGACY = 1 ] && { echo "px4_param_guard: record without an owner line restored by recover-legacy"; return 0; }
    echo "!! px4_param_guard: $PEND has no owner line (old format) - nothing restored; after checking that no batch runs use: $0 recover-legacy"; return 1
  fi
  [ "$op" = "$OWNER" ] && [ "$(pstart "$OWNER")" = "$os" ] && return 0          # the owner itself (its exit trap)
  case $(owner_state "$op" "$os") in
    alive)   echo "!! px4_param_guard: the pending change belongs to a batch that still runs (pid $op) - nothing restored"; return 3;;
    unknown) echo "!! px4_param_guard: owner pid $op exists but its start time can not be read - owner unknown, nothing restored, record kept"; return 1;;
  esac
  return 0
}
restore(){
  [ -e "$PEND" ] || { echo "px4_param_guard: nothing to restore"; return 0; }
  load; local lr=$?; [ $lr = 0 ] || return 1
  local bad=0 row n p v now
  for row in "${ROWS[@]}"; do
    read -r n p v <<< "$row"
    put "$n" "$p" "$v" || { bad=1; continue; }; now=$(get "$n" "$p")
    if [ "$now" = "$v" ]; then echo "px4_param_guard: restored i$n $p = $now"; else echo "!! px4_param_guard: i$n $p is '$now', expected $v"; bad=1; fi
  done
  if [ $bad = 0 ]; then mv "$PEND" "$PEND.done_$(date +%Y%m%d_%H%M%S)" || { echo "!! restored, but $PEND could not be closed"; bad=1; }
  else echo "!! restore incomplete - $PEND kept"; fi
  return $bad
}
case "$1" in
  recover) [ -e "$PEND" ] && { echo "px4_param_guard: a previous batch did not restore - restoring now"; restore || exit 1; }; exit 0;;
  recover-legacy) LEGACY=1
       BATCH_PAT=${PX4_GUARD_BATCH_PAT:-'run_phase[0-9a-z]*\.sh|wk_tofh_batch\.sh|run_real_(voxl_)?sitl\.sh'}
       b=$(pgrep -af "$BATCH_PAT" | grep -v "px4_param_guard" | head -3)
       [ -z "$b" ] || { echo "!! px4_param_guard: a batch runs - recover-legacy refused (nothing restored): $b"; exit 1; }
       [ -e "$PEND" ] && { restore || exit 1; }; exit 0;;
  set) P=$2; V=$3; INST=$4
       [ -e "$PEND" ] && { echo "!! px4_param_guard: an earlier change is not restored ($PEND) - run recover first; nothing changed"; exit 1; }
       for n in $INST; do [[ "$INST_OK" == *" $n "* ]] || { echo "!! px4_param_guard: instance $n is not one of$INST_OK- nothing changed"; exit 1; }; done
       os=$(pstart "$OWNER"); [ -n "$os" ] || { echo "!! px4_param_guard: owner pid $OWNER not found - nothing changed"; exit 1; }
       echo "# owner $OWNER $os $(tr '\0' ' ' < /proc/$OWNER/cmdline 2>/dev/null | cut -c1-120)" > "$PEND.new" || { echo "!! px4_param_guard: cannot write $PEND.new - nothing changed"; exit 1; }
       for n in $INST; do o=$(get "$n" "$P"); [ -n "$o" ] || { echo "!! px4_param_guard: cannot read i$n $P - nothing changed"; rm -f "$PEND.new"; exit 1; }
         echo "$n $P $o" >> "$PEND.new" || { echo "!! px4_param_guard: cannot write $PEND.new - nothing changed"; rm -f "$PEND.new"; exit 1; }; done
       # the restore record must exist BEFORE the first change
       mv "$PEND.new" "$PEND" && [ -s "$PEND" ] || { echo "!! px4_param_guard: cannot publish $PEND - nothing changed"; exit 1; }
       grep -v '^#' "$PEND" | sed 's/^/px4_param_guard: before: i/'
       for n in $INST; do put "$n" "$P" "$V" && [ "$(get "$n" "$P")" = "$V" ] || { echo "!! px4_param_guard: i$n $P not set to $V - restoring"; restore; exit 1; }; done
       echo "px4_param_guard: $P = $V on $INST (owner pid $OWNER)";;
  restore) restore; exit $?;;
  *) echo "usage: $0 recover | recover-legacy | set <param> <value> \"<instances>\" | restore"; exit 2;;
esac
