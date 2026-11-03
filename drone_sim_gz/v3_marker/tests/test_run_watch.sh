#!/bin/bash
# Offline checks of run_watch.py (2026-11-01, codex batch-watch-run-lifecycle / batch-watch-observer-failure): fake run dirs written
# on a timetable, every deadline x0.02 (RUN_WATCH_SCALE), a fake `docker`. exit code = number of failures.
SC=$(cd "$(dirname "$0")/../scripts" && pwd); F=0
mk(){ # dir lane: a run that boots and starts its mission
  mkdir -p "$1"; echo "$1" > "$L/.last_run_i$2"; sleep 0.2; echo "New NED origin" > "$1/sitl.log"; sleep 0.2; echo "[grid_mission] asyncio clock" > "$1/mission.log"; }
fin(){ printf '{"exit": %s, "mission": "ok", "verify": "ok", "truth_verdict": "PASS"}' "$2" > "$1/status.json"; }
t(){ # name expect_rc runs_per_lane scenario_function [docker body]
  L=$(mktemp -d); B=tw; S=$(mktemp -d); printf '#!/bin/bash\n%s\n' "${5:-echo starling-gz; echo starling-flight}" > $S/docker; chmod +x $S/docker
  ( $4 ) & W=$!
  out=$(PATH=$S:$PATH RUN_WATCH_LOGS=$L RUN_WATCH_SCALE=0.02 RUN_WATCH_POLL=0.1 timeout 60 python3 "$SC/run_watch.py" $B 2 45 $3 2>&1); rc=$?
  wait $W 2>/dev/null
  if [ $rc = "$2" ]; then echo "PASS  $1 -> rc $rc"; else echo "FAIL  $1 -> rc $rc (expected $2)"; F=$((F+1)); fi
  echo "$out" | sed 's/^/        /' | tail -${6:-6}; WJ=$L/${B}_watch.json; [ -f $WJ ] && cp $WJ /tmp/_rw_last.json; rm -rf $L $S; }
two_runs_second_stalls(){ mk $L/r1_b${B}_x 2; sleep 0.6; fin $L/r1_b${B}_x 0; sleep 1.0
  mk $L/r2_b${B}_x 2; sleep 4; echo "$L/r2_b${B}_x 2" >> $L/tofh_${B}_status.txt; }      # lane gives up without status.json
second_never_comes(){ mk $L/r1_b${B}_x 2; sleep 0.6; fin $L/r1_b${B}_x 0; }
half_status(){ mk $L/r1_b${B}_x 2; sleep 0.5; printf '{"exit": ' > $L/r1_b${B}_x/status.json; sleep 1.5; fin $L/r1_b${B}_x 0; }
plain(){ mk $L/r1_b${B}_x 2; sleep 1.5; fin $L/r1_b${B}_x 13; }
bad_plan(){ echo '{"runs": [{"seq": 1}]}' > $L/${B}_plan.json; }
t "lane with 2 runs: run 1 ok, run 2 stalls and ends without status -> watcher waits for run 2" 1 2 two_runs_second_stalls
python3 -c "import json;d=json.load(open('/tmp/_rw_last.json'));r=d['runs'];assert d['end']=='finished' and [x['end'] for x in r]==['finished','failed'] and 'progress' in r[1]['failed_at_s'],r" \
  && echo "PASS    watch.json: run 1 finished, run 2 failed with a stall" || { echo "FAIL    watch.json content"; F=$((F+1)); }
t "lane with 2 runs: run 2 never appears -> missing, not a normal end" 1 2 second_never_comes
python3 -c "import json;d=json.load(open('/tmp/_rw_last.json'));assert d['missing']==[{'lane':'2','seq':2}],d['missing']" \
  && echo "PASS    watch.json: missing lane 2 run 2" || { echo "FAIL    watch.json missing"; F=$((F+1)); }
t "status.json half written for a while -> retried, then finished" 0 1 half_status
python3 -c "import json;d=json.load(open('/tmp/_rw_last.json'));assert d['observer']['read_errors']>0 and d['runs'][0]['exit']==0,d['observer']" \
  && echo "PASS    watch.json: read errors recorded, exit 0 read" || { echo "FAIL    read errors"; F=$((F+1)); }
t "docker ps hangs -> recorded as query failure, the run is still watched" 0 1 plain 'sleep 30' 3
python3 -c "import json;d=json.load(open('/tmp/_rw_last.json'));assert d['observer']['docker_errors']>0 and d['runs'][0]['exit']==13,d['observer']" \
  && echo "PASS    watch.json: docker errors recorded, run exit 13 read" || { echo "FAIL    docker errors"; F=$((F+1)); }
t "the watcher itself crashes (bad plan file) -> exit 3, end observer_error" 3 1 bad_plan
python3 -c "import json;d=json.load(open('/tmp/_rw_last.json'));assert d['end'].startswith('observer_error'),d['end']" \
  && echo "PASS    watch.json: end observer_error" || { echo "FAIL    observer_error not recorded"; F=$((F+1)); }
# ---- 2026-11-01 codex reply-2: plan keys, fast transitions, closed stdout
plan2(){ printf '{"runs": [{"lane": 2, "seq": 1, "tag": "b%s_A"}, {"lane": 2, "seq": 2, "tag": "b%s_B"}]}' $B $B > $L/${B}_plan.json; }
wrong_tag(){ plan2; mk $L/t1_i2_b${B}_A 2; sleep 0.4; fin $L/t1_i2_b${B}_A 0; sleep 0.4; mk $L/t2_i2_b${B}_C 2; sleep 0.4; fin $L/t2_i2_b${B}_C 0; }
duplicate(){ plan2; mk $L/t1_i2_b${B}_A 2; sleep 0.4; fin $L/t1_i2_b${B}_A 0; sleep 0.3; mk $L/t2_i2_b${B}_A 2; fin $L/t2_i2_b${B}_A 0; sleep 0.3
  mk $L/t3_i2_b${B}_B 2; sleep 0.3; fin $L/t3_i2_b${B}_B 0; }
fast(){ :; }                                        # both runs exist before the watcher starts (prepared below)
t "plan A,B; lane runs A then C (wrong tag) -> C unexpected, B missing" 2 1 wrong_tag
python3 -c "import json;d=json.load(open('/tmp/_rw_last.json'));assert len(d['unexpected_runs'])==1 and d['unexpected_runs'][0].endswith('_C') and d['missing']==[{'lane':'2','seq':2}],(d['unexpected_runs'],d['missing'])" \
  && echo "PASS    watch.json: C listed as unexpected, lane 2 seq 2 missing" || { echo "FAIL    wrong-tag JSON"; F=$((F+1)); }
t "plan A,B; runs A, A again, B -> second A unexpected, A and B finished" 1 1 duplicate
python3 -c "import json;d=json.load(open('/tmp/_rw_last.json'));e=d['expected_runs'];assert [x['run'] is not None for x in e]==[True,True] and len(d['unexpected_runs'])==1,(e,d['unexpected_runs'])" \
  && echo "PASS    watch.json: both planned runs matched, one duplicate unexpected" || { echo "FAIL    duplicate JSON"; F=$((F+1)); }
# fast transition: two finished runs before the first poll, .last_run_i2 names only the second one
L=$(mktemp -d); B=tw; S=$(mktemp -d); printf '#!/bin/bash
echo starling-gz; echo starling-flight
' > $S/docker; chmod +x $S/docker; plan2
for x in A B; do mkdir -p $L/t_${x}_i2_b${B}_$x; echo "New NED origin" > $L/t_${x}_i2_b${B}_$x/sitl.log; echo "asyncio clock" > $L/t_${x}_i2_b${B}_$x/mission.log; fin $L/t_${x}_i2_b${B}_$x 0; done
echo $L/t_B_i2_b${B}_B > $L/.last_run_i2
PATH=$S:$PATH RUN_WATCH_LOGS=$L RUN_WATCH_SCALE=0.02 RUN_WATCH_POLL=0.1 timeout 60 python3 "$SC/run_watch.py" $B 2 45 1 > /dev/null 2>&1; rc=$?
python3 -c "import json;d=json.load(open('$L/${B}_watch.json'));e=d['expected_runs'];assert all(x['run'] for x in e) and not d['missing'],e" && [ $rc = 0 ] \
  && echo "PASS  both runs passed before the first poll -> both found (rc $rc)" || { echo "FAIL  fast transition (rc $rc)"; F=$((F+1)); }
rm -rf $L $S
# stdout broken (the reader of the pipe is gone): the watch goes on, the JSON is complete
L=$(mktemp -d); S=$(mktemp -d); printf '#!/bin/bash
echo starling-gz; echo starling-flight
' > $S/docker; chmod +x $S/docker
( mk $L/r1_b${B}_x 2; sleep 0.6; fin $L/r1_b${B}_x 0 ) &
# the reader of the pipe is gone at once (`| true`): every print raises BrokenPipeError
PATH=$S:$PATH RUN_WATCH_LOGS=$L RUN_WATCH_SCALE=0.02 RUN_WATCH_POLL=0.1 timeout 60 python3 "$SC/run_watch.py" $B 2 45 1 2>/dev/null | true; rc=${PIPESTATUS[0]}; wait
python3 -c "import json;d=json.load(open('$L/${B}_watch.json'));assert d['end']=='finished' and d['observer'].get('stdout_failed',0)>0 and d['runs'][0]['end']=='finished',d['observer']" && [ $rc = 0 ] \
  && echo "PASS  stdout closed -> still watched to the end, stdout failures counted, JSON finished (rc $rc)" || { echo "FAIL  closed stdout (rc $rc)"; F=$((F+1)); }
rm -rf $L $S
# the same run dir under two spellings (glob through a ../ path, .last_run absolute) is ONE run (batch 1916p12 regression)
L=$(mktemp -d); S=$(mktemp -d); printf '#!/bin/bash\necho starling-gz; echo starling-flight\n' > $S/docker; chmod +x $S/docker; mkdir -p $L/sub
printf '{"runs": [{"lane": 2, "seq": 1, "tag": "b%s_A"}]}' $B > $L/${B}_plan.json
d=$L/t_i2_b${B}_A; mkdir -p $d; echo "New NED origin" > $d/sitl.log; echo "asyncio clock" > $d/mission.log; fin $d 0; echo $d > $L/.last_run_i2
PATH=$S:$PATH RUN_WATCH_LOGS=$L/sub/.. RUN_WATCH_SCALE=0.02 RUN_WATCH_POLL=0.1 timeout 60 python3 "$SC/run_watch.py" $B 2 45 1 > /dev/null 2>&1; rc=$?
python3 -c "import json;d=json.load(open('$L/${B}_watch.json'));assert not d['unexpected_runs'] and d['expected_runs'][0]['run'],d['unexpected_runs']" && [ $rc = 0 ] \
  && echo "PASS  same dir via LOGS=.../sub/.. and an absolute .last_run -> one run, nothing unexpected (rc $rc)" || { echo "FAIL  path spellings (rc $rc)"; F=$((F+1)); }
rm -rf $L $S
# ---- 2026-11-01 codex batch-watch-run-lifecycle reply-4: whole-tag match, seq contract
plan_th(){ printf '{"runs": [{"lane": 2, "seq": 1, "tag": "b%s_th1"}]}' $B > $L/${B}_plan.json; }
prefix(){ plan_th; mk $L/t1_i2_b${B}_th10 2; sleep 0.4; fin $L/t1_i2_b${B}_th10 0; }
t "plan th1; the lane runs th10 (th1 is a prefix of it) -> th10 unexpected, th1 missing" 2 1 prefix
python3 -c "import json;d=json.load(open('/tmp/_rw_last.json'));assert [u.endswith('_th10') for u in d['unexpected_runs']]==[True] and d['missing']==[{'lane':'2','seq':1}],(d['unexpected_runs'],d['missing'])" \
  && echo "PASS    watch.json: th10 unexpected, th1 missing" || { echo "FAIL    prefix JSON"; F=$((F+1)); }
exact(){ plan_th; mk $L/t1_i2_b${B}_th1 2; sleep 0.4; fin $L/t1_i2_b${B}_th1 0; }
t "plan th1; the lane runs th1 -> matched, nothing unexpected" 0 1 exact
other_lane(){ printf '{"runs": [{"lane": 2, "seq": 1, "tag": "b%s_A"}]}' $B > $L/${B}_plan.json; mkdir -p $L/t1_i3_b${B}_A; echo $L/t1_i3_b${B}_A > $L/.last_run_i2
  sleep 0.2; echo "New NED origin" > $L/t1_i3_b${B}_A/sitl.log; fin $L/t1_i3_b${B}_A 0; }
t "plan lane 2 tag A; lane 2 names a dir of lane 3 with tag A -> not matched (unexpected), A missing" 2 1 other_lane
seq_gap(){ printf '{"runs": [{"lane": 2, "seq": 1, "tag": "b%s_A"}, {"lane": 2, "seq": 3, "tag": "b%s_B"}]}' $B $B > $L/${B}_plan.json; }
t "plan with seq 1, 3 (not 1..N) -> observer error (exit 3)" 3 1 seq_gap
seq_dup(){ printf '{"runs": [{"lane": 2, "seq": 1, "tag": "b%s_A"}, {"lane": 2, "seq": 2, "tag": "b%s_A"}]}' $B $B > $L/${B}_plan.json; }
t "plan with the same tag twice in a lane -> observer error (exit 3)" 3 1 seq_dup
# ---- 1921p14 i4: landing started, no result line, log keeps growing (land retries) -> MISS result, not a silent pass
land_hang(){ mk $L/r1_b${B}_x 2; echo "[result] first mission event: safety_stop" >> $L/r1_b${B}_x/mission.log; echo "[7/7] Landing..." >> $L/r1_b${B}_x/mission.log
  for i in $(seq 1 110); do echo "land command failed (retry $i)" >> $L/r1_b${B}_x/mission.log; sleep 0.1; done; fin $L/r1_b${B}_x 10; }
t "landing started, no [result] EXIT line for LAND_END_S while the log grows -> MISS result" 1 1 land_hang
python3 -c "import json;d=json.load(open('/tmp/_rw_last.json'));r=d['runs'][0];assert 'result' in r['failed_at_s'] and 'landing' in r['done_at_s'],r" \
  && echo "PASS    watch.json: landing done, result missed" || { echo "FAIL    land_hang JSON"; F=$((F+1)); }
land_ok(){ mk $L/r1_b${B}_x 2; printf '[result] first mission event: complete\n[7/7] Landing...\n' >> $L/r1_b${B}_x/mission.log; sleep 0.5
  echo "[result] EXIT 0 (mission complete)" >> $L/r1_b${B}_x/mission.log; sleep 0.3; fin $L/r1_b${B}_x 0; }
t "landing then [result] EXIT in time -> no miss" 0 1 land_ok
rm -f /tmp/_rw_last.json; echo "$F failure(s)"; exit $F
