#!/bin/bash
# Offline checks of px4_param_guard.sh and batch_expect.py check (2026-11-01, codex experiment-provenance / runner-exit-status
# reply-6). Fake docker (PX4 parameters in a file) and synthetic batch logs; no instance is touched. exit = failures.
SC=$(cd "$(dirname "$0")/../scripts" && pwd); F=0
c(){ if eval "$2"; then echo "PASS  $1"; else echo "FAIL  $1"; F=$((F+1)); fi; }
# ---------------- px4_param_guard with a fake PX4 (values in $P/params, SAVE_FAIL / SET_FAIL switches)
G(){ PATH=$S:$PATH PX4_GUARD_LOGS=$L "$SC/px4_param_guard.sh" "$@"; }
L=$(mktemp -d); S=$(mktemp -d); P=$(mktemp -d); printf '2 15\n3 15\n' > $P/params
cat > $S/docker <<D
#!/bin/bash
cmd="\${@: -1}"; n=\$(echo "\$cmd" | grep -o -- '--instance [0-9]*' | awk '{print \$2}')
case "\$cmd" in
  *" show "*) v=\$(awk -v n=\$n '\$1==n{print \$2}' $P/params); [ -n "\$v" ] && echo "EKF2_EV_CTRL [351,576] : \$v"; exit 0;;
  *" set "*) [ -f $P/SET_FAIL ] && exit 1; v=\$(echo "\$cmd" | awk '{print \$NF}' | sed 's/>.*//'); v=\$(echo "\$cmd" | grep -o 'EKF2_EV_CTRL [0-9]*' | awk '{print \$2}')
             awk -v n=\$n -v v=\$v '\$1==n{\$2=v}1' $P/params > $P/p.tmp && mv $P/p.tmp $P/params; exit 0;;
  *" save"*) [ -f $P/SAVE_FAIL ] && exit 1; exit 0;;
esac
D
chmod +x $S/docker
G set EKF2_EV_CTRL 7 "2 3" > $L/o1 2>&1; r1=$?; v1=$(cat $P/params | tr '\n' ' '); G restore > $L/o2 2>&1; r2=$?; v2=$(cat $P/params | tr '\n' ' ')
c "guard: set 7 then restore -> 15 again (rc $r1/$r2; $v1-> $v2)" '[ $r1 = 0 ] && [ $r2 = 0 ] && [ "$v2" = "2 15 3 15 " ] && [ ! -f $L/.px4_param_pending ]'
touch $P/SAVE_FAIL; G set EKF2_EV_CTRL 7 "2 3" > $L/o3 2>&1; r3=$?
c "guard: save fails during set -> rc 1, restore attempted, pending KEPT (save fails too)" '[ $r3 = 1 ] && grep -q "save i2 failed" $L/o3 && [ -f $L/.px4_param_pending ]'
G set EKF2_EV_CTRL 7 "2 3" > $L/o4 2>&1; r4=$?
c "guard: set refused while a change is pending (rc $r4)" '[ $r4 = 1 ] && grep -q "not restored" $L/o4'
rm $P/SAVE_FAIL; G recover > $L/o5 2>&1; r5=$?
c "guard: next recover restores and closes the pending record (rc $r5)" '[ $r5 = 0 ] && [ ! -f $L/.px4_param_pending ] && [ "$(cat $P/params | tr "\n" " ")" = "2 15 3 15 " ]'
touch $P/SET_FAIL; G set EKF2_EV_CTRL 7 "2" > $L/o6 2>&1; r6=$?; rm $P/SET_FAIL; G recover > /dev/null 2>&1
c "guard: set command fails -> rc 1, value unchanged (rc $r6)" '[ $r6 = 1 ] && [ "$(awk "\$1==2{print \$2}" $P/params)" = 15 ]'
# ---- 2026-11-01 codex experiment-provenance reply-8: the pending record is checked before anything is restored; its owner
pv(){ cat $P/params | tr '\n' ' '; }
G set EKF2_EV_CTRL 7 "2 3" > /dev/null 2>&1; chmod 000 $L/.px4_param_pending; G restore > $L/o7 2>&1; r7=$?; chmod 644 $L/.px4_param_pending
c "guard: pending record unreadable -> rc 1, nothing restored, record kept (rc $r7; $(pv))" '[ $r7 = 1 ] && grep -q "cannot be read" $L/o7 && [ -f $L/.px4_param_pending ] && [ "$(pv)" = "2 7 3 7 " ]'
G restore > /dev/null 2>&1
G set EKF2_EV_CTRL 7 "2 3" > /dev/null 2>&1; echo "3 EKF2_EV_CTRL" >> $L/.px4_param_pending; G restore > $L/o8 2>&1; r8=$?
c "guard: a row without a value -> rc 1, NOTHING restored (not even the good rows), record kept (rc $r8; $(pv))" '[ $r8 = 1 ] && grep -q "line 4 is not" $L/o8 && [ -f $L/.px4_param_pending ] && [ "$(pv)" = "2 7 3 7 " ]'
sed -i '$d' $L/.px4_param_pending; echo "1 EKF2_EV_CTRL 15" >> $L/.px4_param_pending; G restore > $L/o9 2>&1; r9=$?
c "guard: a row for instance 1 (not 2..5) -> rc 1, record kept (rc $r9)" '[ $r9 = 1 ] && grep -q "line 4 is not" $L/o9 && [ -f $L/.px4_param_pending ]'
sed -i '$d' $L/.px4_param_pending; G restore > /dev/null 2>&1
printf '# owner 1 1\n' > $L/.px4_param_pending; G restore > $L/o10 2>&1; r10=$?
c "guard: a record with no value row -> rc 1, kept (rc $r10)" '[ $r10 = 1 ] && grep -q "no value to restore" $L/o10 && [ -f $L/.px4_param_pending ]'
rm $L/.px4_param_pending
G set EKF2_EV_CTRL 7 "2 9" > $L/o11 2>&1; r11=$?
c "guard: set on instance 9 refused before any change (rc $r11)" '[ $r11 = 1 ] && grep -q "instance 9 is not" $L/o11 && [ ! -f $L/.px4_param_pending ] && [ "$(pv)" = "2 15 3 15 " ]'
# another batch that still runs owns the change: the next batch's recover must not restore it; once it is gone, recover does
sleep 60 & OP=$!; PX4_GUARD_OWNER=$OP G set EKF2_EV_CTRL 7 "2 3" > /dev/null 2>&1
G recover > $L/o12 2>&1; r12=$?; v12=$(pv)
c "guard: recover while the owner batch runs -> refused rc 1, values stay 7 (rc $r12; $v12)" '[ $r12 = 1 ] && grep -q "still runs (pid $OP" $L/o12 && [ "$v12" = "2 7 3 7 " ] && [ -f $L/.px4_param_pending ]'
PX4_GUARD_OWNER=$OP G restore > $L/o13 2>&1; r13=$?
c "guard: the owner itself restores (its exit trap) -> rc 0 (rc $r13)" '[ $r13 = 0 ] && [ "$(pv)" = "2 15 3 15 " ] && [ ! -f $L/.px4_param_pending ]'
PX4_GUARD_OWNER=$OP G set EKF2_EV_CTRL 7 "2 3" > /dev/null 2>&1; kill $OP; wait $OP 2>/dev/null
G recover > $L/o14 2>&1; r14=$?
c "guard: owner batch gone (killed) -> next recover restores (rc $r14)" '[ $r14 = 0 ] && [ "$(pv)" = "2 15 3 15 " ] && [ ! -f $L/.px4_param_pending ]'
G set EKF2_EV_CTRL 7 "2 3" > /dev/null 2>&1; chmod 555 $L; G restore > $P/o15 2>&1; r15=$?; chmod 755 $L
c "guard: values restored but the record cannot be closed (dir read-only) -> rc 1, record kept (rc $r15)" '[ $r15 = 1 ] && grep -q "could not be closed" $P/o15 && [ -f $L/.px4_param_pending ]'
G restore > /dev/null 2>&1
printf '#!/bin/bash\nexit 1\n' > $S/docker.fail; cp $S/docker $S/docker.ok; cp $S/docker.fail $S/docker
G set EKF2_EV_CTRL 7 "2" > $L/o16 2>&1; r16=$?; cp $S/docker.ok $S/docker
c "guard: the parameter query itself fails (docker rc 1) -> nothing changed (rc $r16)" '[ $r16 = 1 ] && grep -q "cannot read i2" $L/o16 && [ ! -f $L/.px4_param_pending ]'
# ---- owner line required, owner states (codex experiment-provenance reply-10)
G set EKF2_EV_CTRL 7 "2 3" > /dev/null 2>&1; sed -i '1d' $L/.px4_param_pending          # an old-format record: no owner line
G recover > $L/o17 2>&1; r17=$?
c "guard: record without owner line -> recover refuses (rc $r17), names recover-legacy, record kept, values stay 7" '[ $r17 = 1 ] && grep -q "recover-legacy" $L/o17 && [ -f $L/.px4_param_pending ] && [ "$(pv)" = "2 7 3 7 " ]'
echo "9 EKF2_EV_CTRL 15" >> $L/.px4_param_pending; G recover-legacy > $L/o18 2>&1; r18=$?
c "guard: recover-legacy still checks every row (instance 9) -> rc 1, nothing restored (rc $r18)" '[ $r18 = 1 ] && [ "$(pv)" = "2 7 3 7 " ]'
sed -i '$d' $L/.px4_param_pending
bash -c 'exec -a run_phase99_fake.sh sleep 30' & BP=$!; sleep 0.3
PX4_GUARD_BATCH_PAT='run_phase99_fake' G recover-legacy > $L/o23 2>&1; r23=$?; kill $BP; wait $BP 2>/dev/null
c "guard: recover-legacy while a batch process runs -> refused rc 1, nothing restored (rc $r23)" '[ $r23 = 1 ] && grep -q "a batch runs" $L/o23 && [ "$(pv)" = "2 7 3 7 " ]'
PX4_GUARD_BATCH_PAT='run_phase99_fake' G recover-legacy > $L/o19 2>&1; r19=$?
c "guard: recover-legacy on a valid old record -> restored (rc $r19)" '[ $r19 = 0 ] && [ "$(pv)" = "2 15 3 15 " ] && [ ! -f $L/.px4_param_pending ]'
for bad in two_owner owner_not_first owner_bad; do
  G set EKF2_EV_CTRL 7 "2 3" > /dev/null 2>&1; o=$(head -1 $L/.px4_param_pending)
  case $bad in two_owner) echo "$o" >> $L/.px4_param_pending;; owner_not_first) { tail -n +2 $L/.px4_param_pending; echo "$o"; } > $L/x && mv $L/x $L/.px4_param_pending;;
    owner_bad) sed -i '1s/.*/# owner abc 12 x/' $L/.px4_param_pending;; esac
  G recover > $L/o20 2>&1; r20=$?
  c "guard: $bad -> refused rc 1, record kept (rc $r20)" '[ $r20 = 1 ] && grep -q "not a valid first-line owner" $L/o20 && [ -f $L/.px4_param_pending ]'
  sed -i '/^#/d' $L/.px4_param_pending; PX4_GUARD_BATCH_PAT='run_phase99_fake' G recover-legacy > /dev/null 2>&1
done
# owner pid alive but with another start time (pid reused) -> gone -> restored; pid exists, start time unreadable -> unknown -> kept
FP=$(mktemp -d); sleep 60 & OP=$!; mkdir -p $FP/$OP; awk '{$22=1; print}' /proc/$OP/stat > $FP/$OP/stat
PX4_GUARD_OWNER=$OP G set EKF2_EV_CTRL 7 "2 3" > /dev/null 2>&1; sed -i "1s/^# owner $OP [0-9]*/# owner $OP 999999999/" $L/.px4_param_pending
PX4_GUARD_PROC=$FP G recover > $L/o21 2>&1; r21=$?
c "guard: owner pid now another process (start time differs) -> taken as gone, restored (rc $r21)" '[ $r21 = 0 ] && [ "$(pv)" = "2 15 3 15 " ]'
PX4_GUARD_OWNER=$OP G set EKF2_EV_CTRL 7 "2 3" > /dev/null 2>&1; rm $FP/$OP/stat
PX4_GUARD_PROC=$FP G recover > $L/o22 2>&1; r22=$?
c "guard: owner pid exists, start time unreadable -> owner unknown, refused rc 1, record kept (rc $r22)" '[ $r22 = 1 ] && grep -q "owner unknown" $L/o22 && [ -f $L/.px4_param_pending ] && [ "$(pv)" = "2 7 3 7 " ]'
kill $OP; wait $OP 2>/dev/null; G recover > /dev/null 2>&1; rm -rf $FP
rm -rf $L $S $P
# ---------------- batch_expect check on synthetic logs
L=$(mktemp -d); B=tx
mkrun(){ d=$L/$1; mkdir -p $d; printf '%s\n' "$2" > $d/mission.log; echo "{\"exit\": $3, \"truth_verdict\": \"$4\", \"archive\": \"ok\", \"provenance\": \"ok\"}" > $d/status.json; echo "$d $3" >> $L/tofh_${B}_status.txt; }
setup(){ rm -rf $L/*; cat > $L/${B}_plan.json <<J
{"batch": "$B", "code_sha256": {}, "runs": [
 {"lane": 2, "seq": 1, "tag": "b${B}_A", "inject": "none", "expect_exit": [0], "expect_log": ["Done!"], "forbid_log": []},
 {"lane": 3, "seq": 1, "tag": "b${B}_B", "inject": "stop", "expect_exit": [13], "expect_log": ["Done!"], "forbid_log": []}]}
J
  mkrun r_i2_b${B}_A "Done!" 0 PASS; mkrun r_i3_b${B}_B "Done!" 13 PASS
  python3 - $L $B <<'W'
import json, sys; L, B = sys.argv[1], sys.argv[2]
json.dump({"end": "finished", "runs": [{"failed_at_s": {}}], "missing": [], "unexpected_runs": [],
           "expected_runs": [{"lane": "2", "seq": 1, "run": L + "/r_i2_b%s_A" % B}, {"lane": "3", "seq": 1, "run": L + "/r_i3_b%s_B" % B}]},
          open(L + "/%s_watch.json" % B, "w"))
W
  printf 'batch 1\nwatch 0\ninject_i3 0\n' > $L/${B}_children.txt; }
E_(){ BATCH_EXPECT_LOGS=$L python3 "$SC/batch_expect.py" check $B > $L/out 2>&1; echo $?; }
setup; c "expect: everything as planned -> 0 mismatches" '[ "$(E_)" = 0 ]'
setup; echo '{"exit": 13, "truth_verdict": "FAIL", "archive": "ok", "provenance": "ok"}' > $L/r_i3_b${B}_B/status.json
c "expect: exit 13 as planned but truth FAIL -> mismatch" '[ "$(E_)" = 1 ] && grep -q "truth_verdict FAIL" $L/out'
setup; echo '{"exit": 13, "truth_verdict": "PASS", "archive": "failed", "provenance": "ok"}' > $L/r_i3_b${B}_B/status.json
c "expect: exit 13 as planned but archive failed -> mismatch" '[ "$(E_)" = 1 ] && grep -q "archive failed" $L/out'
setup; mkrun r2_i2_b${B}_A "Done!" 0 PASS
c "expect: a second row for planned run A -> duplicate mismatch" '[ "$(E_)" -ge 1 ] && grep -q "duplicate" $L/out'
setup; mkrun r_i4_b${B}_Z "Done!" 0 PASS
c "expect: a row of no planned run -> mismatch" '[ "$(E_)" = 1 ] && grep -q "row of no planned run" $L/out'
setup; sed -i "s#r_i3_b${B}_B\"}#other\"}#" $L/${B}_watch.json
c "expect: the watcher watched another run -> mismatch" '[ "$(E_)" = 1 ] && grep -q "run_watch i3#1 watched" $L/out'
setup; printf 'batch 2\nwatch 0\ninject_i3 1\n' > $L/${B}_children.txt
c "expect: batch rc 2 (planned 1) and an injector rc 1 -> 2 mismatches" '[ "$(E_)" = 2 ]'
# ---- 2026-11-01 codex runner-exit-status reply-8: empty status, children file / rows, whole-tag match
setup; echo '{}' > $L/r_i3_b${B}_B/status.json
c "expect: status.json {} -> mismatch naming the missing keys" '[ "$(E_)" = 1 ] && grep -q "status.json lacks exit, truth_verdict, archive, provenance" $L/out'
setup; rm $L/${B}_children.txt
c "expect: children file missing -> mismatch" '[ "$(E_)" -ge 1 ] && grep -q "children.txt unreadable" $L/out'
setup; printf 'batch 1\ninject_i3 0\n' > $L/${B}_children.txt
c "expect: watch exit code missing from the children file -> mismatch" '[ "$(E_)" = 1 ] && grep -q "child watch has no exit code" $L/out'
setup; printf 'batch 1\nwatch 0\nwatch 0\n' > $L/${B}_children.txt
c "expect: a child listed twice -> mismatch" '[ "$(E_)" = 1 ] && grep -q "child watch listed twice" $L/out'
setup; printf 'batch 1\nwatch zero\n' > $L/${B}_children.txt
c "expect: an unreadable children row (and so watch missing) -> 2 mismatches" '[ "$(E_)" = 2 ] && grep -q "children line 2 unreadable" $L/out'
setup; python3 - $L/${B}_plan.json <<'W'
import json, sys; p = json.load(open(sys.argv[1])); p["children"] = ["batch", "watch", "inject_i3", "inject_i4"]; json.dump(p, open(sys.argv[1], "w"))
W
c "expect: plan requires inject_i4, not in the children file -> mismatch" '[ "$(E_)" = 1 ] && grep -q "child inject_i4 has no exit code" $L/out'
setup; mv $L/r_i2_b${B}_A $L/r_i2_b${B}_A0; sed -i "s#r_i2_b${B}_A #r_i2_b${B}_A0 #" $L/tofh_${B}_status.txt; sed -i "s#r_i2_b${B}_A\"#r_i2_b${B}_A0\"#" $L/${B}_watch.json
c "expect: plan tag A, the row is A0 (A is a prefix) -> run missing + row of no planned run" '[ "$(E_)" = 2 ] && grep -q "run missing" $L/out && grep -q "row of no planned run.*_A0" $L/out'
# ---- inject_lib.sh: a failed step reaches the injector's exit code (codex runner-exit-status reply-8)
( . $SC/inject_lib.sh; step "ok" true; step "action fails" false; step "after" true; inj_end ) > /tmp/_inj1 2>&1; i1=$?
( . $SC/inject_lib.sh; step "precondition fails" false || inj_end; step "never" true; inj_end ) > /tmp/_inj2 2>&1; i2=$?
( . $SC/inject_lib.sh; step "a" true; step "b" true; inj_end ) > /tmp/_inj3 2>&1; i3=$?
c "inject_lib: one failed action -> rc 1, later steps still run; failed precondition -> rc 1, nothing after; all ok -> rc 0 ($i1/$i2/$i3)" \
  '[ $i1 = 1 ] && grep -q "step .after. rc 0" /tmp/_inj1 && [ $i2 = 1 ] && ! grep -q never /tmp/_inj2 && [ $i3 = 0 ]'
rm -f /tmp/_inj1 /tmp/_inj2 /tmp/_inj3
# ---- result file not written: a mismatch even when the run's exit is the planned one (codex decisions reply-7 S3)
setup; echo '{"exit": 13, "truth_verdict": "PASS", "archive": "ok", "provenance": "ok", "result_file": "not_written"}' > $L/r_i3_b${B}_B/status.json
c "expect: exit 13 as planned but result_file not_written -> mismatch" '[ "$(E_)" = 1 ] && grep -q "mission_result.json not_written" $L/out'
# ---- real_sitl_inject.sh: every injection step reaches the exit code (codex runner-exit-status reply-10), fake docker
J=$(mktemp -d); S2=$(mktemp -d); cat > $S2/docker <<'D'
#!/bin/bash
case "$*" in
  *pgrep*) [ -n "$FAKE_PID" ] && echo "$FAKE_PID"; exit 0;;
  *"kill -INT"*) exit ${FAKE_KILL_RC:-0};;
  *starling-gz*) exit ${FAKE_GZ_RC:-0};;
esac
D
chmod +x $S2/docker
IJ(){ PATH=$S2:$PATH INJ_WAIT_S=1 "$SC/real_sitl_inject.sh" "$@" > $J/out 2>&1; echo $?; }
printf '[4/7] Capturing\n' > $J/mission.log
c "inject preflight: trigger seen, pid found, SIGINT ok -> 0" '[ "$(FAKE_PID=123 IJ preflight 2 $J)" = 0 ]'
c "inject preflight: launcher pid not found -> nonzero, no SIGINT step" '[ "$(FAKE_PID= IJ preflight 2 $J)" != 0 ] && grep -q "pid found ().*rc 1" $J/out && ! grep -q "SIGINT" $J/out'
c "inject preflight: SIGINT fails -> nonzero" '[ "$(FAKE_PID=123 FAKE_KILL_RC=1 IJ preflight 2 $J)" != 0 ] && grep -q "SIGINT 123. rc 1" $J/out'
c "inject voxlstop: first STAGE never in the log (1 s) -> nonzero, nothing paused" '[ "$(IJ voxlstop 2 $J)" != 0 ] && grep -q "not in mission.log within 1 s" $J/out && ! grep -q "relay paused" $J/out'
printf '[goto takeoff] [END] ok\n' > $J/mission.log
c "inject offset: the OFFSET write fails -> nonzero" '[ "$(FAKE_GZ_RC=1 IJ offset 2 $J)" != 0 ] && grep -q "offset 0.3 0.3 m. rc 1" $J/out'
c "inject offset: write ok -> 0" '[ "$(IJ offset 2 $J)" = 0 ]'
c "inject: unknown injection name -> nonzero" '[ "$(IJ bogus 2 $J)" != 0 ]'
rm -rf $J $S2
for rf in missing run_id_mismatch; do
  setup; echo "{\"exit\": 0, \"truth_verdict\": \"PASS\", \"archive\": \"ok\", \"provenance\": \"ok\", \"result_file\": \"$rf\"}" > $L/r_i2_b${B}_A/status.json
  c "expect: exit 0 as planned but result_file $rf -> mismatch" '[ "$(E_)" = 1 ] && grep -q "mission_result.json $rf" $L/out'
done
rm -rf $L; echo "$F failure(s)"; exit $F
