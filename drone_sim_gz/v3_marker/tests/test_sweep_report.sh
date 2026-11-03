#!/bin/bash
# wk_sweep_report.py with missing values mixed in (2026-10-31, codex missing-metric-status reply-2): clearance missing, error
# missing, both missing, metrics.json missing, err_legs exactly 0. The summary must finish and show "없음". exit = failures.
PY=${PY:-/home/dronesim/starling_sim/cad/venv/bin/python}; SC=$(cd "$(dirname "$0")/../scripts" && pwd); T=$(mktemp -d); F=0
mk(){ mkdir -p $T/$1; echo "$2" > $T/$1/metrics.json; }
mk full    '{"grid_verify": "PASS", "err_max_abs_legs": 0.0, "err_max_abs": 0.021, "rotor_clear_min_front": 0.12, "rotor_clear_min_back": 0.15}'
mk noclear '{"grid_verify": "PASS", "err_max_abs_legs": 0.01, "err_max_abs": 0.02}'
mk noerr   '{"grid_verify": "FAIL", "rotor_clear_min_front": 0.10}'
mk none    '{"grid_verify": null}'
mkdir -p $T/nometrics
printf 'f yaw20 on %s\nf yaw40 on %s\nf drift5 on %s\nf step3 on %s\nf yaw60 on %s\nf yaw80 on MISSING\n' $T/full $T/noclear $T/noerr $T/none $T/nometrics > $T/list.txt
$PY "$SC/wk_sweep_report.py" $T/out $T/list.txt > $T/stdout 2>&1; rc=$?
c(){ if eval "$2"; then echo "PASS  $1"; else echo "FAIL  $1"; F=$((F+1)); fi; }
c "report finishes (rc $rc)"                        '[ $rc = 0 ]'
c "sweep.md has 4 rows"                             '[ $(grep -c "| on |" $T/out/sweep.md) = 4 ]'
c "missing clearance shown as 없음, not a number"   'grep "| 4 | on |" $T/out/sweep.md | grep -q "없음"'
c "row without numbers marked 미평가"                'grep -q "미평가" $T/out/sweep.md'
c "err_legs 0.0 kept as 0.0 (not replaced)"         'grep "| 2 | on |" $T/out/sweep.md | grep -q "| 0.0 |"'
c "not evaluated: no metrics.json + MISSING (2)"    '[ $(python3 -c "import json;print(len(json.load(open(\"$T/out/sweep_status.json\"))[\"not_evaluated\"]))") = 2 ]'
cat $T/out/sweep.md; rm -rf $T; echo "$F failure(s)"; exit $F
