#!/bin/bash
# Offline checks of repo_export/scripts/px4_model_swap.sh (2026-10-31, codex model-install-atomicity reply-2): migration,
# unchanged, version change, failures (nothing switched / directory put back), and a reader that keeps opening model.sdf while
# the versions are switched back and forth. Temporary tree only. exit code = number of failures.
. "$(cd "$(dirname "$0")/../.." && pwd)/repo_export/scripts/px4_model_swap.sh"
T=$(mktemp -d); F=0; ARC=$T/arc; D=$T/models; mkdir -p $D $T/a $T/b
ck(){ if [ "$2" = "$3" ]; then echo "PASS  $1 -> $2"; else echo "FAIL  $1 -> $2 (expected $3)"; F=$((F+1)); fi; }
for i in $(seq 1 200); do echo "A line $i"; done > $T/a/model.sdf; for i in $(seq 1 200); do echo "B line $i"; done > $T/b/model.sdf
cp -r $T/a $D/m                                                            # old-style real directory
ck "first call on a real directory" "$(swap_model $T/a $D m)" migrated
ck "  link target" "$(readlink $D/m | cut -d. -f2-3 | cut -d. -f1)" "versions/m"
ck "  old directory archived" "$(ls $ARC | wc -l)" 1
ck "same source again" "$(swap_model $T/a $D m)" unchanged
LA=$(readlink $D/m); ck "other source" "$(swap_model $T/b $D m)" updated
ck "  both versions kept" "$(ls $D/.versions | wc -l)" 2
( ln(){ return 1; }; swap_model $T/a $D m >/dev/null ); ck "link creation fails -> non-zero" $? 1
ck "  still the B version" "$(head -c1 $D/m/model.sdf)" B
rm -rf $D/m && cp -r $T/a $D/m
( ln(){ return 1; }; swap_model $T/b $D m >/dev/null ); ck "migration with failing link -> non-zero" $? 1
ck "  real directory put back" "$([ -d $D/m ] && [ ! -L $D/m ] && head -c1 $D/m/model.sdf)" A
swap_model $T/nonexistent $D m >/dev/null 2>&1; ck "missing source -> non-zero" $? 1
ck "  unchanged after missing source" "$([ ! -L $D/m ] && head -c1 $D/m/model.sdf)" A
swap_model $T/a $D m >/dev/null
# reader: open model.sdf through the link 3000 times, every read must be a whole A or a whole B file
( bad=0; for i in $(seq 1 3000); do c=$(cat $D/m/model.sdf 2>/dev/null) || { bad=$((bad+1)); continue; }
    n=$(echo "$c" | wc -l); k=$(echo "$c" | cut -c1 | sort -u | tr -d '\n'); [ "$n" = 200 ] && { [ "$k" = A ] || [ "$k" = B ]; } || bad=$((bad+1)); done
  echo $bad > $T/reader ) &
R=$!; sw=0; while kill -0 $R 2>/dev/null; do swap_model $T/b $D m >/dev/null; swap_model $T/a $D m >/dev/null; sw=$((sw+2)); done
ck "reader during $sw switches: failed or mixed reads" "$(cat $T/reader)" 0
rm -rf "$T"; echo "$F failure(s)"; exit $F
