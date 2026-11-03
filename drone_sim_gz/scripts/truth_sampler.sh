#!/bin/bash
# Sample gz ground truth (position + RPY) of the PX4 model every N seconds. Run inside starling-gz; stop with: pkill -x truth_sampler.sh  (or kill the PID)
# usage: truth_sampler.sh [model=starling2_0] [period_s=2.5]
M=${1:-starling2_0}; P=${2:-2.5}
while true; do
  ts=$(date +%H:%M:%S.%1N)     # stamp BEFORE the query: gz model -p can take >1 s under GUI load (2026-10-28 run13 s21)
  out=$(timeout -s KILL 3 gz model -m "$M" -p 2>/dev/null | grep -A2 -m1 "Pose \[" | tail -2 | tr -d '\n' | tr -s ' ')
  echo "$ts $out"
  sleep "$P"
done
