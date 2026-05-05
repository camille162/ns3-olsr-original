#!/usr/bin/env bash
#
# Batch runner for anti-drone / ETX-OLSR ns-3 scenarios.
# - Each experiment writes: logs/<id>.log (tee), manifest.tsv (exit codes)
# - Runs `sync` after each experiment to reduce data loss on crash/power loss
#
# Usage:
#   cd ~/etx-olsr/build
#   SIM=./sim bash /path/to/scripts/run-experiment-matrix.sh          # all experiments
#   SIM=./sim bash .../run-experiment-matrix.sh smoke                    # smoke tests only
#   SIM=./sim SESSION_ROOT=$HOME/etx-olsr-results bash .../run-experiment-matrix.sh
#
# Rebuild note: if ./sim --PrintHelp has no failureFraction / enableKillChainFlows, copy this
# repository's src/main.cc into your ns-3 project and recompile to get GUIDE_* lines.
#
# nohup: put env before bash (otherwise SIM= is wrong and you get exit 127):
#   nohup env SIM="${PWD}/sim" bash ../scripts/run-experiment-matrix.sh guide > guide.log 2>&1 &
#
# Crash / freeze resilience tips:
#   - Run inside tmux or screen so SSH disconnect does not kill the job
#   - Point SESSION_ROOT to a persistent disk (not tmpfs)
#   - For VM VMware: enable shared folder or mount NFS to host backup
#   - Optional: run `watch -n 60 sync` in another terminal during long batches
#
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SIM_BIN="${SIM:-./sim}"
FILTER="${1:-all}"

if [[ ! -x "$SIM_BIN" && "$SIM_BIN" == ./sim ]]; then
  echo "WARN: ./sim not executable in cwd=$(pwd). Set SIM to full path, e.g. SIM=~/etx-olsr/build/sim" >&2
fi

SESSION_ROOT="${SESSION_ROOT:-$HOME/etx-olsr-results}"
SESSION="${SESSION_ROOT}/session-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$SESSION/logs"

# One --PrintHelp snapshot: old sim builds omit options that exist only in newer main.cc.
set +e
SIM_HELP=$("$SIM_BIN" --PrintHelp 2>&1)
set -e
sim_has_opt() {
  # Match ns-3 help lines like: --failureFraction:
  printf '%s' "$SIM_HELP" | grep -qE "[[:space:]]--${1}(:|[[:space:]])"
}

if sim_has_opt enableControlAgent; then
  ETX_FAIR_CTRL=(--enableControlAgent=0)
  ETX_FULL_CTRL=(--enableControlAgent=1)
else
  ETX_FAIR_CTRL=()
  ETX_FULL_CTRL=()
  echo "NOTE: sim has no --enableControlAgent (older main.cc). ETX fair/full flags skipped." | tee -a "$SESSION/session.txt"
fi

if ! sim_has_opt failureFraction || ! sim_has_opt enableKillChainFlows; then
  echo "NOTE: sim missing guide-only flags (failureFraction / enableKillChainFlows / ...)." \
       "Copy scripts/../src/main.cc from this repo into your ns-3 tree and rebuild for GUIDE_* output." | tee -a "$SESSION/session.txt"
fi

# Optional argument packs (only passed when this binary's --PrintHelp lists them).
GUIDE_EXTRA_OPTS=()
if sim_has_opt failureFraction; then GUIDE_EXTRA_OPTS+=(--failureFraction=0.1); fi
if sim_has_opt enableKillChainFlows; then
  GUIDE_EXTRA_OPTS+=(--enableKillChainFlows=1 --senseNode=0 --decisionNode=1 --interceptorNode=2)
fi
if sim_has_opt routeGenLimitSec; then
  GUIDE_EXTRA_OPTS+=(--routeGenLimitSec=10 --routeSwitchLimitSec=3)
fi
if sim_has_opt limitDelaySenseDecisionMs; then
  GUIDE_EXTRA_OPTS+=(--limitDelaySenseDecisionMs=100 --limitDelayDecisionInterceptorMs=50)
fi

MANIFEST="$SESSION/manifest.tsv"
echo -e "timestamp_iso\texit_code\texperiment_id\tcommand" > "$MANIFEST"
echo "Session directory: $SESSION" | tee -a "$SESSION/session.txt"
echo "SIM binary: $SIM_BIN" >> "$SESSION/session.txt"

run_one() {
  local exp_id="$1"
  shift
  local -a args=("$@")
  local logf="$SESSION/logs/${exp_id}.log"
  echo ""
  echo "========== ${exp_id} =========="
  echo "${args[*]}"
  # tee writes incrementally; crash mid-run still keeps partial log
  "$SIM_BIN" "${args[@]}" 2>&1 | tee "$logf"
  local ec=${PIPESTATUS[0]}
  sync 2>/dev/null || true
  printf '%s\t%d\t%s\t%s\n' "$(date -Iseconds 2>/dev/null || date)" "$ec" "$exp_id" "${args[*]}" >> "$MANIFEST"
  if [[ $ec -ne 0 ]]; then
    echo "FAILED ${exp_id} exit=${ec}" | tee -a "$SESSION/failures.txt"
  fi
  # Continue batch even if one experiment fails
  return 0
}

# --- Smoke (fast) ---
run_smoke() {
  run_one SMOKE-ETX --routing=etx "${ETX_FAIR_CTRL[@]}" --enableJammer=0 --uavCount=20 --stopTime=15 --seed=1 --run=1 --connectivityLogInterval=2
  run_one SMOKE-OLSR --routing=olsr --enableJammer=0 --uavCount=20 --stopTime=15 --seed=1 --run=1 --connectivityLogInterval=2
}

# --- Milestone 1: scale >=50, comparable routing A/B ---
run_m1() {
  local common=(--uavCount=50 --stopTime=120 --seed=1 --connectivityLogInterval=1 --flowCount=10 --flowInterval=0.02)
  run_one M1-ETX-FAIR --routing=etx "${ETX_FAIR_CTRL[@]}" "${common[@]}" --run=1
  run_one M1-OLSR-FAIR --routing=olsr "${common[@]}" --run=1
  run_one M1-ETX-FULL --routing=etx "${ETX_FULL_CTRL[@]}" "${common[@]}" --run=1
  run_one M1-ETX-FAIR-S2 --routing=etx "${ETX_FAIR_CTRL[@]}" "${common[@]}" --run=1 --seed=2
  run_one M1-ETX-FAIR-S3 --routing=etx "${ETX_FAIR_CTRL[@]}" "${common[@]}" --run=1 --seed=3
}

# --- Milestone 2: longer run, resilience-oriented (10% random failures built into main.cc for uav=50 -> 5 nodes) ---
run_m2() {
  local common=(--uavCount=50 --stopTime=180 --connectivityLogInterval=1 --flowCount=10 --flowInterval=0.02)
  run_one M2-ETX-RES --routing=etx "${ETX_FAIR_CTRL[@]}" --enableJammer=1 "${common[@]}" --seed=1 --run=2
  run_one M2-OLSR-RES --routing=olsr --enableJammer=1 "${common[@]}" --seed=1 --run=2
  run_one M2-CLEAN-ETX --routing=etx "${ETX_FAIR_CTRL[@]}" --enableJammer=0 "${common[@]}" --seed=1 --run=1
}

# --- If full scale freezes your PC ---
run_cal_lite() {
  run_one CAL-LITE --routing=etx "${ETX_FAIR_CTRL[@]}" --uavCount=30 --stopTime=60 --seed=1 --run=1 --connectivityLogInterval=1
}

# --- 教材式验收跑法：杀伤链端口 + 指南阈值（新版 main.cc 打印 GUIDE_*；旧 sim 自动省略未知参数）---
run_guide() {
  local common=(
    --uavCount=50 --stopTime=120 --failureTime=5
    --seed=1 --run=1 --connectivityLogInterval=1 --flowCount=8 --flowInterval=0.02
  )
  run_one GUIDE-ETX-FAIR --routing=etx "${ETX_FAIR_CTRL[@]}" "${common[@]}" "${GUIDE_EXTRA_OPTS[@]}" --enableJammer=1
  run_one GUIDE-OLSR --routing=olsr "${common[@]}" "${GUIDE_EXTRA_OPTS[@]}" --enableJammer=1
}

run_all() {
  run_smoke
  run_m1
  run_m2
}

case "$FILTER" in
  all) run_all ;;
  smoke) run_smoke ;;
  m1) run_m1 ;;
  m2) run_m2 ;;
  cal|lite|cal-lite) run_cal_lite ;;
  guide) run_guide ;;
  *)
    echo "Unknown filter: $FILTER (use: all, smoke, m1, m2, guide, cal-lite)" >&2
    exit 1
    ;;
esac

sync 2>/dev/null || true
echo ""
echo "Done. Logs under: $SESSION/logs"
echo "Manifest: $MANIFEST"
echo "Copy session elsewhere for backup: cp -a \"$SESSION\" /media/usb/"
