#!/usr/bin/env bash
set -euo pipefail
integration_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
host_python="${PANTHERA_PYTHON:-/home/aster/miniforge3/envs/panthera/bin/python}"
run_dir="$(mktemp -d "$integration_root/evidence/state-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
printf 'State-only check: arm queries 500 Hz, hand angle queries 50 Hz.\nReport: %s\n' "$run_dir"
# Queries and telemetry configuration only. No claim, active, target or mode command.
"$host_python" -u "$integration_root/source/host/examples/1_robot-state.py" \
    --seconds 15 --motor-query-hz 500 --hand-query-hz 50 --hand-angles-only \
    --report "$run_dir/state.json" "$@" | tee "$run_dir/console.log"
