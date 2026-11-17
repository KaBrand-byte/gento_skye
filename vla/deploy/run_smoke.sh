#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/env.sh"
vla_root="$VLA_DEPLOY_ROOT"
trap 'result=$?; printf "%s\n" "$result" > "$vla_root/logs/smoke.exit"' EXIT
cd "$vla_root/openpi"
vla_uv="$vla_root/tools/bin/uv"
printf 'Checkpoint download started: %s\n' "$(date -Is)"
python3 -u "$vla_root/download_http.py" --root "$vla_root"
vla_checkpoint="$(cat "$vla_root/logs/checkpoint-path.txt")"
printf 'Inference test started: %s\n' "$(date -Is)"
"$vla_uv" run --no-sync python -u "$vla_root/official_smoke.py" benchmark \
    --root "$vla_root" --checkpoint "$vla_checkpoint" --steps "${VLA_SMOKE_STEPS:-2}"
printf 'Inference test completed: %s\n' "$(date -Is)"
