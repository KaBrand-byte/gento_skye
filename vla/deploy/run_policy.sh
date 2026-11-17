#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/env.sh"
cd "$VLA_DEPLOY_ROOT/openpi"
exec "$VLA_DEPLOY_ROOT/openpi/.venv/bin/python" -u "$VLA_DEPLOY_ROOT/serve_observation.py" \
    --checkpoint "$VLA_DEPLOY_ROOT/cache/openpi/openpi-assets/checkpoints/pi05_droid" "$@"
