#!/usr/bin/env bash
set -eo pipefail
source /etc/apex/apex_ros_env.sh
vla_orin_root="${VLA_ORIN_ROOT:-$HOME/vla_pi05}"
export PYTHONPATH="$vla_orin_root/client_python${PYTHONPATH:+:$PYTHONPATH}"
export NO_PROXY="6.6.7.99,192.168.8.101,localhost,127.0.0.1${NO_PROXY:+,$NO_PROXY}"
export PYTHONDONTWRITEBYTECODE=1
exec python3 -u "$vla_orin_root/observe_infer.py" "$@"
