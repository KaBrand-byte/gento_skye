#!/usr/bin/env bash
set -eo pipefail
if systemctl is-active --quiet apex-camera.service; then
    echo 'Apex camera service is already active; use that stream or stop it explicitly.' >&2
    exit 1
fi
source /etc/apex/apex_ros_env.sh
export PYTHONPATH="/opt/kernelmind/apex/camera-python${PYTHONPATH:+:$PYTHONPATH}"
vla_orin_root="${VLA_ORIN_ROOT:-$HOME/vla_pi05}"
exec ros2 launch gmsl_quadcam quad_csi_quickview.launch.py \
    params_file:="$vla_orin_root/camera-observation-full.yaml" signal_host:=127.0.0.1
