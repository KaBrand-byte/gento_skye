#!/bin/bash
# One-command start of the full task ON THE ORIN (arms via SDK 0407 aarch64, chassis via the ROS 2 bridge).
# Runs inside tmux, so a Wi-Fi drop of the operator's SSH session does not interrupt the robot.
#
#   bash orin_task.sh start [runner args]   start (or re-attach to) the task in tmux session "gento_task"
#   bash orin_task.sh attach                re-attach (e.g. after a Wi-Fi drop); Ctrl-b d detaches again
#   bash orin_task.sh stop                  Ctrl-C to the task (stops arms + chassis) and software-stop the chassis
#   bash orin_task.sh check                 no motion: config check and SDK version
#
# Runner args, e.g.:  --selfscan   --start ''   --place D --place-gap 0.30
# Environment (override before calling): GENTO_SDK_ROOT, GENTO_TASK_CONFIG, CYCLONEDDS_URI
set -e
REPO=$(cd "$(dirname "$0")/../.." && pwd)
SESSION=gento_task
env_setup() {
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=24 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file:///etc/apex/cyclonedds.xml}
  export GENTO_SDK_ROOT=${GENTO_SDK_ROOT:-$HOME/robot_sdk/tianji-robot-SDK}
  cd "$REPO"
}
case "${1:-start}" in
  run)        # internal: what runs inside tmux
    shift; env_setup
    python3 chassis/client/run_task_with_navigation.py --config "${GENTO_TASK_CONFIG:-config/default.json}" "$@"
    echo; echo "task finished - result in $REPO/runs/with-navigation.json; press Enter to close"; read -r _ ;;
  start)
    shift
    if tmux has-session -t $SESSION 2>/dev/null; then echo "task already running - attaching"; exec tmux attach -t $SESSION; fi
    tmux new-session -s $SESSION "bash '$REPO/chassis/client/orin_task.sh' run $(printf '%q ' "$@")" ;;
  attach)     exec tmux attach -t $SESSION ;;
  stop)
    tmux send-keys -t $SESSION C-c 2>/dev/null && echo "Ctrl-C sent to the task" || echo "no task session"
    env_setup
    python3 -c "
import sys, time; sys.path.insert(0, 'chassis/client')
from gento_navigation import GentoNavigation
n = GentoNavigation(); time.sleep(1); n.stop(); print('chassis stopped'); n.close()" ;;
  check)
    env_setup
    python3 -m skye_task check --config "${GENTO_TASK_CONFIG:-config/default.json}" > /dev/null && echo "config ok"
    python3 -c "
import sys; sys.path.insert(0, '$GENTO_SDK_ROOT/PYTHON_SDK')
import GentoRobot; print('SDK', GentoRobot.GentoRobot(precision=6).get_sdk_version())" ;;
  *) sed -n 2,12p "$0" ;;
esac
