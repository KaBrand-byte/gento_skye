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
  run)        # internal: what runs inside tmux; the window always stays open so errors can be read
    shift; set +e; env_setup
    N=1; if [ "$1" = "--cycles" ]; then N=$2; shift 2; fi      # --cycles N: N full tasks in a row (success rate)
    ok=0; LOG="$REPO/runs/cycles.log"; mkdir -p "$REPO/runs"
    [ "$N" -gt 1 ] && echo "===== $(date '+%F %T') start $N cycles: $*" >> "$LOG"
    for i in $(seq 1 "$N"); do
      [ "$N" -gt 1 ] && echo && echo "=================== 第 $i/$N 次 ==================="
      RES="$REPO/runs/with-navigation.json"; [ "$N" -gt 1 ] && RES="$REPO/runs/cycle-$(printf %02d "$i").json"
      GENTO_CYCLE="$i/$N" python3 chassis/client/run_task_with_navigation.py           --config "${GENTO_TASK_CONFIG:-config/default.json}" --result "$RES" "$@"
      rc=$?; [ $rc -eq 0 ] && ok=$((ok + 1))
      [ "$N" -gt 1 ] && echo "$(date '+%F %T') cycle $i/$N exit $rc  (ok $ok/$i)  $RES" | tee -a "$LOG"
      [ $rc -ne 0 ] && break                     # state unknown after a failure: stop the series
    done
    echo; echo "finished: $ok of $N successful (last exit $rc) - results in $REPO/runs/; press Enter to close"; read -r _ ;;
  start)
    shift
    # tmux needs a usable terminal type (Windows ssh -t may send none or one the Orin does not know)
    if [ -z "$TERM" ] || [ "$TERM" = dumb ] || ! infocmp "$TERM" >/dev/null 2>&1; then export TERM=xterm-256color; fi
    if tmux has-session -t $SESSION 2>/dev/null; then
      if pgrep -f "^python3 chassis/client/run_task_with_navigation.py" >/dev/null; then
        echo "task already running - attaching"; exec tmux attach -t $SESSION
      fi
      echo "previous task window had finished - closing it and starting a new task"
      tmux kill-session -t $SESSION
    fi
    ARGS=""; [ $# -gt 0 ] && ARGS=$(printf '%q ' "$@")      # no args: forward nothing (printf would emit '')
    tmux new-session -s $SESSION "bash '$REPO/chassis/client/orin_task.sh' run $ARGS" ;;
  attach)
    if [ -z "$TERM" ] || [ "$TERM" = dumb ] || ! infocmp "$TERM" >/dev/null 2>&1; then export TERM=xterm-256color; fi
    exec tmux attach -t $SESSION ;;
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
