#!/bin/bash
# Gento demo control from Linux - ON THE ORIN ITSELF (ssh nvidia@<orin>, then `gento ...`) or from any Ubuntu PC
# on the robot network.  Same commands as tools/gento.ps1.  Usage:  gento <command> [args]
#
#   status     robot state, nav tasks, running job, last log lines        (no motion)
#   selfscan   measure arms (+ tray) with the lidar, store in the model    (no motion; nothing within ~1 m in front)
#   check T|D  what docking at T / D would do from the current pose       (no motion)
#   goto NAME [GAP]       DRIVE: dock at station NAME (T, D, ...)                                  (asks y/n)
#   lineup NAME           DRIVE: like goto, but stop lined up in front of NAME before driving in  (asks y/n)
#   undock                DRIVE: reverse straight out along the last docking approach              (asks y/n)
#   tune NAME KEY VALUE   fine-tune: gap (m) | lat_offset (m, + = further left) | yaw_offset (deg, + = left)
#   teach-pose NAME [BACK] [left|right]   NO MOTION: robot EXACTLY at the final pose of NAME: record it
#   teach-staging NAME [map]  NO MOTION: robot at a safe point near NAME (room to turn): its staging point
#   teach-ref NAME        NO MOTION: robot exactly at map station NAME: record a reference scan
#   accept NAME [clear]   NO MOTION: robot lined up in front of NAME: structure verified clear on site
#   restage NAME          NO MOTION: recompute the staging spot of a taught station from here
#   relocate   fix the localisation (scan matched to the whole map)                    (no motion)
#   job        job status JSON
#   stop       software stop: kill job, cancel moves + tasks, soft manual
#   log        follow the job log (Ctrl+C to leave; the job keeps running)
#   task [args]    START THE FULL TASK (arms + navigation) in tmux on the Orin           (asks y/n)
#                  args e.g. --down-mm 120 | --holding | --no-return | --start ''
#   task-attach    show the running task again (Ctrl-b d detaches; the task keeps running)
#   task-stop      stop the task (arms + chassis) and software-stop the chassis
#   task-check     no motion: task config + SDK version on the Orin
#   web        (PC only) open the chassis web page through an ssh tunnel
#   setup      ON THE ORIN, once: key login Orin -> chassis (asks for the chassis password one time)
#
# Before any motion: someone at the red e-stop, hard switch on AUTO, charging cable unplugged.
#
# Targets (override with environment variables):
#   GENTO_ORIN     ssh target of the Orin (PC default: orin);       on the Orin itself: local
#   GENTO_CHASSIS  ssh target of the chassis (PC default: chassis); on the Orin: sunrise@6.6.7.6 (wired)

ON_ORIN=0
if [ -x "$HOME/gento_skye/chassis/client/orin_task.sh" ] && hostname -I 2>/dev/null | tr ' ' '\n' | grep -qx '6.6.7.100'; then
  ON_ORIN=1
fi
if [ $ON_ORIN = 1 ]; then CHASSIS=${GENTO_CHASSIS:-sunrise@6.6.7.6}; else CHASSIS=${GENTO_CHASSIS:-chassis}; fi
ORIN=${GENTO_ORIN:-orin}
REMOTE=${GENTO_DOCK_RUN:-bash ~/gento_dock/run.sh}
TASK_SH='bash ~/gento_skye/chassis/client/orin_task.sh'

confirm() {
  echo
  echo "  About to MOVE the robot: $1"
  echo "  - someone at the e-stop?  - hard switch AUTO?  - charging cable unplugged?"
  read -r -p "  Type y to start: " a
  [ "$a" = y ] || { echo "  cancelled"; exit 1; }
}
ch()    { ssh -o ConnectTimeout=10 "$CHASSIS" "$REMOTE $*"; }                 # chassis command
ch_t()  { ssh -t -o ConnectTimeout=10 "$CHASSIS" "$REMOTE $* && $REMOTE follow"; }   # motion job, show it until it ends
orin()  {                                                                      # task launcher on the Orin
  if [ $ON_ORIN = 1 ]; then eval "$TASK_SH $*"; else ssh -t -o ConnectTimeout=10 "$ORIN" "$TASK_SH $*"; fi
}
quote() { local out=""; for a in "$@"; do out+=" $(printf '%q' "$a")"; done; echo "$out"; }

cmd=${1:-help}; shift || true
case "$cmd" in
  status|selfscan|job-status|relocate|stop)  ch "$cmd" ;;
  job)        ch job-status ;;
  check)      ch check "${1:-D}" ;;
  tune|teach-pose|teach-staging|teach-ref|accept|restage)  ch "$cmd$(quote "$@")" ;;
  goto)       [ -n "$1" ] || { echo "usage: gento goto NAME [GAP]"; exit 1; }
              confirm "dock at station $1"; ch_t "goto$(quote "$@")" ;;
  lineup)     [ -n "$1" ] || { echo "usage: gento lineup NAME"; exit 1; }
              confirm "line up in front of station $1 (not driving in)"; ch_t "lineup$(quote "$1")" ;;
  undock)     confirm "reverse straight out of the last docking"; ch_t undock ;;
  log)        ssh -t "$CHASSIS" "$REMOTE log" ;;
  task)       confirm "FULL TASK: arms to the carry pose, T, Enter, D, place, back to T"; orin "start$(quote "$@")" ;;
  task-attach) orin attach ;;
  task-stop)  orin stop ;;
  task-check) orin check ;;
  web)        if [ $ON_ORIN = 1 ]; then echo "on the Orin: open http://6.6.7.6:8080 from a PC instead (gento web there)"; exit 1; fi
              if ! curl -s -m 3 -o /dev/null http://127.0.0.1:8080/; then
                pkill -f "ssh -N -L 8080:6.6.7.6:8080" 2>/dev/null
                ssh -f -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
                    -L 8080:6.6.7.6:8080 -L 6001:6.6.7.6:6001 "$ORIN" || { echo "tunnel failed: is this PC on the robot network?"; exit 1; }
                sleep 2
              fi
              xdg-open http://localhost:8080 >/dev/null 2>&1 || echo "open http://localhost:8080" ;;
  setup)      [ $ON_ORIN = 1 ] || { echo "run 'gento setup' on the Orin"; exit 1; }
              [ -f ~/.ssh/id_ed25519 ] || ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
              ssh-copy-id -o ConnectTimeout=10 sunrise@6.6.7.6 && ssh -o BatchMode=yes sunrise@6.6.7.6 echo "chassis login OK" ;;
  *)          sed -n '2,33p' "$0" ;;
esac
