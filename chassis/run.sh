#!/bin/bash
# Gento demo runner on the chassis.  Every motion job runs detached (survives Wi-Fi drops), one at a time.
#   run.sh to-start [GAP]   drive to the table: T1, align, straight in to GAP m (default 0.45)        (MOVES)
#   run.sh demo [GAP]       from the table: move out, D1, face the conveyor, straight in towards GAP m
#                           (default 0.25) from the current lateral position; stops early where the
#                           robot/arms/tray would touch something                                       (MOVES)
#   run.sh goto NAME [GAP]  dock at station NAME of stations.json (used by the skye_task adapter)     (MOVES)
#   run.sh demo-dry [GAP]   same checks as demo, no motion
#   run.sh stop             software stop (kill job, cancel moves/tasks, soft manual)
#   run.sh status           state, tasks, running job, last log lines
#   run.sh job-status       one JSON line {"state": idle|running|done|failed, ...} (for programs)
#   run.sh tune NAME KEY V  fine-tune a station: gap (m) | lat_offset (m, + = robot further left) | yaw_offset (deg, + = left)
#   run.sh teach NAME       record the CURRENT lateral position as station NAME (robot at its staging point,
#                           facing the edge; turns a little to square up)                             (MOVES a little)
#   run.sh selfscan         measure arms/tray with the lidar and store them (no motion)
#   run.sh check NAME       what docking at NAME would do from here (no motion)
#   run.sh log              follow the job log
DIR=$(cd "$(dirname "$0")" && pwd)
cd "$DIR" && source env.sh
LOG=$DIR/run.log; PID=$DIR/run.pid; JS=$DIR/job_status.json
running() { [ -f "$PID" ] && kill -0 "$(cat $PID)" 2>/dev/null; }
num() { [[ "$1" =~ ^0?\.[0-9]+$|^[0-9]+(\.[0-9]+)?$ ]] && echo "$1" || echo "$2"; }
start() {                                   # start JOB CMD...: detached, status kept in job_status.json
  if running; then echo "BUSY: job $(cat $PID) still running - use stop first"; exit 1; fi
  local job="$1"; shift
  # mark the NEW job as running before returning, so no reader can still see the previous job's result
  printf '{"state": "running", "job": "%s", "t": %s}\n' "$job" "$(date +%s)" > "$JS"
  setsid nohup python3 "$DIR/job_wrap.py" "$job" -- "$@" > /dev/null 2>&1 < /dev/null &
  echo $! > "$PID"; echo "started job $job ($!): $*  (follow with: log)"
}
case "$1" in
  to-start) start T timeout 400 python3 dock.py dock T $(num "$2" 0.45) --leave --closest ;;
  demo)     start D timeout 500 python3 dock.py dock D $(num "$2" 0.25) --leave --here --closest ;;
  goto)     [ -z "$2" ] && { echo "usage: goto NAME [GAP]"; exit 1; }
            extra=$(python3 -c "import json,sys;s=json.load(open('stations.json')).get(sys.argv[1]);print('' if s is None else ('--here --closest' if s.get('untaught') else '--closest'))" "$2")
            [ -z "$extra" ] && { echo "unknown station $2"; exit 1; }
            start "$2" timeout 500 python3 dock.py dock "$2" $(num "$3" '') --leave $extra ;;
  demo-dry) timeout 120 python3 dock.py dock D $(num "$2" 0.25) --leave --here --closest --dry ;;
  stop)     if running; then pkill -TERM -s "$(ps -o sid= -p $(cat $PID) | tr -d ' ')" 2>/dev/null; fi
            rm -f "$PID"; printf '{"state": "failed", "job": "stopped", "error": "stopped by user", "t": %s}\n' "$(date +%s)" > "$JS"
            timeout 30 python3 dock.py stop ;;
  status)   timeout 15 python3 nav.py state
            curl -s -m 5 -X POST -d '{}' http://127.0.0.1:8080/api/Task/GetAllTask | python3 -c "import sys,json; c=json.load(sys.stdin)['Content']; print('tasks:', [t['id'] for t in c] or 'none')"
            if running; then echo "job running: $(cat $PID)"; else echo "no job running"; fi
            tail -n 8 "$LOG" 2>/dev/null ;;
  job-status) if [ ! -f "$JS" ]; then echo '{"state": "idle"}'
              elif grep -q '"running"' "$JS" && ! running; then echo '{"state": "failed", "error": "job died"}'
              else cat "$JS"; echo; fi ;;
  tune)     python3 - "$2" "$3" "$4" <<'P'
import sys, json
name, key, val = sys.argv[1], sys.argv[2], float(sys.argv[3])
assert key in ('gap', 'lat_offset', 'yaw_offset'), 'key must be gap | lat_offset | yaw_offset'
s = json.load(open('stations.json')); s[name][key] = val
json.dump(s, open('stations.json', 'w'), indent=1); print(name, json.dumps(s[name]))
P
            ;;
  teach)    read sid side gap z0 z1 < <(python3 -c "import json,sys;s=json.load(open('stations.json'))[sys.argv[1]];print(s['staging'],s['side'],s['gap'],*s['z'])" "$2") || { echo "unknown station $2"; exit 1; }
            timeout 120 python3 dock.py teach "$2" "$sid" "$side" "$gap" --z "$z0" "$z1" ;;
  selfscan) timeout 60 python3 dock.py selfscan --save ;;
  check)    timeout 90 python3 dock.py check "${2:-D}" ;;
  log)      tail -n 30 -f "$LOG" ;;
  *)        sed -n 2,17p "$0" ;;
esac
