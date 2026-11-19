#!/bin/bash
# One-command start of the full task ON THE ORIN (arms via SDK 0407 aarch64, chassis via the ROS 2 bridge).
# Runs inside tmux, so a Wi-Fi drop of the operator's SSH session does not interrupt the robot.
#
#   bash orin_task.sh start [runner args]   start (or re-attach to) the task in tmux session "gento_task"
#   bash orin_task.sh attach                re-attach (e.g. after a Wi-Fi drop); Ctrl-b d detaches again
#   bash orin_task.sh stop                  Ctrl-C to the task (stops arms + chassis) and software-stop the chassis
#   bash orin_task.sh check                 no motion: config check and SDK version
#   bash orin_task.sh doctor                no motion: health check of the Orin end (link, bridge, arms, lift); exit 2 = do not start
#   bash orin_task.sh grip open|close [left|right]   only the grippers (arms do not move); not while a task
#                                           runs - in the task window use o / c instead.  Stays as it is after exit.
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
    LOGDIR="$REPO/runs/logs"; mkdir -p "$LOGDIR"; STAMP=$(date +%Y%m%d-%H%M%S)
    CH=${GENTO_CHASSIS:-sunrise@6.6.7.6}
    chlines() { ssh -o BatchMode=yes -o ConnectTimeout=5 "$CH" 'wc -l < ~/gento_dock/run.log' 2>/dev/null; }
    for i in $(seq 1 "$N"); do
      [ "$N" -gt 1 ] && echo && echo "=================== 第 $i/$N 次 ==================="
      RES="$REPO/runs/with-navigation.json"; [ "$N" -gt 1 ] && RES="$REPO/runs/cycle-$(printf %02d "$i").json"
      # the complete output of every run goes to runs/logs/ (tee -i: Ctrl-C stops the task, not the log)
      TLOG="$LOGDIR/task-$STAMP.log"; [ "$N" -gt 1 ] && TLOG="$LOGDIR/task-$STAMP-c$(printf %02d "$i").log"
      BODY="$TLOG.part"                         # the run as it happens; the response time goes on top at the end
      echo "===== $(date '+%F %T') gento task $i/$N: $*  (result $RES)" > "$BODY"
      L0=$(chlines)
      # chassis eth1 (the link to this Orin) and load: 2026-11-17 its MAC TX path hung and was reset 39 times in
      # 9 min (load 12-14 on 4 cores) - the bridge status and the stop were lost, the localisation reset
      E0=$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$CH" 'cat /sys/class/net/eth1/carrier_changes; cut -d" " -f1-3 /proc/loadavg' 2>/dev/null)
      GENTO_CYCLE="$i/$N" stdbuf -oL -eL python3 -u chassis/client/run_task_with_navigation.py \
        --config "${GENTO_TASK_CONFIG:-config/default.json}" --result "$RES" "$@" 2>&1 | tee -i -a "$BODY"
      rc=${PIPESTATUS[0]}; [ $rc -eq 0 ] && ok=$((ok + 1))
      echo "===== $(date '+%F %T') exit $rc" >> "$BODY"
      OFF=""; NET=""
      if [ -n "$L0" ]; then                     # the chassis side of the same run (docking jobs, run.log)
        { echo "===== chassis ~/gento_dock/run.log from line $((L0 + 1))"
          ssh -o BatchMode=yes -o ConnectTimeout=5 "$CH" "tail -n +$((L0 + 1)) ~/gento_dock/run.log"; } >> "$BODY" 2>&1
        E1=$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$CH" 'cat /sys/class/net/eth1/carrier_changes; cut -d" " -f1-3 /proc/loadavg' 2>/dev/null)
        C0=$(echo "$E0" | head -1); C1=$(echo "$E1" | head -1)
        NET="chassis eth1 link changes during this run: $(( ${C1:-0} - ${C0:-0} ))   load before: $(echo "$E0" | tail -1)   after: $(echo "$E1" | tail -1)"
        echo "===== $NET" >> "$BODY"
        if [ -n "$C0" ] && [ -n "$C1" ] && [ "$C1" -gt "$C0" ]; then
          { echo "===== chassis kernel log (eth1)"; ssh -o BatchMode=yes -o ConnectTimeout=5 "$CH" \
              'dmesg -T | grep -E "eth1|tx_fifo|tx queue" | tail -n 60'; } >> "$BODY" 2>&1
        fi
        T1=$(date +%s.%N); TC=$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$CH" 'date +%s.%N' 2>/dev/null); T2=$(date +%s.%N)
        [ -n "$TC" ] && OFF=$(python3 -c "print('%.3f' % (($T1 + $T2) / 2 - $TC))")     # Orin clock - chassis clock
      else
        echo "(chassis log not included: no key login Orin -> chassis yet - 'gento setup' once)" | tee -a "$BODY"
      fi
      # response time (first lidar input -> first chassis command, < 1.2 s) on top, the complete run below;
      # runs/response_times.csv gets one line per run (appended, never overwritten)
      PLACE=D; prev=""; for a in "$@"; do [ "$prev" = --place ] && PLACE=$a; prev=$a; done   # the leg that counts
      python3 chassis/client/response_summary.py "$BODY" --place "$PLACE" ${OFF:+--offset "$OFF"} --name "$(basename "$TLOG")" \
        --csv "$REPO/runs/response_times.csv" > "$TLOG.head" 2>&1
      [ -n "$NET" ] && echo "$NET" >> "$TLOG.head"
      cat "$TLOG.head"
      { cat "$TLOG.head"; echo; cat "$BODY"; } > "$TLOG" && rm -f "$TLOG.head" "$BODY"
      echo "log: $TLOG"
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
  grip)       # skye_task's own gripper tool over the arm SDK link; no --disable-on-exit: it keeps holding
    case "$2" in open|close) ;; *) echo "usage: orin_task.sh grip open|close [left|right]"; exit 1 ;; esac
    if pgrep -f "^python3 chassis/client/run_task_with_navigation.py" >/dev/null; then
      echo "a task is running: use o / c in the task window (gento task-attach)"; exit 1
    fi
    env_setup
    python3 -m skye_task gripper "$2" --transport sdk --robot-config "${GENTO_TASK_CONFIG:-config/default.json}" \
      --config config/gripper-can.json --side "${3:-both}" --hold-seconds 1 --brief --result runs/gripper-manual.json ;;
  doctor)     # NO MOTION: the Orin end - link to the chassis, docking bridge over ROS, arm controller; exit 2 = do not start
    set +e; env_setup; CH=${GENTO_CHASSIS:-sunrise@6.6.7.6}; HOST=${CH#*@}; fail=0; warn=0
    say() { printf '  %s %s\n' "$1" "$2"; [ "$1" = '[失败]' ] && fail=$((fail + 1)); [ "$1" = '[警告]' ] && warn=$((warn + 1)); true; }
    echo "Orin 体检  $(date '+%F %T')"
    loss=$(ping -c 10 -i 0.2 -W 1 "$HOST" 2>/dev/null | grep -oE '[0-9.]+% packet loss' | cut -d% -f1)
    case "${loss:-100}" in
      0)   say '[ OK ]' "到底盘 $HOST 的网络：10/10 通" ;;
      100) say '[失败]' "到底盘 $HOST 完全不通  ->  底盘是否开机；Orin 与底盘之间的网线/接口" ;;
      *)   say '[警告]' "到底盘 $HOST 丢包 ${loss}%  ->  网口在掉线：演示前整机断电重启；仍丢包就换网线并报厂家" ;;
    esac
    if ssh -o BatchMode=yes -o ConnectTimeout=5 "$CH" true 2>/dev/null; then say '[ OK ]' "ssh 免密登录底盘正常"
    else say '[失败]' "ssh 登录底盘失败  ->  网络不通，或还没做过 gento setup"; fi
    IF=$(ip -o -4 addr 2>/dev/null | awk '$4 ~ /^6\.6\.7\./ {print $2; exit}')
    if [ -n "$IF" ]; then
      cc=$(cat "/sys/class/net/$IF/carrier_changes" 2>/dev/null)
      if [ "${cc:-0}" -gt 2 ]; then say '[警告]' "Orin 网口 $IF 开机以来断开过 $(( (cc - 1) / 2 )) 次  ->  查网线和接口"
      else say '[ OK ]' "Orin 网口 $IF 开机以来没断过"; fi
    fi
    n=$(timeout 20 python3 - <<'P' 2>/dev/null
import sys, time; sys.path.insert(0, 'chassis/client')
from gento_navigation import GentoNavigation
nav = GentoNavigation(fallback_ssh=None); seen = set(); t0 = time.time()
while time.time() - t0 < 5:
    s = nav._status
    if s:
        seen.add(s.get('t'))
    time.sleep(0.1)
nav.close(); print(len(seen))
P
)
    if [ "${n:-0}" -ge 6 ]; then say '[ OK ]' "导航桥状态经 ROS 收到：5 s 内 $n 条"
    elif [ "${n:-0}" -gt 0 ]; then say '[警告]' "导航桥状态经 ROS 只收到 $n 条（应约 10 条）  ->  底盘过载或网口不稳"
    else say '[失败]' "导航桥状态经 ROS 一条都没收到  ->  看底盘体检里的导航桥一项；网络通的话检查 ROS_DOMAIN_ID/CycloneDDS 配置"; fi
    if pgrep -f "^python3 chassis/client/run_task_with_navigation.py" >/dev/null; then
      say '[警告]' "已有任务在运行（手臂检查跳过）  ->  gento task-attach 查看"
    else                                      # arm controller, read only: errors, lift height, arm pose
      arms=$(timeout 60 python3 chassis/client/arm_check.py --config "${GENTO_TASK_CONFIG:-config/default.json}" 2>&1)
      [ -n "$arms" ] || arms="  [失败] 手臂检查没有输出（超时？）  ->  手臂控制器是否上电"
      echo "$arms" | grep -E '^  \[' || echo "  [失败] 手臂检查出错：$(echo "$arms" | tail -n 1)"
      fail=$((fail + $(echo "$arms" | grep -c '^  \[失败\]'))); warn=$((warn + $(echo "$arms" | grep -c '^  \[警告\]')))
      echo "$arms" | grep -qE '^  \[' || fail=$((fail + 1))
    fi
    if [ $fail -gt 0 ]; then echo "结论：不能开始（$fail 项失败）"; exit 2; fi
    [ $warn -gt 0 ] && echo "结论：可以开始（$warn 项警告，看一下）" || echo "结论：一切正常，可以开始" ;;
  check)
    env_setup
    python3 -m skye_task check --config "${GENTO_TASK_CONFIG:-config/default.json}" > /dev/null && echo "config ok"
    python3 -c "
import sys; sys.path.insert(0, '$GENTO_SDK_ROOT/PYTHON_SDK')
import GentoRobot; print('SDK', GentoRobot.GentoRobot(precision=6).get_sdk_version())" ;;
  *) sed -n 2,13p "$0" ;;
esac
