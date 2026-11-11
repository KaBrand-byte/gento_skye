#!/bin/bash
# Boot-time safety + localisation check for the Gento chassis.  Runs from cron @reboot; safe to run by hand.
# Never moves the robot: it only cancels leftover tasks and (if needed) resets the localisation estimate.
DIR=$(cd "$(dirname "$0")" && pwd)
LOG=$DIR/boot_check.log; exec >>"$LOG" 2>&1
echo "===== $(date '+%F %T') boot_check start"
source $DIR/env.sh
trap "ros2 daemon stop >/dev/null 2>&1" EXIT
PARK="${GENTO_PARK:-0.12 0.155 80}"   # charging spot (map x y yaw_deg); override with GENTO_PARK
match() { python3 -c "import sys,json;print(json.loads(sys.stdin.read().strip().splitlines()[0])['$1'])"; }
ok()    { python3 -c "import sys;sys.exit(0 if float('$1')>=0.85 else 1)"; }

for i in $(seq 1 90); do ros2 service list 2>/dev/null | grep -q /server/CancelAllTask && break; sleep 5; done
echo "1) cancel tasks left over from before the reboot"
ros2 service call /server/CancelAllTask server/srv/CancelAllTask "{}" | grep -a Response
echo "2) wait for localisation"
for i in $(seq 1 60); do timeout 5 ros2 topic echo --once slam/LocationData >/dev/null 2>&1 && break; sleep 2; done
sleep 20                          # let the vendor initialisation finish first
echo "3) scan-to-map check"
R=$(timeout 60 python3 $DIR/locmatch.py check); echo "$R"; M=$(echo "$R" | match match)
if ok "$M"; then echo "RESULT: LOCALISATION OK ($M)"; exit 0; fi
echo "4) try the charging spot"
R=$(timeout 90 python3 $DIR/locmatch.py at $PARK --apply); echo "$R"
R=$(timeout 60 python3 $DIR/locmatch.py check); echo "$R"; M=$(echo "$R" | match match)
if ok "$M"; then echo "RESULT: FIXED AT CHARGING SPOT ($M)"; exit 0; fi
echo "5) global search"
timeout 400 python3 $DIR/locmatch.py global --apply
R=$(timeout 60 python3 $DIR/locmatch.py check); echo "$R"; M=$(echo "$R" | match match)
if ok "$M"; then echo "RESULT: FIXED BY GLOBAL SEARCH ($M)"; exit 0; fi
echo "RESULT: LOCALISATION BAD ($M) - do not send tasks; relocate by hand"; exit 1
