#!/bin/bash
# Install / update the chassis docking system on the Gento chassis board (run ON the chassis as the robot user).
#   bash install.sh [TARGET_DIR]      default TARGET_DIR: ~/gento_dock
# Copies the scripts, keeps existing site data (stations.json, robot_model.json, logs), installs the systemd
# units (needs passwordless sudo) and enables the boot check timer and the ROS 2 bridge.  Never moves the robot.
set -e
SRC=$(cd "$(dirname "$0")" && pwd); DIR=${1:-$HOME/gento_dock}; USER_=$(id -un)
mkdir -p "$DIR"
for f in dock.py robot_model.py dock_bridge.py job_wrap.py run.sh nav.py locmatch.py boot_check.sh edge.py over_map.py env.sh sim_test.py; do
  install -m 0755 "$SRC/$f" "$DIR/$f"
done
[ -f "$DIR/stations.json" ]    || cp "$SRC/stations.example.json" "$DIR/stations.json"
[ -f "$DIR/robot_model.json" ] || echo "note: no robot_model.json yet - run 'run.sh selfscan' with the arms/tray in carry pose"
for u in gento-boot-check.service gento-boot-check.timer gento-dock-bridge.service gento-dock-bridge.timer; do
  sed "s|@DIR@|$DIR|g; s|@USER@|$USER_|g" "$SRC/systemd/$u" | sudo -n tee /etc/systemd/system/$u > /dev/null
done
sudo -n systemctl daemon-reload
sudo -n systemctl disable gento-boot-check.service > /dev/null 2>&1 || true     # started by its timer only
sudo -n systemctl enable gento-boot-check.timer gento-dock-bridge.service gento-dock-bridge.timer
sudo -n systemctl restart gento-dock-bridge.service
echo "installed to $DIR; bridge restarted (it waits 60 s before starting)"
