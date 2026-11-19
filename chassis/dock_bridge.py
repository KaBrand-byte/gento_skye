#!/usr/bin/env python3
"""ROS 2 bridge on the chassis so programs on the Orin (or anywhere on ROS_DOMAIN_ID 24) can use the docking
system with STANDARD messages only (no vendor message packages needed on the client side).

  /gento_dock/cmd     std_msgs/String  JSON {"id": "...", "cmd": "goto", "station": "T", "gap": 0.30}
                                       cmds: goto (station; gap, via, jitter optional) | undock | stop |
                                             tune (station, key, value) | selfscan | check (station)
  /gento_dock/status  std_msgs/String  JSON, 2 Hz, transient-local (late joiners get the last one):
                      {"job": {...job_status.json...}, "ack": {"id", "ok", "out"}, "pose": [x, y, yaw_deg], "t": ...,
                       "vendor": {...vendor_watchdog.json...} or null when the watchdog is not installed}

Every motion job is still run by run.sh / job_wrap.py exactly like from gento.ps1 (detached, one at a time).
"""
import json, math, os, subprocess, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from slam.msg import LocationData

HERE = os.path.dirname(os.path.abspath(__file__))
RUN = ['bash', os.path.join(HERE, 'run.sh')]
JS, PIDF = os.path.join(HERE, 'job_status.json'), os.path.join(HERE, 'run.pid')
WATCHDOG = os.path.join(HERE, 'vendor_watchdog.json')


def read_job():
    """What 'run.sh job-status' prints: job_status.json, or a running job whose process is gone -> failed."""
    if not os.path.exists(JS):
        return {'state': 'idle'}
    with open(JS) as fh:
        job = json.load(fh)
    if job.get('state') == 'running':
        try:
            with open(PIDF) as fh:
                os.kill(int(fh.read().strip()), 0)
        except (OSError, ValueError):
            return {'state': 'failed', 'error': 'job died'}
    return job


class Bridge(Node):
    def __init__(self):
        super().__init__('gento_dock_bridge')
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(String, '/gento_dock/status', q)
        self.create_subscription(String, '/gento_dock/cmd', self.on_cmd, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(LocationData, 'slam/LocationData', self.on_pose, qos_profile_sensor_data)
        self.ack = None; self.pose = None; self.job = {'state': 'idle'}; self.vendor = None; self.lock = threading.Lock()
        self.create_timer(0.5, self.publish)
        threading.Thread(target=self.poll_job, daemon=True).start()

    def on_pose(self, m):
        p = m.pose; self.pose = [round(p.position.x, 3), round(p.position.y, 3), round(math.degrees(p.orientation.z), 1)]

    def poll_job(self):
        """job_status.json read directly - like 'run.sh job-status', whose bash sources the ROS environment on
        every call: 3.8 s and 2.7 s of CPU each on the loaded chassis (load 14 on 4 cores), every 0.5 s - the job
        status lagged ~4 s and the status to the Orin once stopped for > 3 s (2026-11-17)."""
        while rclpy.ok():
            try:
                self.job = read_job()
            except Exception as e:
                self.job = {'state': 'unknown', 'error': str(e)}
            try:                                    # the vendor watchdog's state: the Orin waits for its recovery
                with open(WATCHDOG) as fh:
                    self.vendor = json.load(fh)
            except (OSError, ValueError):
                self.vendor = None
            time.sleep(0.5)

    def on_cmd(self, msg):
        try:
            c = json.loads(msg.data)
        except Exception:
            parts = msg.data.split(); c = {'cmd': parts[0] if parts else '', 'args': parts[1:]}
        cid = c.get('id', '')
        if c.get('cmd') == 'goto':
            args = ['goto', str(c['station'])] + ([('%.3f' % float(c['gap']))] if c.get('gap') is not None else [])
            if c.get('via'):                        # route variation: none | random | waypoint name
                args += ['--via', str(c['via'])]
            if c.get('jitter'):                     # m: random spot near the safe point
                args += ['--jitter', '%.3f' % float(c['jitter'])]
        elif c.get('cmd') == 'undock':              # reverse straight out along the last docking approach
            args = ['undock']
        elif c.get('cmd') == 'stop':
            args = ['stop']
        elif c.get('cmd') == 'tune':
            args = ['tune', str(c['station']), str(c['key']), str(float(c['value']))]
        elif c.get('cmd') == 'selfscan':            # no motion: measure arms/tray, store in robot_model.json
            args = ['selfscan']
        elif c.get('cmd') == 'check':               # no motion: what docking at a station would do from here
            args = ['check', str(c['station'])]
        else:
            self.ack = {'id': cid, 'ok': False, 'out': 'unknown command: %s' % msg.data}; self.publish(); return
        self.get_logger().info('cmd %s: %s' % (cid, ' '.join(args)))

        def run():
            r = subprocess.run(RUN + args, capture_output=True, text=True, timeout=150)
            out = (r.stdout + r.stderr).strip()
            ok = r.returncode == 0 and 'BUSY' not in out and 'unknown station' not in out
            self.ack = {'id': cid, 'ok': ok, 'out': out[-400:]}; self.publish()
        threading.Thread(target=run, daemon=True).start()

    def publish(self):
        self.pub.publish(String(data=json.dumps({'job': self.job, 'ack': self.ack, 'pose': self.pose, 't': time.time(),
                                                 'vendor': self.vendor})))


def main():
    rclpy.init(); n = Bridge()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
