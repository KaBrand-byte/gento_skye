#!/usr/bin/env python3
"""ROS 2 bridge on the chassis so programs on the Orin (or anywhere on ROS_DOMAIN_ID 24) can use the docking
system with STANDARD messages only (no vendor message packages needed on the client side).

  /gento_dock/cmd     std_msgs/String  JSON {"id": "...", "cmd": "goto", "station": "T", "gap": 0.30}
                                       cmds: goto (station, gap optional) | stop | tune (station, key, value)
  /gento_dock/status  std_msgs/String  JSON, 2 Hz, transient-local (late joiners get the last one):
                      {"job": {...job_status.json...}, "ack": {"id", "ok", "out"}, "pose": [x, y, yaw_deg], "t": ...}

Every motion job is still run by run.sh / job_wrap.py exactly like from gento.ps1 (detached, one at a time).
"""
import json, math, os, subprocess, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from slam.msg import LocationData

RUN = ['bash', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'run.sh')]


class Bridge(Node):
    def __init__(self):
        super().__init__('gento_dock_bridge')
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(String, '/gento_dock/status', q)
        self.create_subscription(String, '/gento_dock/cmd', self.on_cmd, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(LocationData, 'slam/LocationData', self.on_pose, qos_profile_sensor_data)
        self.ack = None; self.pose = None; self.job = {'state': 'idle'}; self.lock = threading.Lock()
        self.create_timer(0.5, self.publish)
        threading.Thread(target=self.poll_job, daemon=True).start()

    def on_pose(self, m):
        p = m.pose; self.pose = [round(p.position.x, 3), round(p.position.y, 3), round(math.degrees(p.orientation.z), 1)]

    def poll_job(self):
        while rclpy.ok():
            try:
                out = subprocess.run(RUN + ['job-status'], capture_output=True, text=True, timeout=10).stdout.strip().splitlines()
                if out:
                    self.job = json.loads(out[-1])
            except Exception as e:
                self.job = {'state': 'unknown', 'error': str(e)}
            time.sleep(0.5)

    def on_cmd(self, msg):
        try:
            c = json.loads(msg.data)
        except Exception:
            parts = msg.data.split(); c = {'cmd': parts[0] if parts else '', 'args': parts[1:]}
        cid = c.get('id', '')
        if c.get('cmd') == 'goto':
            args = ['goto', str(c['station'])] + ([('%.3f' % float(c['gap']))] if c.get('gap') is not None else [])
        elif c.get('cmd') == 'stop':
            args = ['stop']
        elif c.get('cmd') == 'tune':
            args = ['tune', str(c['station']), str(c['key']), str(float(c['value']))]
        else:
            self.ack = {'id': cid, 'ok': False, 'out': 'unknown command: %s' % msg.data}; self.publish(); return
        self.get_logger().info('cmd %s: %s' % (cid, ' '.join(args)))

        def run():
            r = subprocess.run(RUN + args, capture_output=True, text=True, timeout=60)
            out = (r.stdout + r.stderr).strip()
            ok = r.returncode == 0 and 'BUSY' not in out and 'unknown station' not in out
            self.ack = {'id': cid, 'ok': ok, 'out': out[-400:]}; self.publish()
        threading.Thread(target=run, daemon=True).start()

    def publish(self):
        self.pub.publish(String(data=json.dumps({'job': self.job, 'ack': self.ack, 'pose': self.pose, 't': time.time()})))


def main():
    rclpy.init(); n = Bridge()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
