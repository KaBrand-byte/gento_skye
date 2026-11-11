"""Client for the chassis docking system (see chassis/ and docs/chassis-navigation.md).

Standalone (standard library; rclpy only for the ROS transport) and independent of skye_task: `GentoNavigation`
implements the skye_task `interfaces.Navigation` protocol (start / is_arrived_and_stopped / stop) by duck typing,
so the task code can use it without importing anything from this directory except this file.  Every motion job (navigate to the staging
point, square up to the edge with the lidar, straight approach with arm/tray-aware collision checks) runs ON
the chassis; this class only submits a job and reads its status, so a network hiccup cannot leave the chassis
half-controlled.

Transports
  "ros" (default): ROS 2 over the robot's internal Ethernet, std_msgs/String only, via the chassis bridge
         /gento_dock/cmd (JSON commands) and /gento_dock/status (JSON, 2 Hz, transient local).
         Needs rclpy + std_msgs and the robot's ROS env (ROS_DOMAIN_ID=24, rmw_cyclonedds_cpp).
  "ssh": runs run.sh on the chassis over SSH (passwordless key required), e.g. from a PC.

rclpy is imported only when the ROS transport is used, so the package stays standard-library only otherwise.
"""

import json
import shlex
import subprocess
import threading
import time
import uuid

class NavigationError(RuntimeError):
    pass


class GentoNavigation:
    def __init__(self, *, transport="ros", ssh_target="chassis", gaps=None, node=None,
                 ack_timeout=10.0, status_timeout=3.0, run_sh="bash ~/gento_dock/run.sh"):
        if transport not in ("ros", "ssh"):
            raise ValueError("transport 必须是 ros 或 ssh")
        self.transport = transport
        self.ssh_target = ssh_target
        self.gaps = dict(gaps or {})
        self.ack_timeout = ack_timeout
        self.status_timeout = status_timeout
        self.run_sh = run_sh
        self.active = None
        self._status = None
        self._status_at = 0.0
        if transport == "ros":
            self._init_ros(node)

    # ---------------------------------------------------------------- ROS transport
    def _init_ros(self, node):
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        self._string = String
        self._context = None
        if node is None:
            # Private context and executor: never touches an rclpy setup owned by the caller.
            from rclpy.executors import SingleThreadedExecutor
            self._context = rclpy.Context()
            rclpy.init(context=self._context)
            node = rclpy.create_node("skye_task_navigation_%s" % uuid.uuid4().hex[:6], context=self._context)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(node)
            threading.Thread(target=self._executor.spin, daemon=True).start()
        self._node = node
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        node.create_subscription(String, "/gento_dock/status", self._on_status, latched)
        self._publisher = node.create_publisher(
            String, "/gento_dock/cmd", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def _on_status(self, message):
        try:
            self._status = json.loads(message.data)
            self._status_at = time.monotonic()
        except ValueError:
            pass

    def _ros_command(self, **command):
        command["id"] = uuid.uuid4().hex[:8]
        begin = time.monotonic()
        while self._publisher.get_subscription_count() == 0 and time.monotonic() - begin < 5:
            time.sleep(0.1)
        if self._publisher.get_subscription_count() == 0:
            raise NavigationError("未发现底盘导航桥 /gento_dock/cmd，检查 gento-dock-bridge 与 ROS 环境")
        self._publisher.publish(self._string(data=json.dumps(command)))
        while time.monotonic() - begin < self.ack_timeout:
            ack = (self._status or {}).get("ack") or {}
            if ack.get("id") == command["id"]:
                return ack
            time.sleep(0.05)
        raise NavigationError(f"底盘导航桥未应答: {command}")

    def close(self):
        if self.transport == "ros" and self._context is not None:
            self._executor.shutdown()
            self._node.destroy_node()
            self._context.try_shutdown()

    # ---------------------------------------------------------------- SSH transport
    def _ssh(self, *arguments, timeout=20.0):
        command = self.run_sh + " " + " ".join(shlex.quote(a) for a in arguments)
        done = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", self.ssh_target, command],
                              capture_output=True, text=True, timeout=timeout)
        if done.returncode != 0:
            raise NavigationError(f"ssh 调用底盘失败 ({done.returncode}): "
                                  f"{(done.stderr or done.stdout).strip()[-300:]}")
        return done.stdout

    # ---------------------------------------------------------------- status
    def job_status(self):
        """{"state": idle|running|done|failed, "job", "final", "error"} of the latest chassis job."""
        if self.transport == "ros":
            if self._status is None or time.monotonic() - self._status_at > self.status_timeout:
                raise NavigationError("底盘导航桥状态超时")
            return self._status.get("job") or {"state": "unknown"}
        lines = self._ssh("job-status").strip().splitlines()
        return json.loads(lines[-1]) if lines else {"state": "idle"}

    def pose(self):
        """Map pose [x_m, y_m, yaw_deg] reported by the bridge (ROS transport only)."""
        return (self._status or {}).get("pose")

    # ---------------------------------------------------------------- Navigation protocol
    def start(self, destination):
        """Submit a navigation + docking job; the return is not an arrival indication."""
        gap = self.gaps.get(destination)
        if self.transport == "ros":
            ack = self._ros_command(cmd="goto", station=destination, gap=gap)
            if not ack.get("ok"):
                raise NavigationError(f"底盘拒绝导航任务: {ack.get('out')}")
        else:
            out = self._ssh("goto", destination, *(["%.3f" % gap] if gap is not None else []))
            if "started job" not in out:
                raise NavigationError(f"底盘拒绝导航任务: {out.strip()}")
        self.active = destination

    def is_arrived_and_stopped(self):
        """True only after the docking job finished successfully (docked and standing still)."""
        status = self.job_status()
        return status.get("state") == "done" and status.get("job") == self.active

    def raise_if_failed(self):
        status = self.job_status()
        if status.get("state") == "failed" and status.get("job") in (self.active, "stopped"):
            raise NavigationError(status.get("error") or "导航失败")

    def stop(self):
        if self.transport == "ros":
            self._ros_command(cmd="stop")
        else:
            self._ssh("stop")
        self.active = None

    def tune(self, destination, key, value):
        """Persist a fine-tune value on the chassis: gap (m) | lat_offset (m, + left) | yaw_offset (deg, + left)."""
        if self.transport == "ros":
            ack = self._ros_command(cmd="tune", station=destination, key=key, value=float(value))
            if not ack.get("ok"):
                raise NavigationError(ack.get("out"))
            return ack.get("out")
        return self._ssh("tune", destination, key, str(float(value)))
