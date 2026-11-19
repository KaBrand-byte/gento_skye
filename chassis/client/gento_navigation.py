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
import logging
import os
import shlex
import subprocess
import threading
import time
import uuid

# ROS transport: when the bridge's status goes stale, read the job status / send the stop over SSH instead
# (2026-11-17: the status stopped for > 3 s on the loaded chassis, the task aborted, and the ROS stop could not be
# delivered either - the chassis drove on to D on its own).  Needs the Orin's key login (gento setup).
FALLBACK_SSH = os.environ.get("GENTO_CHASSIS", "sunrise@6.6.7.6")
JOB_STATUS_SH = ("cd ~/gento_dock && if [ ! -f job_status.json ]; then echo '{\"state\": \"idle\"}'; "
                 "elif grep -q '\"running\"' job_status.json && ! kill -0 \"$(cat run.pid 2>/dev/null)\" 2>/dev/null; "
                 "then echo '{\"state\": \"failed\", \"error\": \"job died\"}'; else cat job_status.json; echo; fi")
log = logging.getLogger(__name__)

class NavigationError(RuntimeError):
    pass


class GentoNavigation:
    def __init__(self, *, transport="ros", ssh_target="chassis", gaps=None, node=None,
                 ack_timeout=10.0, status_timeout=3.0, run_sh="bash ~/gento_dock/run.sh", route=None,
                 fallback_ssh=FALLBACK_SSH, status_lost=60.0):
        if transport not in ("ros", "ssh"):
            raise ValueError("transport 必须是 ros 或 ssh")
        self.transport = transport
        self.ssh_target = ssh_target
        self.gaps = dict(gaps or {})
        # route variation for demos, e.g. {"via": "random", "jitter": 0.08} (see chassis/route.py); empty = fixed
        self.route = {k: v for k, v in dict(route or {}).items() if k in ("via", "jitter") and v}
        self.ack_timeout = ack_timeout
        self.status_timeout = status_timeout
        self.run_sh = run_sh
        self.active = None
        self._seen_active = False       # True once the chassis reported OUR job (older results are ignored)
        self._status = None
        self._status_at = 0.0
        self.fallback_ssh = fallback_ssh            # ROS transport: SSH target used while the bridge is silent
        # s without any status (ROS nor SSH): the job counts as lost.  The chassis job runs on its own through a drop
        # of the Orin link (2026-11-17: eth1 dropped for 3-4 s at a time, the job drove on to D): wait it out and
        # pick the result up afterwards instead of giving up after 15 s
        self.status_lost = status_lost
        self._ssh_job, self._ssh_at, self._ssh_tried = None, 0.0, 0.0
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
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
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

    def _spin(self):
        context = self._context
        try:
            while context.ok():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception:
            pass                                     # context shut down while waiting

    def close(self):
        """Stop the private ROS executor cleanly (only when this object created it)."""
        if self.transport == "ros" and self._context is not None:
            context, self._context = self._context, None
            self._stop_context(context)

    def _stop_context(self, context):
        context.try_shutdown()                       # makes _spin return
        self._thread.join(timeout=2.0)
        try:
            self._executor.shutdown(timeout_sec=1.0)
            self._node.destroy_node()
        except Exception:
            pass

    # ---------------------------------------------------------------- SSH transport
    def _ssh_raw(self, command, timeout=20.0):
        """ROS transport's fallback: a shell command on the chassis over SSH (key login)."""
        done = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", self.fallback_ssh, command],
                              capture_output=True, text=True, timeout=timeout)
        if done.returncode != 0:
            raise NavigationError(f"ssh 底盘失败 ({done.returncode}): {(done.stderr or done.stdout).strip()[-200:]}")
        return done.stdout

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
            now = time.monotonic()
            if self._status is not None and now - self._status_at <= self.status_timeout:
                return self._status.get("job") or {"state": "unknown"}
            # the bridge is silent (DDS / a loaded chassis): ask over SSH, at most once a second
            if self.fallback_ssh and now - self._ssh_tried >= 1.0:
                self._ssh_tried = now
                try:
                    lines = self._ssh_raw(JOB_STATUS_SH, timeout=5.0).strip().splitlines()
                    self._ssh_job, self._ssh_at = (json.loads(lines[-1]) if lines else {"state": "idle"}), now
                    log.warning("底盘导航桥 %.1f s 无状态：改由 ssh 读取作业状态 %s",
                                now - self._status_at if self._status is not None else -1, self._ssh_job.get("state"))
                except (NavigationError, ValueError, subprocess.TimeoutExpired) as error:
                    log.warning("底盘导航桥无状态，ssh 读取也失败: %s", error)
            newest = max(self._status_at if self._status is not None else 0.0, self._ssh_at)
            if self._ssh_job is not None and self._ssh_at >= (self._status_at if self._status is not None else 0.0):
                job = self._ssh_job
            else:
                job = (self._status or {}).get("job")
            if job is None or now - newest > self.status_lost:
                raise NavigationError(f"底盘导航桥状态超时（ROS 与 ssh 都已 {self.status_lost:.0f} s 无应答）："
                                      "底盘网口掉线或底盘程序停止 - 运行 gento doctor")
            return job
        lines = self._ssh("job-status").strip().splitlines()
        return json.loads(lines[-1]) if lines else {"state": "idle"}

    def pose(self):
        """Map pose [x_m, y_m, yaw_deg] reported by the bridge (ROS transport only)."""
        return (self._status or {}).get("pose")

    # ---------------------------------------------------------------- Navigation protocol
    def _peek_job(self):
        """The chassis job status as last received over ROS, without waiting (None over ssh: there run.sh marks
        the new job as running before it returns, so no stale result can be read afterwards)."""
        if self.transport != "ros" or self._status is None:
            return None
        return dict(self._status.get("job") or {})

    def _begin(self, destination, baseline):
        self.active = destination
        self._seen_active = False
        self._baseline = baseline

    def start(self, destination):
        """Submit a navigation + docking job; the return is not an arrival indication."""
        gap = self.gaps.get(destination)
        baseline = self._peek_job()
        if self.transport == "ros":
            ack = self._ros_command(cmd="goto", station=destination, gap=gap, **self.route)
            if not ack.get("ok"):
                raise NavigationError(f"底盘拒绝导航任务: {ack.get('out')}")
        else:
            extra = []
            if self.route.get("via"):
                extra += ["--via", str(self.route["via"])]
            if self.route.get("jitter"):
                extra += ["--jitter", "%.3f" % float(self.route["jitter"])]
            out = self._ssh("goto", destination, *(["%.3f" % gap] if gap is not None else []), *extra)
            if "started job" not in out:
                raise NavigationError(f"底盘拒绝导航任务: {out.strip()}")
        self._begin(destination, baseline)

    def start_undock(self):
        """Reverse straight out along the last docking approach (e.g. after placing, before the arms move).
        Completion is polled like a destination: is_arrived_and_stopped() / raise_if_failed()."""
        baseline = self._peek_job()
        if self.transport == "ros":
            ack = self._ros_command(cmd="undock")
            if not ack.get("ok"):
                raise NavigationError(f"底盘拒绝后退任务: {ack.get('out')}")
        else:
            out = self._ssh("undock")
            if "started job" not in out:
                raise NavigationError(f"底盘拒绝后退任务: {out.strip()}")
        self._begin("undock", baseline)

    def _current(self):
        """Status of the job we started.  Until the chassis has reported our job once, any other job's result
        (e.g. a stale "stopped" left by the previous run) is ignored and reads as still starting."""
        status = self.job_status()
        if status.get("job") == self.active and self._is_ours(status):
            self._seen_active = True
            return status
        if self._seen_active and status.get("job") == "stopped":
            return status                       # our job was stopped (run.sh stop) after it had started
        return {"state": "starting", "job": self.active}

    def _is_ours(self, status):
        """A result with the same job name can still be the PREVIOUS job's (e.g. goto T while already at T): the
        bridge's status lags the new job by up to ~1 s (2026-11-16: an old "done T" was taken as arrival 8 s
        before the new docking ended).  Ours once it is running, or newer than the status seen at the start."""
        if self._seen_active or status.get("state") == "running":
            return True
        base = getattr(self, "_baseline", None)
        if not base or base.get("job") != status.get("job"):
            return True
        return base.get("t") is not None and status.get("t") is not None and status["t"] > base["t"]

    def is_arrived_and_stopped(self):
        """True only after the docking job finished successfully (docked and standing still)."""
        status = self._current()
        return status.get("state") == "done" and status.get("job") == self.active

    def note(self):
        """Progress note of the running job, e.g. 'WAITING: ... blocked ...' while an object is in the way."""
        status = self._current()
        return status.get("note") if status.get("state") == "running" else None

    def raise_if_failed(self):
        status = self._current()
        if status.get("state") == "failed":
            raise NavigationError(status.get("error") or "导航失败")

    def stop(self):
        """Stop the chassis job.  ROS transport: over the bridge while it answers; when its status is stale or
        the ROS stop fails, (also) over SSH - the stop has to arrive."""
        if self.transport == "ros":
            fresh = self._status is not None and time.monotonic() - self._status_at <= self.status_timeout
            if fresh:
                try:
                    self._ros_command(cmd="stop")
                    self.active = None
                    return
                except NavigationError as error:
                    log.error("ROS 停止失败，改用 ssh 停止: %s", error)
            elif self._publisher.get_subscription_count() > 0:      # fire and forget, the SSH stop follows
                self._publisher.publish(self._string(data=json.dumps({"id": uuid.uuid4().hex[:8], "cmd": "stop"})))
            if not self.fallback_ssh:
                raise NavigationError("底盘导航桥无应答，且未配置 ssh 停止")
            self._ssh_raw(self.run_sh + " stop", timeout=20.0)
            log.warning("已通过 ssh 停止底盘 (%s)", self.fallback_ssh)
        else:
            self._ssh("stop")
        self.active = None

    def _query(self, timeout, **command):
        if self.transport == "ros":
            saved, self.ack_timeout = self.ack_timeout, timeout
            try:
                ack = self._ros_command(**command)
            finally:
                self.ack_timeout = saved
            return ack.get("out", "")
        args = [command["cmd"]] + ([command["station"]] if "station" in command else [])
        return self._ssh(*args, timeout=timeout)

    def selfscan(self):
        """No motion: measure the arms (+tray) with the lidar and store them on the chassis.
        Needs nothing else within ~1 m in front of the robot; arms in the carry pose."""
        return self._query(90.0, cmd="selfscan")

    def check(self, destination):
        """No motion: report what docking at `destination` would do from the current pose."""
        return self._query(120.0, cmd="check", station=destination)

    def tune(self, destination, key, value):
        """Persist a fine-tune value on the chassis: gap (m) | lat_offset (m, + left) | yaw_offset (deg, + left)."""
        if self.transport == "ros":
            ack = self._ros_command(cmd="tune", station=destination, key=key, value=float(value))
            if not ack.get("ok"):
                raise NavigationError(ack.get("out"))
            return ack.get("out")
        return self._ssh("tune", destination, key, str(float(value)))
