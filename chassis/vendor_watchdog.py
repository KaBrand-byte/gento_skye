#!/usr/bin/env python3
"""vendor_watchdog.py - keeps the vendor chassis programs running, so a task can go on after they stop.

The vendor programs (linkhou_robot.service: driver_node, localisation, controller_core, robot_server) publish the
lidar, the chassis state and the pose.  The service is Restart=no; on 2026-11-16 and 11-18 it did not start at
all after a boot (no journal entries), and nothing worked although ping / ssh did.

Every PERIOD s (systemd service gento-vendor-watchdog, user of the docking system, env.sh sourced):
  - programs gone (service not active or a process missing, DEBOUNCE checks in a row), or a job ended with
    "no vendor data" and a 5 s check of the topics confirms it -> stop the running job (the chassis cannot move
    without its driver anyway), restart the service (sudo -n), wait for the data, cancel leftover vendor tasks,
    relocalise (at T, at D, else the whole map) -> state ok
  - at most MAX_RESTARTS restarts in RESTART_WINDOW s, then state gave_up (power-cycle the chassis)
  - never relocalises while a motion job runs; leaves the first minutes after a boot to boot_check.sh
State for the Orin (via the docking bridge) and gento doctor: vendor_watchdog.json
  {"state": booting|ok|restarting|relocating|relocate_failed|gave_up|down, "ok_since": chassis epoch of the last
   full check (programs up, data flowing, localisation matching the map), "restarts": [...], "event", "match", "t"}
The task on the Orin waits until ok_since is newer than its failed job, then drives the interrupted leg again.
"""
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import doctor  # noqa: E402  (process names, service name, match threshold)

STATE_FILE = os.path.join(HERE, 'vendor_watchdog.json')
LOG_FILE = os.path.join(HERE, 'watchdog.log')
JS, PIDF, ST_FILE = (os.path.join(HERE, f) for f in ('job_status.json', 'run.pid', 'stations.json'))
PERIOD = 5.0           # s between checks
DEBOUNCE = 3           # checks in a row with the programs gone before acting (~15 s)
BOOT_GRACE = 180       # s after boot: the vendor start-up and boot_check.sh first
MAX_RESTARTS, RESTART_WINDOW = 3, 600
UP_TIMEOUT = 150       # s for the restarted programs to publish again
SETTLE = 20            # s after they publish, before relocalising (vendor initialisation, as boot_check.sh)
RECHECK = 15           # s between localisation checks while relocate_failed
VENDOR_ERR = ('vendor chassis programs', 'VENDOR DOWN', 'job died')
KILLED = 'ABORT: VENDOR DOWN: the vendor chassis programs stopped (%s) - restarting them (vendor_watchdog)'


def read(path):
    with open(path) as fh:
        return fh.read()


def load(path):
    with open(path) as fh:
        return json.load(fh)


def save(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w') as fh:
        json.dump(data, fh)
    os.replace(tmp, path)


def sh(cmd, timeout=30):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout, cwd=HERE)
        return r.returncode, r.stdout.strip()
    except subprocess.TimeoutExpired:
        return 124, ''


class Watchdog:
    def __init__(self, run=sh, now=time.time, sleep=time.sleep, state_file=STATE_FILE):
        self.run, self.now, self.sleep, self.state_file = run, now, sleep, state_file
        try:
            self.s = load(state_file)
        except (OSError, ValueError):
            self.s = {}
        boot = self.boot_id()
        if self.s.get('boot') != boot:                 # a new boot: nothing verified, no restarts yet
            self.s = dict(boot=boot, state='booting', ok_since=0, restarts=[], handled_t=0, event='boot')
        self.misses, self.last_recheck = 0, 0.0

    # ------------------------------------------------------------------ facts
    def boot_id(self):
        try:
            return read('/proc/sys/kernel/random/boot_id').strip()
        except OSError:
            return 'unknown'

    def uptime(self):
        try:
            return float(read('/proc/uptime').split()[0])
        except (OSError, ValueError):
            return 1e9

    def missing(self):
        """What is missing of the vendor programs ([] = all running)."""
        rc, active = self.run('systemctl is-active %s' % doctor.SERVICE, 10)
        rc, comms = self.run('ps -eo comm', 10)
        gone = [p for p in doctor.VENDOR if p not in set(comms.split())]
        return ([] if active == 'active' else ['%s %s' % (doctor.SERVICE, active or 'unknown')]) + gone

    def job(self):
        try:
            j = load(JS)
        except (OSError, ValueError):
            return {'state': 'idle'}
        if j.get('state') == 'running':
            try:
                os.kill(int(read(PIDF).strip()), 0)
            except (OSError, ValueError):
                return dict(j, state='failed', error='job died', t=int(os.path.getmtime(JS)))
        return j

    def data_ok(self):
        """5 s of the three vendor topics (doctor.gather_ros, in its own process)."""
        rc, out = self.run('timeout 40 python3 -c "import doctor, json; print(json.dumps(doctor.gather_ros()))"', 50)
        try:
            f = json.loads(out.splitlines()[-1])
        except (IndexError, ValueError):
            return False
        return all(f.get(k, 0) > 0 for k in ('lidar', 'state', 'pose'))

    def match(self):
        rc, out = self.run('timeout 60 python3 locmatch.py check', 70)
        try:
            return float(json.loads(out.splitlines()[-1])['match'])
        except (IndexError, ValueError, KeyError, TypeError):
            return None

    # ------------------------------------------------------------------ state
    def log(self, text):
        line = '%s %s' % (time.strftime('%F %T'), text)
        print(line, flush=True)
        try:
            with open(LOG_FILE, 'a') as fh:
                fh.write(line + '\n')
        except OSError:
            pass

    def set(self, state, event=None, **kw):
        changed = state != self.s.get('state') or (event and event != self.s.get('event'))
        self.s.update(kw, state=state, t=int(self.now()), **({'event': event} if event else {}))
        if changed:
            self.log('%s: %s' % (state, self.s.get('event')))
        save(self.state_file, self.s)

    def ok(self, m, event):
        self.set('ok', event, ok_since=int(self.now()), match=m)

    # ------------------------------------------------------------------ actions
    def kill_job(self, job, why):
        """The chassis cannot move without the vendor driver: end the job, record why (the Orin resumes it)."""
        try:
            pid = int(read(PIDF).strip())
            os.killpg(pid, signal.SIGTERM)                  # job_wrap.py runs in its own session (setsid)
        except (OSError, ValueError):
            pass
        save(JS, {'state': 'failed', 'job': job.get('job'), 'error': KILLED % why, 't': int(self.now())})
        try:
            os.remove(PIDF)
        except OSError:
            pass
        self.log('stopped job %s: %s' % (job.get('job'), why))

    def relocate(self):
        """At T, at D (the robot usually stands at one of them), else over the whole map; the match after it."""
        try:
            st = load(ST_FILE)
        except (OSError, ValueError):
            st = {}
        for name in ('T', 'D'):
            pose = st.get(name, {}).get('pose')
            if pose:
                self.run('timeout 90 python3 locmatch.py at %.3f %.3f %.2f --apply' % tuple(pose[:3]), 100)
                m = self.match()
                if m is not None and m >= doctor.MATCH_OK:
                    return m, 'at %s' % name
        self.run('timeout 400 python3 locmatch.py global --apply', 410)
        return self.match(), 'whole map'

    def verify(self, why):
        """Programs up: data flowing and the pose matching the map (relocalise if not).  False: no data."""
        if not self.data_ok():
            return False
        m = self.match()
        if m is not None and m >= doctor.MATCH_OK:
            self.ok(m, 'checked (%s): data and localisation OK, match %.2f' % (why, m))
            return True
        self.set('relocating', 'localisation off (match %s): relocalising' % m)
        m, how = self.relocate()
        if m is not None and m >= doctor.MATCH_OK:
            self.ok(m, 'relocalised %s, match %.2f' % (how, m))
        else:
            self.set('relocate_failed', 'relocalisation failed (match %s): gento relocate, or on the web page' % m,
                     match=m)
        return True

    def recover(self, why):
        now = self.now()
        recent = [r for r in self.s.get('restarts', []) if now - r < RESTART_WINDOW]
        if len(recent) >= MAX_RESTARTS:
            self.set('gave_up', '%d restarts in %d min did not help (%s): power-cycle the chassis'
                     % (len(recent), RESTART_WINDOW // 60, why))
            return
        self.s['restarts'] = recent + [int(now)]
        self.set('restarting', 'restarting %s: %s' % (doctor.SERVICE, why))
        self.run('sudo -n systemctl restart %s' % doctor.SERVICE, 60)
        t = self.now()
        while self.now() - t < UP_TIMEOUT:
            self.sleep(PERIOD)
            if not self.missing() and self.data_ok():
                break
        else:
            self.set('down', 'restarted, but no data after %d s (%s)' % (UP_TIMEOUT, ', '.join(self.missing()) or 'no topics'))
            return
        self.run('timeout 30 ros2 service call /server/CancelAllTask server/srv/CancelAllTask "{}"', 40)
        self.sleep(SETTLE)
        self.verify('after the restart')

    def step(self):
        if self.uptime() < BOOT_GRACE:
            self.set('booting', 'waiting for the vendor start-up (%.0f s after boot)' % self.uptime())
            return
        job, gone = self.job(), self.missing()
        self.misses = self.misses + 1 if gone else 0
        if gone and self.misses >= DEBOUNCE:
            if job.get('state') == 'running':
                self.kill_job(job, ', '.join(gone))
            self.recover('missing: %s' % ', '.join(gone))
            self.misses = 0
            return
        booting = self.run('systemctl is-active gento-boot-check', 10)[1] == 'activating'
        if booting:                                     # boot_check.sh relocalises after a boot: let it finish
            self.set('booting', 'boot check still running')
            return
        if gone:
            return                                      # not yet DEBOUNCE checks in a row
        running = job.get('state') == 'running'
        err_t = int(job.get('t') or 0)
        if (job.get('state') == 'failed' and any(e in str(job.get('error')) for e in VENDOR_ERR)
                and err_t > self.s.get('handled_t', 0)):
            self.s['handled_t'] = err_t                 # a job saw no vendor data: check it, restart if confirmed
            if not self.verify('job %s: %s' % (job.get('job'), str(job.get('error'))[:80])):
                self.recover('no vendor data (job %s)' % job.get('job'))
            return
        if running:                                     # no checks that relocalise while the robot may move
            self.set(self.s.get('state', 'booting'))    # heartbeat
            return
        if self.s.get('state') in ('booting', 'down') or not self.s.get('ok_since'):
            if not self.verify('first check after boot'):
                self.recover('no vendor data')
            return
        if self.s.get('state') == 'relocate_failed' and self.now() - self.last_recheck >= RECHECK:
            self.last_recheck = self.now()
            m = self.match()
            if m is not None and m >= doctor.MATCH_OK:
                self.ok(m, 'localisation fixed by hand, match %.2f' % m)
            return
        self.set(self.s.get('state', 'ok'))             # heartbeat

    def loop(self):
        self.log('vendor watchdog started')
        while True:
            try:
                self.step()
            except Exception as e:                      # never die: log and go on
                self.log('error: %r' % e)
            self.sleep(PERIOD)


if __name__ == '__main__':
    Watchdog().loop()
