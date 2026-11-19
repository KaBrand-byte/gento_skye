"""Vendor watchdog (vendor_watchdog.py) decisions, with the shell, the clock and the files faked."""
import json
import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import vendor_watchdog as VW  # noqa: E402

ALL = 'systemd\nbash\ndriver_node\nlinkhou_locatio\ncontroller_core\nrobot_server\n'
NO_DRIVER = 'systemd\nbash\nlinkhou_locatio\ncontroller_core\nrobot_server\n'


class Chassis:
    """What the shell commands answer; changes as the scenario goes on."""
    def __init__(self):
        self.procs, self.service, self.data, self.matches = ALL, 'active', True, [0.95]
        self.calls, self.boot_check = [], 'inactive'

    def run(self, cmd, timeout=30):
        self.calls.append(cmd)
        if cmd.startswith('systemctl is-active linkhou_robot'):
            return 0, self.service
        if cmd.startswith('systemctl is-active gento-boot-check'):
            return 0, self.boot_check
        if cmd.startswith('ps -eo comm'):
            return 0, self.procs
        if 'gather_ros' in cmd:
            n = 30 if self.data else 0
            return 0, json.dumps(dict(lidar=n, state=n, pose=n))
        if 'locmatch.py check' in cmd:
            m = self.matches.pop(0) if len(self.matches) > 1 else self.matches[0]
            return 0, json.dumps(dict(match=m))
        if 'systemctl restart linkhou_robot' in cmd:
            self.procs, self.service, self.data = ALL, 'active', True       # the restart brings them back
        return 0, ''


class WatchdogTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.ch = Chassis(); self.clock = [10000.0]
        files = {'JS': 'job_status.json', 'PIDF': 'run.pid', 'ST_FILE': 'stations.json', 'LOG_FILE': 'watchdog.log'}
        for k, f in files.items():
            p = mock.patch.object(VW, k, os.path.join(self.tmp, f)); p.start(); self.addCleanup(p.stop)
        json.dump({'T': {'pose': [1.307, 2.39, -94.75]}, 'D': {'pose': [2.557, -0.799, 173.9]}},
                  open(os.path.join(self.tmp, 'stations.json'), 'w'))
        self.job({'state': 'done', 'job': 'T', 't': 9000})
        p = mock.patch.object(VW.os, 'killpg', lambda pid, sig: self.ch.calls.append('killpg %d' % pid), create=True); p.start(); self.addCleanup(p.stop)
        self.wd = self.make()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def make(self, uptime=900.0):
        def sleep(s):
            self.clock[0] += s
        wd = VW.Watchdog(run=self.ch.run, now=lambda: self.clock[0], sleep=sleep,
                         state_file=os.path.join(self.tmp, 'vendor_watchdog.json'))
        wd.uptime = lambda: uptime
        wd.boot_id = lambda: 'boot-1'
        return wd

    def job(self, j, pid=None):
        json.dump(j, open(os.path.join(self.tmp, 'job_status.json'), 'w'))
        if pid:
            open(os.path.join(self.tmp, 'run.pid'), 'w').write(str(pid))

    def step(self, n=1):
        for _ in range(n):
            self.wd.step(); self.clock[0] += VW.PERIOD
        return self.wd.s

    def restarts(self):
        return [c for c in self.ch.calls if 'systemctl restart' in c]

    def test_first_check_after_boot_sets_ok(self):
        s = self.step()
        self.assertEqual(s['state'], 'ok'); self.assertGreater(s['ok_since'], 0); self.assertEqual(self.restarts(), [])

    def test_waits_during_the_boot_grace_and_the_boot_check(self):
        self.wd = self.make(uptime=60.0)
        self.assertEqual(self.step()['state'], 'booting')
        self.wd = self.make(); self.ch.boot_check = 'activating'
        self.assertEqual(self.step()['state'], 'booting')

    def test_programs_gone_while_a_job_runs_job_stopped_restarted_relocalised(self):
        self.step()
        self.job({'state': 'running', 'job': 'D', 't': 10010}, pid=os.getpid())
        self.ch.procs = NO_DRIVER
        self.step(2); self.assertEqual(self.restarts(), [])                 # debounce: not on the first misses
        self.ch.matches = [0.40, 0.95]                                      # pose lost: relocalised at T
        s = self.step()
        self.assertEqual(len(self.restarts()), 1)
        self.assertIn('killpg %d' % os.getpid(), self.ch.calls)
        job = json.load(open(os.path.join(self.tmp, 'job_status.json')))
        self.assertEqual(job['state'], 'failed'); self.assertIn('VENDOR DOWN', job['error']); self.assertEqual(job['job'], 'D')
        self.assertTrue(any('CancelAllTask' in c for c in self.ch.calls))
        self.assertTrue(any('locmatch.py at 1.307 2.390' in c for c in self.ch.calls))
        self.assertEqual(s['state'], 'ok'); self.assertGreaterEqual(s['ok_since'], job['t'])   # the Orin may go on

    def test_job_without_vendor_data_checked_then_restarted(self):
        self.step()
        self.ch.data = False
        self.job({'state': 'failed', 'job': 'T', 'error': 'ABORT: no lidar point cloud - the vendor chassis programs '
                  'are not running', 't': int(self.clock[0])})
        s = self.step()
        self.assertEqual(len(self.restarts()), 1); self.assertEqual(s['state'], 'ok')
        self.step(); self.assertEqual(len(self.restarts()), 1)              # the same failure is handled once

    def test_job_failure_with_data_flowing_needs_no_restart(self):
        self.step(); before = self.wd.s['ok_since']
        self.job({'state': 'failed', 'job': 'D', 'error': 'job died', 't': int(self.clock[0])})
        s = self.step()
        self.assertEqual(self.restarts(), []); self.assertEqual(s['state'], 'ok'); self.assertGreater(s['ok_since'], before)

    def test_relocalisation_failing_is_reported_and_rechecked(self):
        self.step()
        self.ch.data = False
        self.job({'state': 'failed', 'job': 'D', 'error': 'VENDOR DOWN', 't': int(self.clock[0])})
        self.ch.matches = [0.3]
        s = self.step()
        self.assertEqual(s['state'], 'relocate_failed')
        self.ch.matches = [0.93]                                            # fixed by hand (gento relocate)
        self.clock[0] += VW.RECHECK
        self.assertEqual(self.step()['state'], 'ok')

    def test_gives_up_after_too_many_restarts(self):
        self.step()
        for _ in range(VW.MAX_RESTARTS):
            self.ch.procs = NO_DRIVER
            self.step(VW.DEBOUNCE)
        self.assertEqual(len(self.restarts()), VW.MAX_RESTARTS)
        self.ch.procs = NO_DRIVER
        s = self.step(VW.DEBOUNCE)
        self.assertEqual(s['state'], 'gave_up'); self.assertEqual(len(self.restarts()), VW.MAX_RESTARTS)

    def test_service_never_started_after_boot_is_started(self):
        self.ch.service, self.ch.procs = 'inactive', 'systemd\nbash\n'      # 2026-11-16 / 11-18
        s = self.step(VW.DEBOUNCE)
        self.assertEqual(len(self.restarts()), 1); self.assertEqual(s['state'], 'ok')


if __name__ == '__main__':
    unittest.main()
