#!/usr/bin/env python3
"""latency_report.py [LOG] [--jobs N] : planning-latency report from the docking log (run.log).

Requirement: from data input (lidar snapshot, settled localisation) to planning done / motion start < 1.2 s.
dock.py writes timed events (wall clock, ms) for every command it issues:
    LIDAR scan start -> INPUT ready -> PLAN start -> PLAN done (input -> plan) -> COMMAND accepted -> MOTION started
This lists, per job, every command with those times and marks the START-UP: the first command of the job (e.g.
leaving T for D right after the task is given).  Standard library only.
"""
import re
import sys

LIMIT_MS = 1200
LINE = re.compile(r'^\[(\d\d:\d\d:\d\d\.\d{3})\s+[\d.]+s\]\s*(.*)$')
HEAD = re.compile(r'^===== (\d\d:\d\d:\d\d) (.*)$')
EV = [('scan', re.compile(r'^LIDAR scan start')),
      ('input', re.compile(r'^INPUT ready: (.*)$')),
      ('pstart', re.compile(r'^PLAN start: (.*)$')),
      ('plan', re.compile(r'^PLAN done: (.*) \| input -> plan (\d+) ms')),
      ('cmd', re.compile(r'^COMMAND accepted: (.*) \| input -> command (\d+) ms')),
      ('motion', re.compile(r'^MOTION started: (.*) \| input -> motion (\d+) ms'))]


def parse(lines):
    """[{'head', 'time', 'cmds': [{step, scan, input, input_what, plan_start, plan, plan_ms, cmd_ms, motion, motion_ms}]}]"""
    jobs = []; job = None; scan = inp = what = pstart = None
    for raw in lines:
        raw = raw.rstrip('\n')
        h = HEAD.match(raw)
        if h:
            job = dict(time=h.group(1), head=h.group(2), cmds=[], end=None); jobs.append(job)
            scan = inp = what = pstart = None
            continue
        m = LINE.match(raw)
        if not m or job is None:
            if job is not None and ('ABORT' in raw or raw.startswith('FINAL')):
                job['end'] = raw.strip()
            continue
        t, body = m.groups()
        if body.startswith(('FINAL', 'STAGE-ONLY')) or 'ABORT' in body:
            job['end'] = body.strip()
        for kind, rx in EV:
            e = rx.match(body)
            if not e:
                continue
            if kind == 'scan':
                scan = t
            elif kind == 'input':
                inp, what, pstart = t, e.group(1), None
                if not what.startswith('lidar'):
                    scan = None
            elif kind == 'pstart':
                pstart = pstart or t
            elif kind == 'plan':
                job['cmds'].append(dict(step=e.group(1), scan=scan, input=inp, input_what=what, plan_start=pstart,
                                        plan=t, plan_ms=int(e.group(2)), cmd_ms=None, motion=None, motion_ms=None))
                scan = inp = what = pstart = None
            elif kind == 'cmd' and job['cmds'] and job['cmds'][-1]['step'] == e.group(1):
                job['cmds'][-1].update(cmd=t, cmd_ms=int(e.group(2)))
            elif kind == 'motion':
                for c in reversed(job['cmds']):
                    if e.group(1).startswith(c['step']) and c['motion'] is None:
                        c.update(motion=t, motion_ms=int(e.group(2)))
                        break
            break
    return jobs


def verdict(ms):
    return '-' if ms is None else ('OK' if ms < LIMIT_MS else 'OVER')


def report(jobs, out=sys.stdout):
    w = out.write
    for job in jobs:
        if not job['cmds']:
            continue
        w('===== %s %s\n' % (job['time'], job['head']))
        w('  %-3s %-34s %-12s %-12s %-12s %-12s %8s %8s %8s\n' % ('#', 'command', 'scan start', 'input ready',
                                                                   'plan start', 'plan done', 'in>plan', 'in>cmd', 'in>move'))
        for i, c in enumerate(job['cmds']):
            w('  %-3s %-34s %-12s %-12s %-12s %-12s %8s %8s %8s\n' % (
                ('%d*' if i == 0 else '%d') % (i + 1), c['step'][:34], c['scan'] or '-', c['input'] or '-',
                c['plan_start'] or '-', c['plan'], '%d' % c['plan_ms'],
                '-' if c['cmd_ms'] is None else '%d' % c['cmd_ms'], '-' if c['motion_ms'] is None else '%d' % c['motion_ms']))
        s = job['cmds'][0]
        w('  START-UP (* first command): input (%s) -> plan done %d ms [%s]; -> motion started %s ms [%s]\n' % (
            s['input_what'] or '?', s['plan_ms'], verdict(s['plan_ms']),
            '-' if s['motion_ms'] is None else s['motion_ms'], verdict(s['motion_ms'])))
        worst = max(job['cmds'], key=lambda c: c['plan_ms'])
        w('  all %d commands: input -> plan max %d ms (%s) [%s]%s\n' % (
            len(job['cmds']), worst['plan_ms'], worst['step'], verdict(worst['plan_ms']),
            '' if not job['end'] else '; end: ' + job['end'][:90]))


def main(argv):
    path = 'run.log'; n = 5
    a = list(argv)
    if '--jobs' in a:
        i = a.index('--jobs'); n = int(a[i + 1]); del a[i:i + 2]
    if a:
        path = a[0]
    with open(path, encoding='utf-8', errors='replace') as f:
        jobs = [j for j in parse(f) if j['cmds']]
    if not jobs:
        print('no timed commands in %s (jobs from before the timing events have none)' % path)
        return 1
    report(jobs[-n:])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
