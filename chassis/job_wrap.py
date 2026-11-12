#!/usr/bin/env python3
"""job_wrap.py JOB -- CMD ARGS...   run a motion job, append its output to run.log and keep job_status.json
up to date:  {"state": "running"|"done"|"failed", "job": JOB, "final": <FINAL line>, "error": <ABORT line>}.
"done" means the job exited 0 AND reported where it stopped (FINAL / STAGE-ONLY)."""
import json, os, re, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
LOG, JS = os.path.join(HERE, 'run.log'), os.path.join(HERE, 'job_status.json')


def status(**kw):
    tmp = JS + '.tmp'
    json.dump(dict(kw, t=int(time.time())), open(tmp, 'w')); os.replace(tmp, JS)


job = sys.argv[1]; cmd = sys.argv[sys.argv.index('--') + 1:]
status(state='running', job=job, pid=os.getpid())
final = error = None
with open(LOG, 'a') as log:
    log.write('===== %s %s\n' % (time.strftime('%H:%M:%S'), ' '.join(cmd))); log.flush()
    p = subprocess.Popen(cmd, cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    for line in p.stdout:
        log.write(line); log.flush()
        body = re.sub(r'^\[\s*[\d.]+s\]\s*', '', line)      # dock.py log lines carry an elapsed-time prefix
        if body.startswith(('FINAL', 'STAGE-ONLY')):
            final = body.strip()
        elif 'ABORT' in line:
            error = line.strip()
    rc = p.wait()
status(state='done' if rc == 0 and final else 'failed', job=job, exit=rc, final=final,
       error=error or (None if rc == 0 else 'exit code %d' % rc))
