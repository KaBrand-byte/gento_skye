"""Response-time metric of one task run, from its log (Orin output + the chassis run.log lines of the same run).

Metric (evaluation): the FIRST environment input of the run - the first lidar point cloud snapshot handed to the
planner ("INPUT ready: lidar snapshot") - to the FIRST chassis motion command issued after it ("COMMAND
accepted").  Required < 1200 ms.  Both system timestamps (the chassis clock, where input and command happen) and
the interval are reported; every job of the run is listed too.

    python3 response_summary.py RUN.log [--offset SECONDS] [--csv runs/response_times.csv] [--name NAME]

Prints the summary block (put at the top of the run log) and appends one line per run to the CSV (never
overwritten).  --offset: Orin clock minus chassis clock (s), to show the Orin time as well.
New dock.py logs carry "RESPONSE: ..." lines with epoch timestamps; older ones are reconstructed from the
"INPUT ready" / "COMMAND accepted" lines (time of day only).
"""
import argparse
import os
import re
import sys
import time

LIMIT_MS = 1200
JOB = re.compile(r'^===== (\d\d:\d\d:\d\d) .*dock\.py (\S+(?: [A-Z0-9]+)?)')
RESP = re.compile(r'RESPONSE: first lidar input ([\d.]+) \(([^)]*)\) -> first chassis command ([\d.]+) \(([^)]*)\) '
                  r'= (\d+) ms \(limit \d+ ms\) (OK|OVER) \| command: (.*?) \| from the scan start (\S+) ms')
LINE = re.compile(r'^\[(\d\d):(\d\d):(\d\d)\.(\d{3})\s+[\d.]+s\]\s+(.*)$')


def parse(lines):
    jobs, cur = [], None
    for raw in lines:
        raw = raw.rstrip('\n')
        m = JOB.match(raw)
        if m:
            cur = dict(job=m.group(2), scan=None, input=None, cmd=None, resp=None); jobs.append(cur)
            continue
        if cur is None:
            continue
        r = RESP.search(raw)
        if r:
            cur['resp'] = dict(t_in=float(r.group(1)), in_s=r.group(2), t_cmd=float(r.group(3)), cmd_s=r.group(4),
                               ms=float(r.group(5)), ok=r.group(6), command=r.group(7), scan_ms=r.group(8))
            continue
        m = LINE.match(raw)
        if not m:
            continue
        sec = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3)) + int(m.group(4)) / 1000.0
        text = m.group(5)
        clock = '%s:%s:%s.%s' % m.group(1, 2, 3, 4)
        if text.startswith('LIDAR scan start') and cur['input'] is None:
            cur['scan'] = (sec, clock)
        elif text.startswith('INPUT ready: lidar snapshot') and cur['input'] is None:
            cur['input'] = (sec, clock)
        elif text.startswith('COMMAND accepted') and cur['input'] is not None and cur['cmd'] is None:
            cur['cmd'] = (sec, clock, text.split('|')[0].replace('COMMAND accepted:', '').strip())
    out = []
    for j in jobs:
        if j['resp']:
            r = j['resp']
            out.append(dict(job=j['job'], ms=r['ms'], in_s=r['in_s'], cmd_s=r['cmd_s'], t_in=r['t_in'], t_cmd=r['t_cmd'],
                            command=r['command'], scan_ms=r['scan_ms'], exact=True))
        elif j['input'] and j['cmd']:
            ms = (j['cmd'][0] - j['input'][0]) % 86400 * 1000.0
            scan_ms = '%.0f' % ((j['cmd'][0] - j['scan'][0]) % 86400 * 1000.0) if j['scan'] else 'nan'
            out.append(dict(job=j['job'], ms=ms, in_s=j['input'][1], cmd_s=j['cmd'][1], t_in=None, t_cmd=None,
                            command=j['cmd'][2], scan_ms=scan_ms, exact=False))
    return out


def verdict(ms):
    return '合格' if ms < LIMIT_MS else '不合格（超过 %d ms）' % LIMIT_MS


def orin_time(t, offset):
    if t is None or offset is None:
        return ''
    u = t + offset
    return '（Orin 时间 %s.%03d）' % (time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(u)), int((u % 1) * 1000))


def summary(res, offset=None, name=''):
    bar = '#' * 78
    out = [bar, '响应时间指标（本次运行）%s' % (('  ' + name) if name else ''),
           '定义：环境信息首次输入（首帧激光点云快照交给规划）→ 首次下发底盘运动指令；要求 < %d ms' % LIMIT_MS]
    if not res:
        out += ['未找到底盘日志中的点云输入 / 运动指令，无法统计（底盘日志未取到，或本次没有底盘运动）。', bar]
        return '\n'.join(out)
    f = res[0]
    out += ['首次点云输入      ：底盘系统时间 %s%s%s' % (f['in_s'], ('  epoch %.3f' % f['t_in']) if f['t_in'] else '', orin_time(f['t_in'], offset)),
            '首次底盘运动指令  ：底盘系统时间 %s%s%s' % (f['cmd_s'], ('  epoch %.3f' % f['t_cmd']) if f['t_cmd'] else '', orin_time(f['t_cmd'], offset)),
            '时间间隔          ：%.0f ms  →  %s' % (f['ms'], verdict(f['ms'])),
            '（首条指令：%s；从开始采集点云算起 %s ms；所属底盘作业：%s）' % (f['command'], f['scan_ms'], f['job'])]
    if not f['exact']:
        out.append('（旧版底盘日志：时间戳只有时分秒，由 INPUT ready / COMMAND accepted 两行推算）')
    out.append('本次各底盘作业（每个作业的首帧点云 → 首条运动指令）：')
    for r in res:
        out.append('  %-10s %6.0f ms  %s   %s → %s   %s' % (r['job'], r['ms'], '合格' if r['ms'] < LIMIT_MS else '超时',
                                                         r['in_s'], r['cmd_s'], r['command']))
    out.append('最大值 %.0f ms；全部 %s' % (max(r['ms'] for r in res),
                                       '合格' if all(r['ms'] < LIMIT_MS for r in res) else '有超时'))
    out.append(bar)
    return '\n'.join(out)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('log')
    p.add_argument('--offset', type=float)
    p.add_argument('--csv')
    p.add_argument('--name', default='')
    a = p.parse_args(argv)
    with open(a.log, encoding='utf-8', errors='replace') as fh:
        res = parse(fh)
    print(summary(res, a.offset, a.name))
    if a.csv and res:
        f = res[0]
        new = not os.path.exists(a.csv)
        with open(a.csv, 'a', encoding='utf-8') as fh:
            if new:
                fh.write('run,first_input_chassis_time,first_input_epoch,first_command_chassis_time,first_command_epoch,'
                         'ms,result,first_command,max_ms_all_jobs\n')
            fh.write('%s,%s,%s,%s,%s,%.0f,%s,%s,%.0f\n' % (
                a.name or os.path.basename(a.log), f['in_s'], '%.3f' % f['t_in'] if f['t_in'] else '', f['cmd_s'],
                '%.3f' % f['t_cmd'] if f['t_cmd'] else '', f['ms'], 'OK' if f['ms'] < LIMIT_MS else 'OVER',
                f['command'].replace(',', ' '), max(r['ms'] for r in res)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
