#!/usr/bin/env python3
"""arm_check.py [--config config/default.json] - NO MOTION: the arm controller before a run (part of gento doctor).

Connects like the task does (link, SDK / controller version, axis counts), reads the feedback once and disconnects:
nothing is enabled, uploaded, moved or reset.  Judges with the same rules as the task start:
  - no axis reports an error (state err), none is running a trajectory
  - the lift within fixed_lift_tolerance_mm of the taught heights (ready_lift .. receive_lift) - otherwise the task
    refuses to start ("lift 偏离当前阶段允许高度")
  - lift / waist / head never initialised by the controller (all 0, no error, not in position mode; after some
    power-ups): a warning only - the task runs on the arms and treats them as fixed (offline_axes.py)
  - body / head at their taught pose
  - the arms in the carry (receive) pose, an older carry pose (wrists only), or the ready / transition pose
Prints one line per check ("  [ OK ] ...", "  [警告] ...", "  [失败] ..."); exit 2 when something fails.
"""
import argparse
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

WRIST_ADJUST_DEG = 10.0        # as run_task_with_navigation.arm_start
LINK_RELEASE_S = 2.0           # the controller refuses a new link for ~1 s after one was closed


def arm_pose(joints, config):
    """'carry' | 'carry_wrist' | 'prepare' | None - the classification of run_task_with_navigation.arm_start."""
    def near(pose, tol):
        return all(abs(a - b) <= tol for arm, target in zip(joints, pose) for a, b in zip(arm, target))
    if near(config.receive, 1.0):
        return "carry"
    if all(abs(a - b) <= (WRIST_ADJUST_DEG if j >= 5 else 1.0)
           for arm, target in zip(joints, config.receive) for j, (a, b) in enumerate(zip(arm, target))):
        return "carry_wrist"
    if near(config.ready, 5.0) or near(config.transition, 5.0):
        return "prepare"
    return None


def evaluate(f, config):
    """f: facts read from the controller; [(level, text, fix)]."""
    out = []
    add = lambda level, text, fix='': out.append((level, text, fix))
    clear = '在示教器 / 厂家界面上查明并清除报错后再体检'
    off = [n for n in ('lift', 'body', 'head') if n in f.get('offline', ())]
    if off:
        add('warn', '%s没有上线（读数为 0，控制器开机时没有初始化这些驱动器）：任务会按固定的示教值运行（升降 %.0f mm），'
            '不使用、不移动它们' % ('、'.join({'lift': '升降', 'body': '腰部', 'head': '头部'}[n] for n in off),
                                 config.receive_lift[0]),
            '升降确实固定不动就可以开始；想恢复读数：松开所有急停后整机断电重启，等 1~2 分钟')
    bad = ['%s(err=%s, mode=%s)' % (n, s.get('err'), s.get('cur')) for n, s in f['states'].items()
           if s.get('err') != 0 or s.get('cur') not in (0, 1)]
    if bad:
        add('fail', '手臂控制器报告异常：%s' % '、'.join(bad), clear)
    else:
        add('ok', '手臂控制器连接正常，%d 个控制对象无报错' % len(f['states']))
    lift, tol = f['lift'], config.fixed_lift_tolerance_mm
    lo, hi = sorted((config.ready_lift[0], config.receive_lift[0]))
    span = '%.0f mm' % lo if lo == hi else '%.0f~%.0f mm' % (lo, hi)
    if 'lift' in off:
        pass
    elif f.get('lift_moving'):
        add('warn', '升降正在运动（%.1f mm）' % lift, '等它停下再体检')
    elif lo - tol <= lift <= hi + tol:
        add('ok', '升降高度 %.1f mm（任务要求 %s，允许 ±%.0f mm）' % (lift, span, tol))
    else:
        add('fail', '升降高度 %.1f mm，不在任务要求的 %s（±%.0f mm）内：开始任务会被拒绝' % (lift, span, tol),
            '先把升降调回 %s（厂家网页“立柱升降控制”或示教器）' % span)
    for name in ('body', 'head'):
        if name in off:
            continue
        actual, taught = f[name], getattr(config, name)
        if any(abs(a - b) > config.fixed_angle_tolerance_deg for a, b in zip(actual, taught)):
            add('fail', '%s 不在示教姿态（%s，应为 %s）' % (name, [round(v, 2) for v in actual], taught),
                '先把 %s 调回示教姿态' % name)
    if any(s >= 3 for s in f['traj_states']):
        add('warn', '双臂有轨迹正在执行', '等它结束；没有任务在跑却一直如此，就在示教器上停止')
    pose = arm_pose(f['joints'], config)
    if pose == 'carry':
        add('ok', '双臂在运输（接盘）位：开始任务时双臂不动')
    elif pose == 'carry_wrist':
        add('ok', '双臂在旧的运输位：开始任务时只转动腕部')
    elif pose == 'prepare':
        add('warn', '双臂在准备位/过渡位：开始任务时会先移到运输位', '确认双臂周围没有人和东西')
    else:
        add('fail', '双臂既不在运输位，也不在准备位或过渡位：任务会拒绝开始（%s）'
            % [[round(v, 1) for v in arm] for arm in f['joints']],
            '先经过渡位把双臂移回准备位')
    return out


def read_facts(config):
    from skye_task.sdk import GentoSDK
    from skye_task.sequence import FIXED_AXES
    from offline_axes import is_offline
    sdk = GentoSDK(config)
    try:
        meta = sdk.connect()
        fb = sdk.read()
    finally:
        sdk.close()
        time.sleep(LINK_RELEASE_S)
    states = {'左臂': fb.rt['arms'][0]['state'], '右臂': fb.rt['arms'][1]['state']}
    states.update({name: fb.rt[name]['state'] for name in FIXED_AXES})
    return dict(meta=meta, states=states, lift=fb.rt['lift']['fb_pos'][0],
                offline=[n for n in ('lift', 'body', 'head') if is_offline(fb.rt[n])],
                lift_moving=fb.sg['lift']['get']['traj_state'] >= 3,
                body=fb.rt['body']['fb_pos'][:FIXED_AXES['body']], head=fb.rt['head']['fb_pos'][:FIXED_AXES['head']],
                joints=fb.joints, traj_states=fb.trajectory_states)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--config', default=str(ROOT / 'config' / 'default.json'))
    a = p.parse_args(argv)
    from skye_task.config import load_config
    mark = {'ok': '[ OK ]', 'warn': '[警告]', 'fail': '[失败]'}
    try:
        config = load_config(a.config)
    except Exception as e:
        print('  [失败] 任务配置 %s 无效：%s  ->  gento task-check 看详细信息' % (a.config, e))
        return 2
    try:
        res = evaluate(read_facts(config), config)
    except Exception as e:
        print('  [失败] 连不上手臂控制器或读不到反馈：%s  ->  手臂控制器是否上电、网线是否插好（%s）'
              % (e, config.controller_ip))
        return 2
    for level, text, fix in res:
        print('  %s %s%s' % (mark[level], text, ('  ->  ' + fix) if fix and level != 'ok' else ''))
    return 2 if any(level == 'fail' for level, _, _ in res) else 0


if __name__ == '__main__':
    sys.exit(main())
