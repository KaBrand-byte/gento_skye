#!/usr/bin/env python3
"""doctor.py - NO-MOTION health check of the chassis before a demo / between runs (run.sh doctor, gento doctor).

Separates the failure modes seen so far, each with what to do:
  - vendor programs not running after a boot (linkhou_robot.service, Restart=no; 2026-11-16 and 11-18): no lidar,
    no chassis state, no pose although ping / ssh work
  - eth1 (the link to the Orin) dropping out (2026-11-17: 39 times in 9 min, gmac TX FIFO hang)
  - localisation lost or wrong, hard switch on MANUAL, docking bridge not running, chassis overloaded
Prints one line per check, a verdict, and a last JSON line.  Exit 0: OK (warnings allowed), 2: do not start.
"""
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SERVICE = 'linkhou_robot'
VENDOR = ('driver_node', 'linkhou_locatio', 'controller_core', 'robot_server')   # process names (15 chars max)
LIDAR, STATE, POSE = '/driver/lidar/point_cloud/Data', 'move/State', 'slam/LocationData'
LISTEN = 5.0                      # s of listening to each topic
MATCH_OK, MATCH_MIN = 0.85, 0.60  # scan-to-map match: good / below this dock.py refuses to start
BOOT_SETTLE = 180                 # s: the vendor programs need ~2-3 min after power-on
RESTART = '整机断电重启（或在底盘上 sudo systemctl restart linkhou_robot），等 3 分钟后再 gento doctor'


def sh(cmd, timeout=10):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except Exception:
        return ''


def gather_system():
    f = {}
    try:
        f['uptime'] = float(open('/proc/uptime').read().split()[0])
        f['load'] = float(open('/proc/loadavg').read().split()[0])
    except (OSError, ValueError, IndexError):
        pass
    f['ncpu'] = os.cpu_count() or 4
    f['service'] = sh('systemctl is-active %s' % SERVICE) or 'unknown'
    f['bridge'] = sh('systemctl is-active gento-dock-bridge') or 'unknown'
    comms = set(sh('ps -eo comm').split())
    f['missing'] = [p for p in VENDOR if p not in comms]
    try:
        f['carrier_changes'] = int(open('/sys/class/net/eth1/carrier_changes').read())
    except (OSError, ValueError):
        pass
    dm = sh('dmesg 2>/dev/null', timeout=10)
    if dm:
        f['eth1_down'] = len(re.findall(r'eth1: Link is Down', dm))
        f['tx_hang'] = len(re.findall(r'gmac_check_tx_fifo', dm))
    try:
        f['job'] = json.load(open(os.path.join(HERE, 'job_status.json'))).get('state')
    except (OSError, ValueError):
        f['job'] = 'idle'
    f['watchdog_service'] = sh('systemctl is-active gento-vendor-watchdog') or 'unknown'
    try:
        f['watchdog'] = json.load(open(os.path.join(HERE, 'vendor_watchdog.json')))
    except (OSError, ValueError):
        f['watchdog'] = None
    return f


def gather_ros(seconds=LISTEN):
    """Messages per topic in `seconds`, plus the last chassis state and pose quality."""
    f = {'lidar': 0, 'state': 0, 'pose': 0}
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from move.msg import State
        from slam.msg import LocationData
    except Exception as e:                           # ROS environment missing: say so instead of failing
        f['ros_error'] = str(e)
        return f
    rclpy.init(); n = rclpy.create_node('gento_doctor'); last = {}

    def count(key):
        def cb(m):
            f[key] += 1; last[key] = m
        return cb
    n.create_subscription(PointCloud2, LIDAR, count('lidar'), qos_profile_sensor_data)
    n.create_subscription(State, STATE, count('state'), 10)
    n.create_subscription(LocationData, POSE, count('pose'), qos_profile_sensor_data)
    t = time.time()
    while time.time() - t < seconds:
        rclpy.spin_once(n, timeout_sec=0.05)
    if 'state' in last:
        s = last['state']; f['hard_auto'] = bool(s.is_hard_auto); f['soft_auto'] = bool(s.is_soft_auto)
    if 'pose' in last:
        m = last['pose']; f['conf'] = getattr(m, 'confidence', None); f['relocating'] = getattr(m, 'is_relocating', None)
    n.destroy_node(); rclpy.shutdown()
    return f


def gather_match():
    out = sh('timeout 60 python3 %s check' % os.path.join(HERE, 'locmatch.py'), timeout=70).splitlines()
    try:
        return json.loads(out[-1])
    except (IndexError, ValueError):
        return None


def evaluate(f):
    """[(level, text, fix)] with level ok | warn | fail, in the order to read them."""
    out = []
    add = lambda level, text, fix='': out.append((level, text, fix))
    up = f.get('uptime')
    if up is not None and up < BOOT_SETTLE:
        add('warn', '底盘开机才 %.0f s，厂家程序可能还在启动' % up, '等开机满 3 分钟再 gento doctor')
    if f.get('service') != 'active':
        add('fail', '厂家主服务 %s 没在运行（%s）' % (SERVICE, f.get('service')), RESTART)
    elif f.get('missing'):
        add('fail', '厂家进程缺失：%s' % '、'.join(f['missing']), RESTART)
    else:
        add('ok', '厂家主服务和进程在运行')
    if f.get('ros_error'):
        add('fail', 'ROS 环境不可用：%s' % f['ros_error'], '在底盘上用 run.sh doctor（会先加载 env.sh）')
    else:
        for key, name, low in (('lidar', '激光点云', 10), ('state', '底盘状态 move/State', 5), ('pose', '定位 slam/LocationData', 5)):
            k = f.get(key, 0)
            if k == 0:
                add('fail', '%s：%.0f s 内没收到任何数据' % (name, LISTEN), RESTART)
            elif k < low:
                add('warn', '%s：%.0f s 内只收到 %d 条（偏少）' % (name, LISTEN, k), '底盘可能过载；再体检一次')
            else:
                add('ok', '%s：%.0f s 内 %d 条' % (name, LISTEN, k))
    if f.get('hard_auto') is False:
        add('fail', '硬开关在 MANUAL', '把硬开关拨到 AUTO')
    elif f.get('hard_auto'):
        add('ok', '硬开关在 AUTO')
    if f.get('relocating'):
        add('warn', '厂家定位正在重定位', '等它结束，再 gento relocate')
    conf = f.get('conf')
    if conf is not None and conf < 60:
        add('warn', '定位置信度偏低（%s）' % conf, 'gento relocate')
    m = f.get('match')
    if m is None:
        if f.get('pose', 0) and f.get('lidar', 0):
            add('warn', '扫描与地图匹配没算出来', '再体检一次；不行就 gento relocate')
    elif m >= MATCH_OK:
        add('ok', '定位与地图吻合（匹配度 %.2f）' % m)
    elif m >= MATCH_MIN:
        add('warn', '定位与地图吻合一般（匹配度 %.2f）' % m, 'gento relocate')
    else:
        add('fail', '定位不对（匹配度 %.2f < %.2f，任务会拒绝出发）' % (m, MATCH_MIN), 'gento relocate；不行就在厂家网页上重定位')
    cc = f.get('carrier_changes')
    if cc is not None:
        drops = max(0, (cc - 1) // 2)                # 1 = the link coming up at boot; each drop adds down + up
        if drops:
            add('warn', '开机以来连 Orin 的网口 eth1 掉线 %d 次%s' % (
                drops, '（其中发送队列卡死 %d 次）' % f['tx_hang'] if f.get('tx_hang') else ''),
                '演示前整机断电重启一次；仍掉线就换网线/接口并报厂家')
        else:
            add('ok', '网口 eth1 开机以来没掉过')
    load, ncpu = f.get('load'), f.get('ncpu', 4)
    if load is not None and load > 3 * ncpu:
        add('warn', '底盘负载很高（1 分钟平均 %.1f，%d 核）' % (load, ncpu), '别在底盘上跑别的程序；必要时重启底盘')
    if f.get('bridge') != 'active':
        add('fail', '导航桥 gento-dock-bridge 没在运行（%s）' % f.get('bridge'),
            '开机 90 s 后它会自动起来；否则 sudo systemctl restart gento-dock-bridge')
    else:
        add('ok', '导航桥在运行')
    if f.get('job') == 'running':
        add('warn', '底盘上有作业正在运行', '等它结束，或 gento stop')
    w, ws = f.get('watchdog') or {}, f.get('watchdog_service')
    if ws is not None and ws != 'active':
        add('warn', '自动看护 gento-vendor-watchdog 没在运行（%s）：厂家程序停了不会自动恢复' % ws,
            '在底盘上 bash ~/gento_dock/install.sh 或 sudo systemctl restart gento-vendor-watchdog')
    elif w:
        st, ev = w.get('state'), w.get('event') or ''
        if st == 'gave_up':
            add('fail', '自动看护已放弃：%s' % ev, '整机断电重启')
        elif st == 'relocate_failed':
            add('fail', '厂家程序已自动重启，但自动重定位没成功', 'gento relocate；不行就在厂家网页上重定位')
        elif st in ('restarting', 'relocating', 'down'):
            add('warn', '自动看护正在恢复厂家程序：%s' % ev, '等 2～3 分钟再 gento doctor')
        else:
            add('ok', '自动看护在运行（%s）' % st)
        n = len(w.get('restarts') or [])
        if n:
            ago = (w.get('t') or 0) - w['restarts'][-1]
            add('warn', '本次开机以来自动重启过厂家程序 %d 次（最近一次约 %.0f 分钟前）' % (n, ago / 60.0),
                '看底盘 ~/gento_dock/watchdog.log；次数多就报厂家')
    return out


def main():
    f = gather_system()
    f.update(gather_ros())
    if f.get('lidar') and f.get('pose'):
        r = gather_match()
        if r:
            f['match'] = r.get('match')
    res = evaluate(f)
    mark = {'ok': '[ OK ]', 'warn': '[警告]', 'fail': '[失败]'}
    print('底盘体检  %s' % time.strftime('%F %T'))
    for level, text, fix in res:
        print('  %s %s%s' % (mark[level], text, ('  ->  ' + fix) if fix and level != 'ok' else ''))
    fails = [t for lv, t, _ in res if lv == 'fail']; warns = [t for lv, t, _ in res if lv == 'warn']
    print('结论：%s' % ('不能开始（%d 项失败）' % len(fails) if fails else
                      '可以开始（%d 项警告，看一下）' % len(warns) if warns else '一切正常，可以开始'))
    print(json.dumps(dict(ok=not fails, fails=fails, warns=warns,
                          facts={k: v for k, v in f.items() if k != 'missing'}), ensure_ascii=False))
    return 2 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
