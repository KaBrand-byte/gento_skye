#!/usr/bin/env python3
"""over_map.py ZLO ZHI : accumulate lidar frames, map frame, print 10 cm cells with points in the height band"""
import sys, time, math, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from slam.msg import LocationData
zlo, zhi = float(sys.argv[1]), float(sys.argv[2])
rclpy.init(); n = Node('claude_overmap'); C = []; L = [None]
n.create_subscription(PointCloud2, '/driver/lidar/point_cloud/Data', lambda m: C.append(m) if len(C) < 15 else None, qos_profile_sensor_data)
n.create_subscription(LocationData, 'slam/LocationData', lambda m: L.__setitem__(0, m), qos_profile_sensor_data)
t = time.time()
while (len(C) < 15 or L[0] is None) and time.time() - t < 8:
    rclpy.spin_once(n, timeout_sec=0.05)
p = L[0].pose; X, Y, T = p.position.x, p.position.y, p.orientation.z
a = np.concatenate([np.stack([s['x'], s['y'], s['z']], 1).astype(float) for s in (pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True) for m in C)])
a = a[np.isfinite(a).all(1)]; a = a[(a[:, 2] > zlo) & (a[:, 2] < zhi) & (np.hypot(a[:, 0], a[:, 1]) > 0.75)]
c, s = math.cos(T), math.sin(T); mx = X + c * a[:, 0] - s * a[:, 1]; my = Y + s * a[:, 0] + c * a[:, 1]
print('pose %.3f %.3f %.1f  pts %d' % (X, Y, math.degrees(T), len(a)))
k = np.stack([np.floor(mx / 0.1), np.floor(my / 0.1)], 1).astype(int)
u, inv, cnt = np.unique(k, axis=0, return_inverse=True, return_counts=True)
for idx in np.argsort(u[:, 1] * 1000 + u[:, 0]):
    if cnt[idx] >= 4:
        z = a[inv.ravel() == idx, 2]
        print('x %5.1f y %5.1f n %4d z %.2f..%.2f' % (u[idx, 0] / 10, u[idx, 1] / 10, cnt[idx], z.min(), z.max()))
