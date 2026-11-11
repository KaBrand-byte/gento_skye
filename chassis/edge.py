#!/usr/bin/env python3
"""edge.py xmin xmax ymin ymax  -> RANSAC line fit of lidar points (map frame) inside box, per height band"""
import sys, time, math, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from slam.msg import LocationData
x0b,x1b,y0b,y1b=map(float,sys.argv[1:5])
rclpy.init(); n=Node('claude_edge'); C=[]; L=[]
n.create_subscription(PointCloud2,'/driver/lidar/point_cloud/Data',lambda m: C.append(m) if len(C)<8 else None, qos_profile_sensor_data)
n.create_subscription(LocationData,'slam/LocationData',lambda m: L.append(m), qos_profile_sensor_data)
t=time.time()
while time.time()-t<5: rclpy.spin_once(n,timeout_sec=0.05)
p=L[-1].pose; X0,Y0,TH=p.position.x,p.position.y,p.orientation.z
print('pose %.3f %.3f %.4f conf %d'%(X0,Y0,TH,L[-1].confidence))
a=np.concatenate([np.stack([s['x'],s['y'],s['z']],1).astype(float) for s in (pc2.read_points(m, field_names=('x','y','z'), skip_nans=True) for m in C)])
a=a[np.isfinite(a).all(1)]
c,s=math.cos(TH),math.sin(TH); mx=X0+c*a[:,0]-s*a[:,1]; my=Y0+s*a[:,0]+c*a[:,1]
rng=np.random.default_rng(0)
for lo,hi in ((0.1,0.4),(0.4,0.7),(0.7,0.8),(0.8,1.0)):
    sel=(a[:,2]>=lo)&(a[:,2]<hi)&(mx>=x0b)&(mx<=x1b)&(my>=y0b)&(my<=y1b)
    xy=np.stack([mx[sel],my[sel]],1)
    if len(xy)<30: print(f'z{lo}-{hi}: {len(xy)} pts (too few)'); continue
    best=(0,None)
    for _ in range(500):
        i,j=rng.choice(len(xy),2,replace=False); d=xy[j]-xy[i]; Ln=np.linalg.norm(d)
        if Ln<0.2: continue
        d/=Ln; nr=np.array([-d[1],d[0]]); inl=np.abs((xy-xy[i])@nr)<0.015
        if inl.sum()>best[0]: best=(inl.sum(),inl)
    if best[1] is None: print(f"z{lo}-{hi}: no line"); continue
    pts=xy[best[1]]; cen=pts.mean(0); u,sv,vt=np.linalg.svd(pts-cen); d=vt[0]
    if d[0]<0: d=-d
    t_=(pts-cen)@d
    ang=math.degrees(math.atan2(d[1],d[0]))
    print(f'z{lo}-{hi}: n={len(xy)} inliers={best[0]} line dir={ang:.2f}deg through ({cen[0]:.3f},{cen[1]:.3f}) span {t_.min():.2f}..{t_.max():.2f} -> ends ({(cen+d*t_.min())[0]:.2f},{(cen+d*t_.min())[1]:.2f}) ({(cen+d*t_.max())[0]:.2f},{(cen+d*t_.max())[1]:.2f}) rms {np.sqrt((((pts-cen)@np.array([-d[1],d[0]]))**2).mean()):.4f}')
