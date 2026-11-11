#!/usr/bin/env python3
"""Helper for nav tests. Usage:
  nav.py state
  nav.py mode auto|manual
  nav.py goto x y yaw_rad [x y yaw ...]  [--speed 0.3]  (vendor convention: orientation.w = yaw rad)
  nav.py goto_q x y yaw ...   (standard quaternion convention)
  nav.py measure            (pose + scan saved for edge fitting)
"""
import sys, time, math, json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from move.msg import State
from move.srv import SetControlMode
from slam.msg import LocationData
rclpy.init(); n = Node('claude_navtool')
S=[None]; L=[None]
n.create_subscription(State,'move/State',lambda m: S.__setitem__(0,m),10)
n.create_subscription(LocationData,'slam/LocationData',lambda m: L.__setitem__(0,m),qos_profile_sensor_data)
def spin(t):
    e=time.time()+t
    while time.time()<e: rclpy.spin_once(n,timeout_sec=0.05)
def pose():
    p=L[0].pose; return p.position.x,p.position.y,p.orientation.z,L[0].confidence
cmd=sys.argv[1]; spin(1.0)
if cmd=='state':
    s=S[0]; x,y,t,c=pose(); print(json.dumps(dict(soft_auto=s.is_soft_auto,hard_auto=s.is_hard_auto,parking=s.is_parking_mode,avoid=s.is_avoid,x=round(x,3),y=round(y,3),yaw=round(t,4),conf=c)))
elif cmd=='mode':
    c=n.create_client(SetControlMode,'move/SetControlMode'); c.wait_for_service(timeout_sec=5)
    r=SetControlMode.Request(); r.mode=0 if sys.argv[2]=='auto' else 1
    f=c.call_async(r); rclpy.spin_until_future_complete(n,f,timeout_sec=5); spin(1.0); print(f.result().error_msg, 'soft_auto=',S[0].is_soft_auto)
elif cmd in ('goto','goto_q','goto_z'):
    from server.srv import CreateTaskByPose
    from server.msg import TargetInfoWithPose
    args=sys.argv[2:]; speed=0.3
    if '--speed' in args: i=args.index('--speed'); speed=float(args[i+1]); args=args[:i]+args[i+2:]
    v=[float(a) for a in args]; tl=[]
    for k in range(0,len(v),3):
        x,y,yaw=v[k:k+3]; t=TargetInfoWithPose(); t.target_pose.position.x=x; t.target_pose.position.y=y
        if cmd=="goto": t.target_pose.orientation.x=0.0; t.target_pose.orientation.y=0.0; t.target_pose.orientation.z=0.0; t.target_pose.orientation.w=yaw
        elif cmd=="goto_z": t.target_pose.orientation.x=0.0; t.target_pose.orientation.y=0.0; t.target_pose.orientation.z=yaw; t.target_pose.orientation.w=1.0
        else: t.target_pose.orientation.z=math.sin(yaw/2); t.target_pose.orientation.w=math.cos(yaw/2)
        t.action_type=0; t.action_param=0.0; tl.append(t)
    c=n.create_client(CreateTaskByPose,'server/CreateTaskByPose'); c.wait_for_service(timeout_sec=5)
    r=CreateTaskByPose.Request(); r.max_speed=speed; r.max_loop_times=1; r.target_list=tl
    f=c.call_async(r); rclpy.spin_until_future_complete(n,f,timeout_sec=10); res=f.result()
    print('task', res.task_id, res.error_code, res.error_msg)
elif cmd=='node':
    from server.srv import CreateTaskById
    from server.msg import TargetInfoWithId
    args=sys.argv[2:]; speed=0.3; mode=0
    if '--speed' in args: i=args.index('--speed'); speed=float(args[i+1]); args=args[:i]+args[i+2:]
    if '--mode' in args: i=args.index('--mode'); mode=int(args[i+1]); args=args[:i]+args[i+2:]
    tl=[]
    for a in args:
        t=TargetInfoWithId(); t.target_id=int(a); t.action_type=0; t.action_param=0.0; tl.append(t)
    c=n.create_client(CreateTaskById,'server/CreateTaskById'); c.wait_for_service(timeout_sec=5)
    r=CreateTaskById.Request(); r.navigation_mode=mode; r.max_speed=speed; r.max_loop_times=1; r.target_list=tl
    f=c.call_async(r); rclpy.spin_until_future_complete(n,f,timeout_sec=10); res=f.result()
    print('task', res.task_id, res.error_code, res.error_msg)
elif cmd=='wait':
    # wait until pose is still for 3 s (or timeout), print final pose
    tmo=float(sys.argv[2]) if len(sys.argv)>2 else 90; t0=time.time(); last=pose(); still=time.time(); moved=False
    while time.time()-t0<tmo:
        spin(0.5); p=pose()
        if abs(p[0]-last[0])>0.003 or abs(p[1]-last[1])>0.003 or abs(p[2]-last[2])>0.005: still=time.time(); moved=True
        last=p
        if moved and time.time()-still>3.0: break
    x,y,t,c=pose(); s=S[0]
    print(json.dumps(dict(x=round(x,3),y=round(y,3),yaw_deg=round(math.degrees(t),2),conf=c,avoid=s.is_avoid,secs=round(time.time()-t0,1))))
