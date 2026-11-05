#!/usr/bin/env python3
"""Open recorded NAO6 data with model parameters restored, without live publishers."""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client
parser=argparse.ArgumentParser()
parser.add_argument('bag',type=Path);parser.add_argument('--loop',action='store_true')
args=parser.parse_args();bag=args.bag.resolve()
if not bag.is_file():parser.error('Bag file does not exist')
root=Path(__file__).resolve().parents[1]
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11433',ROS_HOSTNAME='127.0.0.1',DISABLE_ROS1_EOL_WARNINGS='1')
os.environ.pop('ROS_IP',None)
os.environ['ROS_PACKAGE_PATH']=str(root.parent)+':'+os.environ.get('ROS_PACKAGE_PATH','')
children=[]
def interrupted(signum,frame):
    raise KeyboardInterrupt
signal.signal(signal.SIGINT,interrupted)
signal.signal(signal.SIGTERM,interrupted)
def start(cmd):
    p=subprocess.Popen(cmd,start_new_session=True);children.append(p);return p
try:
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',11433))==0:raise RuntimeError('Port 11433 is in use. Stop the previous demo first.')
    start(['roscore','-p','11433'])
    master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']);until=time.monotonic()+20
    while True:
        try:
            if master.getPid('/replay_setup')[0]==1:break
        except OSError:pass
        if time.monotonic()>until:raise RuntimeError('ROS startup timeout')
        time.sleep(.1)
    launch=start(['roslaunch','eng5325_global_demo_v2','replay.launch'])
    time.sleep(5)
    play=start(['rosbag','play','--clock','--delay=2']+(['--loop'] if args.loop else [])+[str(bag)])
    while play.poll() is None and launch.poll() is None:time.sleep(.2)
except KeyboardInterrupt:pass
finally:
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
