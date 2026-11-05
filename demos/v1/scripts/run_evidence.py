#!/usr/bin/env python3
"""Run this integration check on the authorised Ubuntu/Noetic target only."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client
ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'evidence'/time.strftime('%Y%m%d_%H%M%S');RUN.mkdir(parents=True)
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11432',ROS_HOSTNAME='127.0.0.1',
                  DISPLAY=':0',XAUTHORITY=str(Path.home()/'.Xauthority'),DISABLE_ROS1_EOL_WARNINGS='1')
os.environ.pop('ROS_IP',None)
os.environ['ROS_PACKAGE_PATH']=str(ROOT.parent)+':'+os.environ.get('ROS_PACKAGE_PATH','')
children=[];logs=[]
def start(name,args):
    log=(RUN/(name+'.log')).open('w');logs.append(log)
    p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    children.append(p);return p
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
def run(args):return subprocess.run(args,check=True)
def wait_file(path,p,timeout=20):
    until=time.monotonic()+timeout
    while not path.exists():
        if p.poll() is not None or time.monotonic()>until:raise RuntimeError('Readiness failed: '+str(path))
        time.sleep(.05)
try:
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',11432))==0:raise RuntimeError('Port 11432 in use; leaving existing ROS master untouched.')
    core=start('core',['roscore','-p','11432']);master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    until=time.monotonic()+20
    while True:
        try:
            if master.getPid('/setup')[0]==1:break
        except OSError:pass
        if time.monotonic()>until:raise RuntimeError('Master timeout')
        time.sleep(.1)
    bag=RUN/'nao6_global.bag'
    rec=start('record',['rosbag','record','-a','-O',str(bag),'__name:=global_recorder'])
    time.sleep(1)
    live=start('live',['roslaunch','eng5325_global_demo','live.launch','rviz:=false'])
    rviz=start('rviz_live',['rviz','-d',str(ROOT/'rviz/nao6_global.rviz')])
    # Complete the finite scenario. roslaunch remains alive for state publishers.
    from scenario import Scenario
    time.sleep(6)
    advertised=dict(master.getSystemState('/setup')[2][0])
    if not all(t in advertised for t in ['/loc_pose_red1','/loc_pose_blue4','/map','/field_markings']):
        raise RuntimeError('Required live publisher did not start. Inspect live.log before recording again.')
    run([sys.executable,str(ROOT/'scripts/audit_runtime.py'),str(RUN/'runtime_model_check.json')])
    time.sleep(Scenario().duration+1)
    if live.poll() is not None:raise RuntimeError('Live launch exited early')
    run(['gnome-screenshot','-f',str(RUN/'live.png')])
    run(['rosparam','dump',str(RUN/'parameters.yaml')])
    stop(rec);stop(live);stop(rviz)
    info=subprocess.check_output(['rosbag','info',str(bag)],text=True);(RUN/'bag_info.txt').write_text(info)
    live_state=master.getSystemState('/setup')[2][0]
    publishers=[n for topic,nodes in live_state if topic.startswith('/loc_pose_') for n in nodes]
    if publishers:raise RuntimeError('Live pose publishers remain before playback: '+str(publishers))
    replay=start('replay_config',['roslaunch','eng5325_global_demo','replay.launch','rviz:=false'])
    time.sleep(1)
    rviz=start('rviz_replay',['rviz','-d',str(ROOT/'rviz/nao6_global.rviz')])
    verify=start('verify',[sys.executable,str(ROOT/'scripts/verify_replay.py'),str(bag),str(RUN/'ready'),str(RUN/'stop'),str(RUN/'verification.json')])
    wait_file(RUN/'ready',verify);time.sleep(3)
    video=start('video',['ffmpeg','-y','-f','x11grab','-framerate','15','-video_size','1848x1052','-i',':0.0+72,28','-an','-c:v','libx264','-preset','veryfast','-crf','23','-pix_fmt','yuv420p',str(RUN/'replay.mp4')])
    play=start('play',['rosbag','play','--clock','--delay=2',str(bag)])
    time.sleep(20)
    run(['gnome-screenshot','-f',str(RUN/'replay.png')])
    if play.wait(timeout=100)!=0:raise RuntimeError('Player failed')
    stop(video);time.sleep(.5);(RUN/'stop').write_text('stop')
    if verify.wait(timeout=8)!=0:raise RuntimeError('Replay verification failed')
    result=json.loads((RUN/'verification.json').read_text())
    result['live_pose_publishers_stopped_before_playback']=True
    (RUN/'verification.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2));print('RUN_DIRECTORY='+str(RUN))
finally:
    for p in reversed(children):stop(p)
    for log in logs:log.close()
