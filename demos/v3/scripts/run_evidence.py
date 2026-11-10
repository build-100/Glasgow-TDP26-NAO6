#!/usr/bin/env python3
"""Record actual V3 ROS messages, replay them alone, verify, and film RViz."""
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
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11435',ROS_HOSTNAME='127.0.0.1',DISPLAY=':0',
                  XAUTHORITY=str(Path.home()/'.Xauthority'),DISABLE_ROS1_EOL_WARNINGS='1')
os.environ.pop('ROS_IP',None)
os.environ['ROS_PACKAGE_PATH']=str(ROOT.parent)+':'+os.environ.get('ROS_PACKAGE_PATH','')
children=[];logs=[]

def start(name,args):
    log=(RUN/(name+'.log')).open('w');logs.append(log)
    p=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    children.append(p);return p
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
def run(args):return subprocess.run(args,stdin=subprocess.DEVNULL,check=True)
def wait_file(path,p,timeout=30):
    until=time.monotonic()+timeout
    while not path.exists():
        if p.poll() is not None or time.monotonic()>until:raise RuntimeError('Readiness failed: '+str(path))
        time.sleep(.1)
def wait_progress(seconds,stage):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        print(json.dumps(dict(stage=stage,remaining_seconds=round(end-time.monotonic(),1),run=str(RUN))),flush=True)
        time.sleep(min(20,max(0,end-time.monotonic())))

try:
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',11435))==0:raise RuntimeError('V3 port occupied; existing master left untouched.')
    from scenario import Scenario
    scene=Scenario()
    (RUN/'execution_plan.json').write_text(json.dumps(dict(duration=scene.duration,phases=scene.phases),indent=2))
    (RUN/'demo_plan.json').write_bytes((ROOT/'plans/demo_plan.json').read_bytes())
    core=start('core',['roscore','-p','11435']);master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    until=time.monotonic()+20
    while True:
        try:
            if master.getPid('/v3_setup')[0]==1:break
        except OSError:pass
        if time.monotonic()>until:raise RuntimeError('Master startup timeout')
        time.sleep(.1)
    bag=RUN/'nao6_v3_first_to_three.bag'
    rec=start('record',['rosbag','record','-a','--lz4','-O',str(bag),'__name:=v3_recorder'])
    time.sleep(1)
    live=start('live',['roslaunch','eng5325_global_demo_v3','live.launch','rviz:=false'])
    rviz=start('rviz_live',['rviz','-d',str(ROOT/'rviz/nao6_global.rviz')])
    time.sleep(6)
    advertised=dict(master.getSystemState('/v3_setup')[2][0])
    if not all(t in advertised for t in ['/loc_pose_red1','/loc_pose_blue4','/map','/demo/skills']):
        raise RuntimeError('V3 live publishers not ready; inspect live.log.')
    run([sys.executable,str(ROOT/'scripts/audit_runtime.py'),str(RUN/'runtime_model_check.json')])
    wait_progress(scene.duration+2,'recording_live_execution')
    run(['gnome-screenshot','-f',str(RUN/'live_final.png')])
    run(['rosparam','dump',str(RUN/'parameters.yaml')])
    stop(rec);stop(live);stop(rviz)
    (RUN/'bag_info.txt').write_text(subprocess.check_output(['rosbag','info',str(bag)],text=True))
    run([sys.executable,str(ROOT/'scripts/verify_scenario_bag.py'),str(bag),str(RUN/'scenario_record_check.json')])
    publishers=[n for topic,nodes in master.getSystemState('/v3_setup')[2][0] if topic.startswith('/loc_pose_') for n in nodes]
    if publishers:raise RuntimeError('Live robot pose publishers remain before replay.')
    replay=start('replay_config',['roslaunch','eng5325_global_demo_v3','replay.launch','rviz:=false'])
    time.sleep(1)
    rviz=start('rviz_replay',['rviz','-d',str(ROOT/'rviz/nao6_global.rviz')])
    verify=start('verify',[sys.executable,str(ROOT/'scripts/verify_replay.py'),str(bag),str(RUN/'ready'),str(RUN/'stop'),str(RUN/'verification.json')])
    wait_file(RUN/'ready',verify);time.sleep(3)
    video=start('video',['ffmpeg','-nostdin','-y','-f','x11grab','-framerate','15','-video_size','1848x1052','-i',':0.0+72,28',
                         '-an','-c:v','libx264','-preset','veryfast','-crf','23','-pix_fmt','yuv420p',str(RUN/'replay_full_match.mp4')])
    play=start('play',['rosbag','play','--clock','--delay=2',str(bag)])
    start_time=time.monotonic();captured=set();last_report=-1
    screenshots=[('opening',8.0)]
    for label in ['TURNOVER_ENDS_T','CONTEST_RETAINED_NO_RETRY','KICK_FORWARD','MATCH_COMPLETE_FIRST_TO_THREE']:
        match=next((p for p in scene.phases if p['label']==label),None)
        if match:screenshots.append((label.lower(),match['start']+5.2))
    while play.poll() is None:
        elapsed=time.monotonic()-start_time
        for label,when in screenshots:
            if elapsed>=when and label not in captured:
                run(['gnome-screenshot','-f',str(RUN/(label+'.png'))]);captured.add(label)
        if int(elapsed//20)!=last_report:
            print(json.dumps(dict(stage='recording_verified_bag_playback',elapsed_seconds=round(elapsed,1),run=str(RUN))),flush=True)
            last_report=int(elapsed//20)
        if elapsed>scene.duration+60:raise RuntimeError('Replay exceeded expected duration.')
        time.sleep(.1)
    if play.returncode:raise RuntimeError('rosbag play failed')
    stop(video);time.sleep(.6);(RUN/'stop').write_text('stop')
    if verify.wait(timeout=15)!=0:raise RuntimeError('Recorded/replayed messages or TF verification failed.')
    result=json.loads((RUN/'verification.json').read_text())
    result.update(live_pose_publishers_stopped_before_playback=True,match_rule='first_to_three',
                  final_score=scene.score,motion_backend='kinematic; not validated humanoid dynamics')
    (RUN/'verification.json').write_text(json.dumps(result,indent=2))
    print('EVIDENCE_COMPLETE '+json.dumps(result),flush=True)
    print('RUN_DIRECTORY='+str(RUN),flush=True)
finally:
    for p in reversed(children):stop(p)
    for log in logs:log.close()
