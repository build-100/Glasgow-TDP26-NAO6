#!/usr/bin/env python3
"""ROS adapter for the fixed nine-region integration demonstration."""
import json
import math
import time
from pathlib import Path as FilePath
import xml.etree.ElementTree as ET
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from scenario import IDS, Scenario, centre, edges, region

def pose(x,y,yaw,stamp):
    p=PoseStamped();p.header.frame_id='map';p.header.stamp=stamp
    p.pose.position.x=x+4.5;p.pose.position.y=y+3.0
    p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
    return p
def marker(mid,kind,stamp,color=(1,1,1,1)):
    m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns='football';m.id=mid
    m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1
    m.color.r,m.color.g,m.color.b,m.color.a=color
    return m
def visuals(s,stamp):
    allm=[]
    for k,rid in enumerate(IDS):
        color=(1,.65,.65,1) if rid.startswith('red') else (.55,.8,1,1)
        x,y,yaw=s['robots'][rid]
        m=marker(30+k,Marker.TEXT_VIEW_FACING,stamp,color)
        m.pose.position=Point(x,y+.27,.65);m.scale.z=.16
        m.text=rid.upper()+' | '+s['roles'][rid]
        allm.append(m)
    b=marker(50,Marker.SPHERE,stamp,(1,.72,.12,1));b.pose.position=Point(*s['ball'],.08)
    b.scale.x=b.scale.y=b.scale.z=.14;allm.append(b)
    title=marker(60,Marker.TEXT_VIEW_FACING,stamp);title.pose.position=Point(0,4.35,.05);title.scale.z=.22
    title.text='NAO6 V2 | ROUND %d | RED %d : %d BLUE'%(s['round'],*s['score']);allm.append(title)
    hud=marker(61,Marker.TEXT_VIEW_FACING,stamp,(.8,.9,1,1));hud.pose.position=Point(0,-4.35,.05);hud.scale.z=.18
    hud.text='T=%d | %s ATTACK | %s | Ball: %s'%(s['step'],s['possession'],s['phase'],s['ball_state']);allm.append(hud)
    contest=marker(62,Marker.TEXT_VIEW_FACING,stamp,(1,.85,.35,1));contest.pose.position=Point(0,-4.7,.05);contest.scale.z=.16
    c=s['contest']
    contest.text=('P=0.40 | u=%.3f | %s | One attempt used - no retry'%(c['draw'],'TURNOVER' if c['turnover'] else 'POSSESSION RETAINED')) if c else 'P=0.40 | One contest per round | Waiting for eligible same-ball-region encounter'
    allm.append(contest)
    note=marker(63,Marker.TEXT_VIEW_FACING,stamp,(.7,.75,.8,1));note.pose.position=Point(0,-5.02,.05);note.scale.z=.14
    note.text='Fixed routes + selected seed 15 | Kinematic demo, static joints | Goalkeepers inactive';allm.append(note)
    banner=marker(64,Marker.TEXT_VIEW_FACING,stamp,(1,.9,.25,1));banner.pose.position=Point(0,3.65,.08);banner.scale.z=.22
    if s['phase']=='TURNOVER_ENDS_T':banner.text='TURNOVER: BLUE3 -> RED2 | T%d ends | Red counterattack next'%s['step']
    elif s['setup_reset']:banner.text='ROUND %d SETUP / RESTART - NOT IN PLAY'%s['round']
    elif s['phase']=='GOAL_RED':banner.text='RED GOAL'
    else:banner.action=Marker.DELETE
    allm.append(banner)
    target=marker(80,Marker.ARROW,stamp,(1,.8,.2,1));target.scale.x=.035;target.scale.y=.09;target.scale.z=.1
    r=s['owner'];a=s['robots'][r];b=s['targets'][r]
    target.points=[Point(a[0],a[1],.12),Point(b[0],b[1],.12)];allm.append(target)
    for m in allm:
        m.pose.position.x+=4.5;m.pose.position.y+=3
    return MarkerArray(markers=allm)

def main():
    rospy.init_node('global_demo')
    rospy.set_param('/demo_config/version',2)
    rospy.set_param('/demo_config/contest_probability',.4)
    rospy.set_param('/demo_config/showcase_seed',15)
    rospy.set_param('/demo_config/contest_limit','one eligible contest per round; no retry')
    scene=Scenario(); hz=20
    poses={r:rospy.Publisher('/loc_pose_'+r,PoseStamped,queue_size=10) for r in IDS}
    paths={r:rospy.Publisher('/path_'+r,Path,queue_size=2) for r in IDS}
    joints={r:rospy.Publisher('/joint_states_'+r,JointState,queue_size=10) for r in IDS}
    histories={r:Path() for r in IDS}
    joint_values={'LShoulderPitch':1.4,'RShoulderPitch':1.4,'LShoulderRoll':.2,'RShoulderRoll':-.2,
                  'LElbowRoll':-.4,'RElbowRoll':.4,'LHipPitch':-.05,'RHipPitch':-.05,
                  'LKneePitch':.1,'RKneePitch':.1,'LAnklePitch':-.05,'RAnklePitch':-.05}
    names={}
    for r in IDS:
        xml=ET.fromstring(rospy.get_param('/'+r+'/robot_description'))
        names[r]=[j.get('name') for j in xml.findall('joint') if j.get('type')!='fixed' and j.find('mimic') is None]
    marks=rospy.Publisher('/team_markers',MarkerArray,queue_size=2,latch=True)
    ball=rospy.Publisher('/ball/pose',PoseStamped,queue_size=10)
    state=rospy.Publisher('/demo/state',String,queue_size=10)
    event=rospy.Publisher('/demo/events',String,queue_size=10,latch=True)
    contests=rospy.Publisher('/demo/contests',String,queue_size=10,latch=True)
    last_contest_round=0
    tf=tf2_ros.TransformBroadcaster()
    # Delay gives rosbag record -a time to discover all advertised topics.
    time.sleep(2.5)
    origin=rospy.Time.now().to_sec();rate=rospy.Rate(hz);counter=0;last=None
    while not rospy.is_shutdown():
        stamp=rospy.Time.now();t=stamp.to_sec()-origin;s=scene.sample(t)
        transforms=[]
        for r in IDS:
            p=pose(*s['robots'][r],stamp);poses[r].publish(p)
            tr=TransformStamped();tr.header=p.header;tr.child_frame_id=r+'/base_link'
            tr.transform.translation.x=p.pose.position.x;tr.transform.translation.y=p.pose.position.y
            tr.transform.translation.z=.34;tr.transform.rotation=p.pose.orientation;transforms.append(tr)
            js=JointState();js.header.stamp=stamp;js.name=names[r]
            js.position=[joint_values.get(n.split('/')[-1],0) for n in js.name];joints[r].publish(js)
            if counter%4==0:
                h=histories[r];h.header=p.header;h.poses.append(p);h.poses=h.poses[-1200:];paths[r].publish(h)
        tf.sendTransform(transforms)
        ball.publish(pose(s['ball'][0],s['ball'][1],0,stamp))
        state.publish(json.dumps(s))
        if s['contest'] and s['contest']['round']!=last_contest_round:
            contests.publish(json.dumps(s['contest']));last_contest_round=s['contest']['round']
        if counter%4==0:marks.publish(visuals(s,stamp))
        key=(s['step'],s['phase'])
        if key!=last:event.publish(json.dumps(s));last=key
        if t>=scene.duration:break
        counter+=1;rate.sleep()
    rospy.loginfo('Scripted scenario complete. No live pose publisher remains.')

if __name__=='__main__':main()
