#!/usr/bin/env python3
"""ROS adapter: computed policies, kinematic skills, original namespaced topics."""
import json
import math
import time
from pathlib import Path as FilePath
import sys
import xml.etree.ElementTree as ET
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped,TransformStamped,Point
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from scenario import IDS,Scenario
from skills import joint_pose

def pose(x,y,yaw,stamp):
    p=PoseStamped();p.header.frame_id='map';p.header.stamp=stamp
    p.pose.position.x=x+4.5;p.pose.position.y=y+3
    p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
    return p

def marker(mid,kind,stamp,color=(1,1,1,1)):
    m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns='football_v3';m.id=mid
    m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1
    m.color.r,m.color.g,m.color.b,m.color.a=color
    return m

def visuals(s,stamp):
    marks=[]
    for k,rid in enumerate(IDS):
        color=(1,.65,.65,1) if rid.startswith('red') else (.55,.8,1,1)
        x,y,yaw=s['robots'][rid]
        m=marker(30+k,Marker.TEXT_VIEW_FACING,stamp,color)
        m.pose.position=Point(x,y+.30,.65);m.scale.z=.14
        m.text=rid.upper()+' | '+s['roles'][rid];marks.append(m)
    ball=marker(50,Marker.SPHERE,stamp,(1,.72,.12,1));ball.pose.position=Point(*s['ball'],.08)
    ball.scale.x=ball.scale.y=ball.scale.z=.14;marks.append(ball)
    def text(mid,y,size,value,color=(1,1,1,1)):
        m=marker(mid,Marker.TEXT_VIEW_FACING,stamp,color);m.pose.position=Point(0,y,.06)
        m.scale.z=size;m.text=value
        # RViz rejects an empty TEXT_VIEW_FACING marker. Clear transient banners.
        if not value:m.action=Marker.DELETE
        marks.append(m)
    text(60,4.35,.22,'V4 | FIRST TO 3 | RED %d : %d BLUE'%tuple(s['score']))
    text(65,4.0,.16,'RED: VDN + Double DQN     BLUE: sequential marginal-gain greedy')
    text(61,-4.30,.17,'ROUND %d | T%d | %s | Ball: %s'%(s['round'],s['step'],s['phase'],s['ball_state']),(.8,.9,1,1))
    c=s['contest']
    text(62,-4.65,.15,('P=0.40 | u=%.3f | %s | robot separation %.2f m'%(c['draw'],'TURNOVER' if c['turnover'] else 'RETAINED',c['robot_distance'])) if c else
         'Contest requires same region + both within 0.50 m of ball; at most once per goal round',(1,.85,.35,1))
    text(63,-4.98,.135,'Selected policy match | Kinematic skills, not validated dynamics | Goalkeepers inactive',(.7,.75,.8,1))
    banner=''
    if s['setup_reset']:banner='KICKOFF / RESET - NOT AN IN-PLAY PASS'
    elif s['phase']=='TURNOVER_ENDS_T':banner=c['owner_before'].upper()+' -> '+c['owner_after'].upper()+' | NEXT ACTION AT T+1'
    elif s['phase'].startswith('GOAL_'):banner=s['phase'].replace('_',' ')
    elif s['phase']=='MATCH_COMPLETE_FIRST_TO_THREE':banner=('RED' if s['score'][0]==3 else 'BLUE')+' WINS - FIRST TO THREE'
    text(64,3.63,.20,banner,(1,.9,.25,1))
    # A close-range ring makes contest eligibility visible without path trails.
    ring=marker(70,Marker.CYLINDER,stamp,(1,.80,.18,.12));ring.pose.position=Point(*s['ball'],.012)
    ring.scale.x=ring.scale.y=1.0;ring.scale.z=.008;marks.append(ring)
    for m in marks:m.pose.position.x+=4.5;m.pose.position.y+=3
    return MarkerArray(markers=marks)

def main():
    rospy.init_node('global_demo')
    for k,v in dict(version=4,contest_probability=.4,contest_ball_radius=.5,contest_robot_distance=.9,
                    goals_to_win=3,goalkeepers='fixed; no gameplay effect',
                    policy='VDN+Double DQN vs sequential marginal-gain greedy',motion='kinematic skill simulation; not validated gait').items():
        rospy.set_param('/demo_config/'+k,v)
    scene=Scenario();hz=20
    poses={r:rospy.Publisher('/loc_pose_'+r,PoseStamped,queue_size=20) for r in IDS}
    paths={r:rospy.Publisher('/path_'+r,Path,queue_size=2) for r in IDS}
    joints={r:rospy.Publisher('/joint_states_'+r,JointState,queue_size=20) for r in IDS}
    histories={r:Path() for r in IDS};names={}
    for r in IDS:
        xml=ET.fromstring(rospy.get_param('/'+r+'/robot_description'))
        names[r]=[j.get('name') for j in xml.findall('joint') if j.get('type')!='fixed' and j.find('mimic') is None]
    marks=rospy.Publisher('/team_markers',MarkerArray,queue_size=2,latch=True)
    ball=rospy.Publisher('/ball/pose',PoseStamped,queue_size=20)
    state=rospy.Publisher('/demo/state',String,queue_size=20)
    event=rospy.Publisher('/demo/events',String,queue_size=20,latch=True)
    contests=rospy.Publisher('/demo/contests',String,queue_size=20,latch=True)
    skills=rospy.Publisher('/demo/skills',String,queue_size=20)
    utilities=rospy.Publisher('/policy/utilities',String,queue_size=20)
    marginal=rospy.Publisher('/baseline/marginal_gains',String,queue_size=20)
    tf=tf2_ros.TransformBroadcaster();last=None;last_contest=None;last_step=None
    time.sleep(2.5)
    origin=rospy.Time.now().to_sec();rate=rospy.Rate(hz);counter=0
    while not rospy.is_shutdown():
        stamp=rospy.Time.now();t=stamp.to_sec()-origin;s=scene.sample(t);transforms=[]
        for r in IDS:
            p=pose(*s['robots'][r],stamp);poses[r].publish(p)
            tr=TransformStamped();tr.header=p.header;tr.child_frame_id=r+'/base_link'
            tr.transform.translation.x=p.pose.position.x;tr.transform.translation.y=p.pose.position.y
            tr.transform.translation.z=.34;tr.transform.rotation=p.pose.orientation;transforms.append(tr)
            q=joint_pose(s['phase'],s['phase_elapsed'],s['phase_progress'],r==s['actor'],s['moving'][r]) if not r.endswith('4') else joint_pose('STAND',0,0)
            js=JointState();js.header.stamp=stamp;js.name=names[r]
            js.position=[q.get(n.split('/')[-1],0) for n in js.name];joints[r].publish(js)
            if counter%4==0:
                h=histories[r];h.header=p.header;h.poses.append(p);h.poses=h.poses[-2500:];paths[r].publish(h)
        tf.sendTransform(transforms)
        ball.publish(pose(*s['ball'],0,stamp));state.publish(json.dumps(s))
        if s['contest']:
            key=(s['round'],s['contest']['step'])
            if key!=last_contest:contests.publish(json.dumps(s['contest']));last_contest=key
        if counter%4==0:marks.publish(visuals(s,stamp))
        key=(s['round'],s['step'],s['phase'])
        if key!=last:
            event.publish(json.dumps(s));last=key
            if s['phase']=='MACRO_STEP_COMPLETE':skills.publish(json.dumps(dict(round=s['round'],step=s['step'],feedback=s['skill_feedback'])))
        sk=(s['round'],s['step'])
        if sk!=last_step:
            if s['policy_utilities']:utilities.publish(json.dumps(dict(round=s['round'],step=s['step'],**s['policy_utilities'])))
            if s['baseline_allocation']:marginal.publish(json.dumps(dict(round=s['round'],step=s['step'],**s['baseline_allocation'])))
            last_step=sk
        if t>=scene.duration:break
        counter+=1;rate.sleep()
    rospy.loginfo('First-to-three match execution complete; pose publisher stopped.')
if __name__=='__main__':main()
