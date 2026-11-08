#!/usr/bin/env python3
"""Compare all eight pose streams and state to recorded messages; check NAO TF."""
import hashlib
import io
import json
from pathlib import Path
import sys
import time
import rosbag
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from scenario import IDS

def digest(msg):
    data=io.BytesIO();msg.serialize(data);return hashlib.sha256(data.getvalue()).hexdigest()

bag,ready,stop,out=map(Path,sys.argv[1:])
topics=['/loc_pose_'+r for r in IDS]+['/demo/state','/ball/pose','/map','/field_markings','/planning_graph']
expected={t:[] for t in topics};actual={t:[] for t in topics}
with rosbag.Bag(str(bag)) as stream:
    for topic,msg,_ in stream.read_messages(topics=topics):expected[topic].append(digest(msg))
rospy.init_node('playback_verifier',anonymous=True)
buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
subs=[]
for topic in topics:
    msg_type={'/demo/state':String,'/map':OccupancyGrid,'/field_markings':MarkerArray,'/planning_graph':MarkerArray}.get(topic,PoseStamped)
    subs.append(rospy.Subscriber(topic,msg_type,
        lambda msg,t=topic:actual[t].append(digest(msg)),queue_size=2000))
ready.write_text('ready');frames=set()
while not stop.exists() and not rospy.is_shutdown():
    for rid in IDS:
        for suffix in ['base_link','CameraTop','CameraBottom','Accelerometer','Gyrometer']:
            frame=rid+'/'+suffix
            try:
                buffer.lookup_transform('map',frame,rospy.Time(0));frames.add(frame)
            except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):pass
    time.sleep(.05)
result={'identical_recorded_and_replayed_messages':all(expected[t] and expected[t]==actual[t] for t in topics),
        'streams':{t:{'recorded':len(expected[t]),'replayed':len(actual[t])} for t in topics},
        'expected_robot_tf_chains':40,'verified_robot_tf_chains':len(frames),
        'tf_frames':sorted(frames)}
out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
sys.exit(0 if result['identical_recorded_and_replayed_messages'] and len(frames)==40 else 1)
