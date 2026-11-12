#!/usr/bin/env python3
"""Read-only audit of model parameters, state-publisher wiring and fixed field."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import rospy
import rosgraph
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from scenario import IDS
root=Path(__file__).resolve().parents[1]
rospy.init_node('runtime_model_audit',anonymous=True)
pubs,subs,_=rosgraph.Master(rospy.get_name()).getSystemState();subs=dict(subs)
source=ET.parse(root/'models/nao6_collision_visual.urdf').getroot()
report={}
for rid in IDS:
    expected=ET.parse(root/'urdf'/(rid+'.urdf')).getroot()
    loaded=ET.fromstring(rospy.get_param('/'+rid+'/robot_description'))
    physics=True
    for a,b in zip(source.findall('link'),loaded.findall('link')):
        for tag in ['inertial','collision']:
            physics &= [ET.tostring(e) for e in a.findall(tag)]==[ET.tostring(e) for e in b.findall(tag)]
    for a,b in zip(source.findall('joint'),loaded.findall('joint')):
        physics &= a.get('type')==b.get('type')
        for tag in ['origin','axis','limit','dynamics']:
            physics &= [ET.tostring(e) for e in a.findall(tag)]==[ET.tostring(e) for e in b.findall(tag)]
    report[rid]={'loaded_description_matches_generated_urdf':ET.tostring(expected)==ET.tostring(loaded),
                 'inertias_collisions_joint_geometry_and_limits_preserved':bool(physics),
                 'independent_moving_joints':sum(j.get('type')!='fixed' and j.find('mimic') is None for j in loaded.findall('joint')),
                 'link_count':len(loaded.findall('link')),
                 'state_publisher_subscribes_to_correct_joint_topic':('/'+rid+'/robot_state_publisher') in subs.get('/joint_states_'+rid,[])}
grid=rospy.wait_for_message('/map',OccupancyGrid,timeout=5)
pitch=rospy.wait_for_message('/field_markings',MarkerArray,timeout=5)
config=json.loads((root/'rviz/nao6_global.rviz').read_text())['Visualization Manager']
result={'robots':report,'fixed_frame':config['Global Options']['Fixed Frame'],
        'enabled_robot_model_displays':sum(d['Class']=='rviz/RobotModel' and d['Enabled'] for d in config['Displays']),
        'enabled_path_displays':sum(d['Class']=='rviz/Path' and d['Enabled'] for d in config['Displays']),
        'map':{'frame':grid.header.frame_id,'resolution':grid.info.resolution,'width':grid.info.width,'height':grid.info.height,'origin':[grid.info.origin.position.x,grid.info.origin.position.y]},
        'fixed_pitch_markers_in_map':all(m.header.frame_id=='map' for m in pitch.markers),
        'green_rgb':[pitch.markers[0].color.r,pitch.markers[0].color.g,pitch.markers[0].color.b]}
passed=all(r['loaded_description_matches_generated_urdf'] and r['inertias_collisions_joint_geometry_and_limits_preserved'] and r['independent_moving_joints']==21 and r['state_publisher_subscribes_to_correct_joint_topic'] for r in report.values())
passed &= result['enabled_robot_model_displays']==8 and result['enabled_path_displays']==0 and result['fixed_frame']=='map' and result['fixed_pitch_markers_in_map']
result['passed']=bool(passed)
Path(sys.argv[1]).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
sys.exit(0 if passed else 1)
