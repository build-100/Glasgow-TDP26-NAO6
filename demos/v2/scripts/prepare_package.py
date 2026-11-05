#!/usr/bin/env python3
"""Derive namespaced NAO6 descriptions from the selected team repository."""
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
REPO=next((p for p in ROOT.parents if (p/'ros/nao6_description').is_dir()), None)
if REPO is None: raise RuntimeError('Keep this demo inside the Glasgow-TDP26-NAO6 repository.')
SOURCE=REPO/'ros/nao6_description'
IDS=['red1','red2','red3','red4','blue1','blue2','blue3','blue4']

def main():
    for d in ['urdf','launch','rviz','models','evidence']:(ROOT/d).mkdir(exist_ok=True)
    original=SOURCE/'urdf/nao6_collision_visual.urdf'
    shutil.copy2(original,ROOT/'models/nao6_collision_visual.urdf')
    shutil.copy2(SOURCE/'LICENSE.upstream',ROOT/'models/LICENSE.upstream')
    shutil.copy2(SOURCE/'source_manifest.json',ROOT/'models/source_manifest.json')
    shutil.copy2(REPO/'LICENSE.txt',ROOT/'LICENSE.txt')
    shutil.copy2(REPO/'configs/field.json',ROOT/'models/field.json')
    model_launch=ET.Element('launch');ET.SubElement(model_launch,'arg',name='publish_tf',default='true')
    for rid in IDS:
        tree=ET.parse(original);root=tree.getroot();root.set('name',rid+'_NAO6')
        for link in root.findall('link'):link.set('name',rid+'/'+link.get('name'))
        for j in root.findall('joint'):
            j.set('name',rid+'/'+j.get('name'))
            for key in ['parent','child']:
                elem=j.find(key);elem.set('link',rid+'/'+elem.get('link'))
            mimic=j.find('mimic')
            if mimic is not None:mimic.set('joint',rid+'/'+mimic.get('joint'))
        rgba='.90 .24 .24 1' if rid.startswith('red') else '.20 .48 .95 1'
        for c in root.findall('.//visual/material/color'):c.set('rgba',rgba)
        tree.write(ROOT/'urdf'/(rid+'.urdf'),encoding='utf-8',xml_declaration=True)
        group=ET.SubElement(model_launch,'group',ns=rid)
        ET.SubElement(group,'param',name='robot_description',textfile='$(find eng5325_global_demo_v2)/urdf/'+rid+'.urdf')
        node=ET.SubElement(group,'node',{'if':'$(arg publish_tf)','pkg':'robot_state_publisher','type':'robot_state_publisher','name':'robot_state_publisher','output':'log'})
        ET.SubElement(node,'param',name='publish_frequency',value='20.0')
        ET.SubElement(node,'remap',{'from':'joint_states','to':'/joint_states_'+rid})
    ET.ElementTree(model_launch).write(ROOT/'launch/models.launch',encoding='utf-8',xml_declaration=True)
    for mode in ['live','replay']:
        root=ET.Element('launch');ET.SubElement(root,'arg',name='rviz',default='true')
        ET.SubElement(root,'param',name='use_sim_time',value='true' if mode=='replay' else 'false')
        inc=ET.SubElement(root,'include',file='$(find eng5325_global_demo_v2)/launch/models.launch')
        ET.SubElement(inc,'arg',name='publish_tf',value='true' if mode=='live' else 'false')
        if mode=='live':
            ET.SubElement(root,'node',pkg='eng5325_global_demo_v2',type='field_map.py',name='fixed_pitch_map',output='log')
        if mode=='live':ET.SubElement(root,'node',pkg='eng5325_global_demo_v2',type='global_demo.py',name='global_demo',output='screen')
        ET.SubElement(root,'node',{'if':'$(arg rviz)','pkg':'rviz','type':'rviz','name':'rviz','args':'-d $(find eng5325_global_demo_v2)/rviz/nao6_global.rviz','output':'log'})
        ET.ElementTree(root).write(ROOT/'launch'/(mode+'.launch'),encoding='utf-8',xml_declaration=True)
    # Repository model display is the base. Only the familiar near-top-down
    # camera convention is borrowed from the user's earlier GEVD RViz layout.
    shutil.copy2(SOURCE/'rviz/model.rviz',ROOT/'models/repository_model.rviz')
    config={'Panels':[{'Class':'rviz/Displays','Name':'Displays'}],
            'Visualization Manager':{'Class':''}}
    vm=config['Visualization Manager'];vm['Global Options']={'Background Color':'20; 28; 38','Fixed Frame':'map','Frame Rate':30}
    vm['Displays']=[{'Class':'rviz/Map','Name':'Known field /map','Enabled':True,'Topic':'/map','Alpha':0.0,'Draw Behind':True},
                    {'Class':'rviz/MarkerArray','Name':'Figure 3 fixed pitch','Enabled':True,'Marker Topic':'/field_markings'},
                    {'Class':'rviz/MarkerArray','Name':'Nine-region planning overlay','Enabled':True,'Marker Topic':'/planning_graph'},
                    {'Class':'rviz/MarkerArray','Name':'Ball, labels and phases','Enabled':True,'Marker Topic':'/team_markers'}]
    for rid in IDS:
        color='240; 70; 70' if rid.startswith('red') else '70; 155; 255'
        vm['Displays'].append({'Class':'rviz/RobotModel','Name':rid+' NAO6','Enabled':True,'Robot Description':'/'+rid+'/robot_description','Visual Enabled':True,'Collision Enabled':False})
        vm['Displays'].append({'Class':'rviz/Pose','Name':rid+' loc_pose','Enabled':True,'Topic':'/loc_pose_'+rid,'Shape':'Arrow','Color':color,'Shaft Length':.38,'Shaft Radius':.022,'Head Length':.14,'Head Radius':.06})
        vm['Displays'].append({'Class':'rviz/Path','Name':rid+' path (hidden)','Enabled':False,'Topic':'/path_'+rid,'Color':color,'Line Style':'Lines','Line Width':.025,'Buffer Length':1})
    vm['Displays'].append({'Class':'rviz/TF','Name':'Robot frames (optional)','Enabled':False})
    vm['Views']={'Current':{'Class':'rviz/Orbit','Distance':14.6,'Pitch':1.5697963,'Yaw':-1.5707963,'Focal Point':{'X':4.5,'Y':2.8,'Z':0},'Target Frame':'map','Near Clip Distance':.01}}
    config['Window Geometry']={'Width':1800,'Height':990,'X':45,'Y':35,'Hide Left Dock':False,'Hide Right Dock':True}
    # JSON is valid YAML and can be loaded by RViz without a generator dependency.
    (ROOT/'rviz/nao6_global.rviz').write_text(json.dumps(config,indent=2))
    (ROOT/'models/adaptation.json').write_text(json.dumps({'repository':'https://github.com/build-100/Glasgow-TDP26-NAO6','source':'ros/nao6_description/urdf/nao6_collision_visual.urdf','sha256':hashlib.sha256(original.read_bytes()).hexdigest(),'changes':['Prefix every link, joint and mimic reference per robot','Red/blue visual colour only'],'preserved':['inertia','collision geometry','joint types and limits','camera/IMU frames','hip coupling'],'map_origin':'lower-left field corner, same as repository field.json','visual_frame':'map for all ROS messages and markers; internal scenario coordinates are centre-relative (+4.5,+3 offset)','motion':'scripted root poses, static inspection joint angles; no gait'},indent=2))
    print('Prepared eight namespaced NAO6 descriptions and live/replay launch files.')

if __name__=='__main__':main()
