#!/usr/bin/env python3
"""Audit actual recorded states, close contests and the first-to-three finish."""
import json
import math
from pathlib import Path
import sys
import rosbag
from scenario import IDS

bag,out=map(Path,sys.argv[1:])
states=[];contests=[];skills=[];joint_variants={r:set() for r in IDS}
with rosbag.Bag(str(bag)) as stream:
    for topic,msg,_ in stream.read_messages():
        if topic=='/demo/state':states.append(json.loads(msg.data))
        elif topic=='/demo/contests':contests.append(json.loads(msg.data))
        elif topic=='/demo/skills':skills.append(json.loads(msg.data))
        elif topic.startswith('/joint_states_'):
            rid=topic[len('/joint_states_'):]
            if rid in joint_variants:joint_variants[rid].add(tuple(round(v,5) for v in msg.position))
assert states,'No actual state messages recorded.'
assert max(states[-1]['score'])==3,'Match did not end at three.'
assert states[-1]['phase']=='MATCH_COMPLETE_FIRST_TO_THREE'
assert all(max(s['score'])<=3 for s in states)
assert all(all(a<=b for a,b in zip(s0['score'],s1['score'])) for s0,s1 in zip(states,states[1:]))
for r in ['red4','blue4']:
    assert len({tuple(s['robots'][r]) for s in states})==1,'Keeper moved.'
    assert len(joint_variants[r])==1,'Keeper joint animation is forbidden.'
assert all(len(joint_variants[r])>1 for r in IDS if not r.endswith('4'))
distances={r:0. for r in IDS if not r.endswith('4')}
for s0,s1 in zip(states,states[1:]):
    if s0['round']==s1['round'] and not s1['setup_reset']:
        for r in distances:distances[r]+=math.dist(s0['robots'][r][:2],s1['robots'][r][:2])
assert all(d>.1 for d in distances.values()),'A field player never moved in play.'
assert len({c['round'] for c in contests})==len(contests),'Repeated contest within a round.'
for c in contests:
    assert c['owner_ball_distance']<=.500001 and c['challenger_ball_distance']<=.500001
    assert c['robot_distance']<=.900001
    assert c['turnover']==(c['draw']<.4)
    if c['turnover']:
        assert not any(s['round']==c['round'] and s['step']==c['step'] and s['phase']=='KICK_FORWARD' for s in states)
for s0,s1 in zip(states,states[1:]):
    if s0['phase']==s1['phase']=='BALL_ROLLING_ALL_HOLD':
        assert all(math.dist(s0['robots'][r][:2],s1['robots'][r][:2])<1e-9 for r in IDS)
assert skills,'No explicit completion feedback recorded.'
result=dict(passed=True,states=len(states),contests=contests,final_score=states[-1]['score'],
    goals_to_win=3,goalkeepers_fixed_and_inactive=True,all_field_players_have_joint_animation=True,
    ball_rolling_players_hold=True,one_contest_per_goal_round=True,turnover_ends_T=True,
    skill_completion_messages=len(skills),motion='kinematic skill execution, not dynamics validation')
result['field_player_in_play_distance_m']=distances
out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
