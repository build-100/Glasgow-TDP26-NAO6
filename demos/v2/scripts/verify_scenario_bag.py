#!/usr/bin/env python3
"""Check the requested V2 behaviours in actual recorded messages."""
import json
import math
from pathlib import Path
import sys
import rosbag
from scenario import FIELD_PLAYERS, IDS, region
states=[];events=[];contests=[]
with rosbag.Bag(sys.argv[1]) as bag:
    for topic,msg,_ in bag.read_messages(topics=['/demo/state','/demo/events','/demo/contests']):
        {'/demo/state':states,'/demo/events':events,'/demo/contests':contests}[topic].append(json.loads(msg.data))
assert states and events
assert len(contests)==2 and [c['turnover'] for c in contests]==[False,True]
assert all(c['probability']==.4 and c['turnover']==(c['draw']<.4) for c in contests)
turnovers=[s for s in events if s['phase']=='TURNOVER_ENDS_T']
goals=[s for s in events if s['phase']=='GOAL_RED']
assert len(turnovers)==1 and [s['round'] for s in goals]==[1,2]
assert states[-1]['score']==[2,0]
turn=turnovers[0];idx=events.index(turn)
assert events[idx+1]['step']==turn['step']+1
assert not any(s['phase']=='KICK_FORWARD' and s['step']==turn['step'] for s in events)
assert any(s['round']==2 and s['possession']=='BLUE' and s['phase']=='KICK_FORWARD' for s in states)
distance={str(round_id):{r:0. for r in FIELD_PLAYERS} for round_id in [1,2]}
for old,new in zip(states,states[1:]):
    for r in ['red4','blue4']:assert old['robots'][r]==new['robots'][r]
    if old['round']==new['round']:
        for r in FIELD_PLAYERS:distance[str(new['round'])][r]+=math.dist(old['robots'][r][:2],new['robots'][r][:2])
    if old['phase']==new['phase']=='BALL_ROLLING_ALL_HOLD':assert old['robots']==new['robots']
for values in distance.values():assert all(d>.3 for d in values.values())
for s in events:
    if s['phase'] in ['CONTEST_RETAINED_NO_RETRY','TURNOVER_ENDS_T']:
        c=s['contest'];node=region(*s['ball'])
        assert region(*s['robots'][c['owner_before']][:2])==node
        assert region(*s['robots'][c['defender']][:2])==node
result={'passed':True,'state_messages':len(states),'contest_decisions':contests,
        'turnovers':len(turnovers),'goals_by_round':[s['round'] for s in goals],
        'final_score':states[-1]['score'],'distance_m_by_round':distance,
        'keepers_static':True,'all_hold_while_ball_rolls':True,
        'turnover_ends_T_and_next_owner_starts_T_plus_1':True}
Path(sys.argv[2]).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
