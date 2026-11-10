#!/usr/bin/env python3
"""Execute exported policy rollouts through the shared kinematic skill contract."""
import copy
import json
import math
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
from demo_geometry import ALL_IDS as IDS, IDS as FIELD_PLAYERS, centre, edges, region
from skills import walk_sample

def angle_delta(a,b):return (b-a+math.pi)%(2*math.pi)-math.pi

class Scenario:
    def __init__(self,path=None):
        self.plan=json.loads(Path(path or ROOT/'plans/demo_plan.json').read_text(encoding='utf-8'))
        self.phases=[];self.duration=0.;self.score=[0,0];self.last_contest=None
        self.contest_events=[];self.step=0;self.actions={};self.round=0;self.actor=None
        self.possession='RED';self.owner=None;self.seed=None;self.feedback=[];self.utilities=None
        for number,episode in enumerate(self.plan['episodes'],1):
            initial=episode['initial'];self.round=number;self.seed=initial['seed'];self.step=0
            self.last_contest=None;self.feedback=[];self.actions={};self.actor=None;self.utilities=None;self.baseline=None
            direction=initial['red_direction']
            self.robots={rid:(*initial['robots'][i],0. if (1 if i<3 else -1)*direction>0 else math.pi)
                         for i,rid in enumerate(FIELD_PLAYERS)}
            self.robots.update(red4=(-4.25*direction,-.95,0 if direction>0 else math.pi),
                               blue4=(4.25*direction,.95,math.pi if direction>0 else 0))
            self.ball=list(initial['ball']);self.owner=FIELD_PLAYERS[initial['owner']]
            self.possession='RED' if initial['possession_team']==0 else 'BLUE'
            self._add('ROUND_SETUP_NOT_IN_PLAY',2.5,setup=True)
            self._add('STAND',.6)
            for transition in episode['transitions']:
                self._transition(transition)
            self._add('ROUND_COMPLETE',1.5)
        if max(self.score)!=3:raise ValueError('The match must end at three goals.')
        self._add('MATCH_COMPLETE_FIRST_TO_THREE',3.0)

    def _add(self,label,seconds,targets=None,ball_end=None,setup=False):
        end=copy.deepcopy(self.robots);end.update(targets or {})
        self.phases.append(dict(label=label,start=self.duration,duration=max(.01,seconds),
            round=self.round,step=self.step,seed=self.seed,owner=self.owner,possession=self.possession,
            actor=self.actor,before=copy.deepcopy(self.robots),after=end,
            ball_before=list(self.ball),ball_after=list(self.ball if ball_end is None else ball_end),
            score=list(self.score),contest=copy.deepcopy(self.last_contest),
            actions=copy.deepcopy(self.actions),feedback=copy.deepcopy(self.feedback),setup_reset=setup))
        self.phases[-1]['utilities']=copy.deepcopy(self.utilities)
        self.phases[-1]['baseline_allocation']=copy.deepcopy(self.baseline)
        self.duration+=max(.01,seconds);self.robots=end
        if ball_end is not None:self.ball=list(ball_end)

    def _transition(self,tr):
        self.step=tr['step'];self.feedback=[];self.actor=None;self.utilities=tr.get('policy_utilities')
        self.baseline=tr.get('baseline_allocation')
        self.actions={m['robot']:dict(from_node=m['from_node'],to_node=m['to_node'],action=m['action']) for m in tr['execution']}
        self._add('HEAD_LOOK_SETTLED_BALL',.45)
        targets={m['robot']:m['actual'] for m in tr['execution']}
        orient={}
        for rid,xy in targets.items():
            old=self.robots[rid];delta=np.array(xy)-old[:2]
            yaw=math.atan2(delta[1],delta[0]) if np.linalg.norm(delta)>1e-6 else old[2]
            orient[rid]=(old[0],old[1],yaw)
        self._add('ORIENT_TO_WALK',.35,orient)
        movement={rid:(*xy,self.robots[rid][2]) for rid,xy in targets.items()}
        walk=max(m['distance'] for m in tr['execution'])/.30
        self._add('WALK_ATTACK_AND_DEFENCE',max(.15,walk),movement)
        align={rid:(a[0],a[1],math.atan2(self.ball[1]-a[1],self.ball[0]-a[0])) for rid,a in self.robots.items() if rid in FIELD_PLAYERS}
        self._add('ALIGN',.35,align)
        self._add('STOP',.25)
        for ev in tr['events']:
            kind=ev['type']
            if kind=='BALL_CONTROL_ACQUIRED':
                self.owner=ev['robot'];self.possession='RED' if self.owner.startswith('red') else 'BLUE'
            elif kind=='COMPLETED_PASS':
                self._add('COMPLETED_PASS',.5)
            elif kind=='BALL_SETTLED_REDETECT':
                self._add('HEAD_LOOK_SETTLED_BALL',.25)
            elif kind=='CONTEST':
                self.last_contest=dict(ev,round=self.round)
                self.contest_events.append(self.last_contest)
                self.owner=ev['owner_after'];self.possession='RED' if self.owner.startswith('red') else 'BLUE'
                self._add('TURNOVER_ENDS_T' if ev['turnover'] else 'CONTEST_RETAINED_NO_RETRY',2.0)
            elif kind=='KICK_ALIGNMENT':
                self.actor=ev['robot'];self.owner=self.actor
                self._add('WALK_ALIGN',max(.15,math.dist(ev['start'],ev['end'])/.30),
                          {self.actor:(*ev['end'],ev['theta'])})
                self._add('STOP',.2)
            elif kind=='KICK':
                self.actor=ev['robot'];self._add('KICK_FORWARD',.8)
                self.owner=None
                ratio=min(1.,ev['travelled']/ev['nominal_range'])
                roll_time=1.8*(1-math.sqrt(max(0.,1-ratio)))
                self._add('BALL_ROLLING_ALL_HOLD',roll_time,ball_end=ev['end'])
                self.phases[-1]['nominal_roll_time']=1.8
                self.phases[-1]['nominal_range']=ev['nominal_range']
                self._add('BALL_SETTLED',.45)
            elif kind=='GOAL':
                self.score[ev['team']]+=1
                self._add('GOAL_RED' if ev['team']==0 else 'GOAL_BLUE',2.0)
        self.owner=None if tr['after']['owner'] is None else FIELD_PLAYERS[tr['after']['owner']]
        self.possession='RED' if tr['after']['possession_team']==0 else 'BLUE'
        self.feedback=copy.deepcopy(tr['execution'])
        self._add('MACRO_STEP_COMPLETE',.15)
        for i,rid in enumerate(FIELD_PLAYERS):
            if np.linalg.norm(np.array(self.robots[rid][:2])-tr['after']['robots'][i])>1e-6:
                raise ValueError('Execution position differs from computed rollout: '+rid)
        if np.linalg.norm(np.array(self.ball)-tr['after']['ball'])>1e-6:raise ValueError('Ball teleport between phases.')

    def sample(self,t):
        t=max(0.,min(t,self.duration));p=self.phases[-1]
        for candidate in self.phases:
            if t<candidate['start']+candidate['duration']:p=candidate;break
        elapsed=max(0.,t-p['start']);u=min(1.,elapsed/p['duration'])
        positions={};moving={};statuses={}
        for rid,start in p['before'].items():
            end=p['after'][rid]
            if p['label'] in ('WALK_ATTACK_AND_DEFENCE','WALK_ALIGN'):
                xy,status=walk_sample(start[:2],end[:2],elapsed)
                moving[rid]=status=='RUNNING';statuses[rid]=status
            else:
                xy=np.array(start[:2])+u*(np.array(end[:2])-start[:2]);moving[rid]=False;statuses[rid]='RUNNING' if u<1 else 'SUCCEEDED'
            positions[rid]=(float(xy[0]),float(xy[1]),start[2]+u*angle_delta(start[2],end[2]))
        rolling=p['label']=='BALL_ROLLING_ALL_HOLD';v=u
        if rolling:
            full_time=p.get('nominal_roll_time',1.8)
            fraction=min(1.,elapsed/full_time)
            distance=math.dist(p['ball_before'],p['ball_after'])
            v=min(1.,fraction*(2-fraction)*p.get('nominal_range',2.)/max(distance,1e-9))
        ball=[a+v*(b-a) for a,b in zip(p['ball_before'],p['ball_after'])]
        roles={rid:('GK_HOLD' if rid.endswith('4') else 'BALL_OWNER' if rid==p['owner'] else
                    'SUPPORT' if rid[:3].upper()==p['possession'][:3] else 'DEFEND') for rid in IDS}
        return dict(version=4,time=round(t,3),duration=self.duration,round=p['round'],step=p['step'],
            seed=p['seed'],phase=p['label'],owner=p['owner'],actor=p['actor'],possession=p['possession'],
            robots=positions,targets=p['after'],roles=roles,actions=p['actions'],ball=ball,
            ball_region=region(*ball),robot_regions={r:region(*xy[:2]) for r,xy in positions.items()},
            ball_state='ROLLING' if rolling else 'SETTLED',score=p['score'],contest=p['contest'],
            contest_probability=.4,setup_reset=p['setup_reset'],skill_feedback=p['feedback'],
            skill_status=statuses,moving=moving,phase_elapsed=elapsed,phase_progress=u,
            policy_utilities=p['utilities'],baseline_allocation=p['baseline_allocation'],goals_to_win=3,
            coordinates_frame='field_center',source='VDN+Double DQN vs sequential marginal-gain greedy; common kinematic skills; winner-neutral showcase selection')
