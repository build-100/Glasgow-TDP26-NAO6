#!/usr/bin/env python3
"""Version 2: fixed team routes and auditable, one-shot seeded contests."""
import copy
import math
import random

IDS=['red1','red2','red3','red4','blue1','blue2','blue3','blue4']
FIELD_PLAYERS=[r for r in IDS if not r.endswith('4')]
KICK_RANGE=2.0
CONTEST_PROBABILITY=.4
SHOWCASE_SEED=15  # Selected to show retained possession, then a turnover.
WALK_SPEED=.30
KEEPERS={'red4':(-4.25,-.95,0),'blue4':(4.25,.95,math.pi)}
def region(x,y):
    if not (-4.5<=x<4.5 and -3<=y<3):return -1
    return int((y+3)//2)*3+int((x+4.5)//3)
def centre(n):return (-3+3*(n%3),-2+2*(n//3))
def edges():
    return [(a,b) for a in range(9) for b in range(a+1,9)
            if abs(a//3-b//3)<=1 and abs(a%3-b%3)<=1]
def team(rid):return 'RED' if rid.startswith('red') else 'BLUE'
def angle_delta(a,b):return (b-a+math.pi)%(2*math.pi)-math.pi

class OneShotContest:
    """At most one eligible contest per round; no retry after either outcome."""
    def __init__(self,p=.4,seed=SHOWCASE_SEED):
        if not 0<=p<=1:raise ValueError('Contest probability must be in [0,1]')
        self.p=p;self.rng=random.Random(seed);self.used=set();self.events=[]
    def evaluate(self,round_id,step,owner,robots,ball,settled=True):
        if not settled or round_id in self.used:return None
        node=region(*ball)
        if node<0 or region(*robots[owner][:2])!=node:return None
        rivals=[r for r in FIELD_PLAYERS if team(r)!=team(owner) and region(*robots[r][:2])==node]
        if not rivals:return None
        defender=min(rivals,key=lambda r:math.dist(robots[r][:2],ball))
        self.used.add(round_id);u=self.rng.random();won=u<self.p
        event=dict(round=round_id,step=step,node=node,owner_before=owner,defender=defender,
                   owner_after=defender if won else owner,probability=self.p,draw=u,
                   turnover=won,attempt=1,locked=True)
        self.events.append(event);return event

class Scenario:
    def __init__(self,seed=SHOWCASE_SEED):
        self.phases=[];self.duration=0.;self.step=0;self.round=0;self.score=[0,0]
        self.contests=OneShotContest(seed=seed);self.last_contest=None
        self.robots={};self.ball=(0.,0.);self.owner='red1';self.actions={}
        self._setup(1,{'red1':(-2.4,-.4,0),'red2':(-.8,.7,0),'red3':(1.,-.7,0),
                       'blue1':(.8,.8,math.pi),'blue2':(2.8,1.8,math.pi),'blue3':(2.8,-1.8,math.pi)},(-2.,0.),'red1')
        self._attack('red1',{'red2':(-.5,.65),'red3':(1.,-.65),'blue1':(.6,.8),'blue2':(2.4,1.8),'blue3':(2.4,-1.8)})
        retained=self._attack('red2',{'red1':(-2.5,-.7),'red3':(1.3,-.65),'blue1':(.65,.65),'blue2':(2.2,1.6),'blue3':(2.2,-1.6)})
        if not retained:raise ValueError('Seed does not produce the first-round showcase. Use seed 15.')
        self._attack('red3',{'red1':(-1.1,-.7),'red2':(.2,.7),'blue1':(1.,.9),'blue2':(3.,1.5),'blue3':(3.,-1.5)})
        self._attack('red3',{'red1':(-.6,-.7),'red2':(.8,.7),'blue1':(1.4,.9),'blue2':(3.7,1.5),'blue3':(3.7,-1.5)})
        self._add('ROUND_1_COMPLETE',2.)
        # Independent showcase rounds: this setup is not an in-play pass.
        self._setup(2,{'red1':(-2.6,-1.6,0),'red2':(-2.9,.7,0),'red3':(.6,1.7,0),
                       'blue1':(.5,.2,math.pi),'blue2':(-.8,.65,math.pi),'blue3':(-.9,-.65,math.pi)},(0.,0.),'blue1')
        self._attack('blue1',{'blue2':(-.9,1.5),'blue3':(-1.,-.65),'red1':(-2.4,-1.7),'red2':(-2.65,.65),'red3':(.1,1.7)})
        retained=self._attack('blue3',{'blue1':(.2,-.7),'blue2':(-1.2,1.5),'red1':(-2.6,-1.5),'red2':(-2.65,.65),'red3':(-.1,1.7)})
        if retained or self.owner!='red2':raise ValueError('Seed does not produce the second-round turnover. Use seed 15.')
        self._attack('red2',{'red1':(-.4,-.9),'red3':(.7,1.6),'blue1':(.6,-.8),'blue2':(-.7,1.5),'blue3':(-1.3,-.5)})
        self._attack('red1',{'red2':(-1.2,.65),'red3':(1.2,.7),'blue1':(1.2,-1.3),'blue2':(1.5,1.7),'blue3':(-.6,-.7)})
        self._attack('red3',{'red1':(.3,-.65),'red2':(-.5,.65),'blue1':(2.8,-1.5),'blue2':(2.8,1.5),'blue3':(.9,-1.4)})
        self._attack('red3',{'red1':(.8,-.65),'red2':(.3,.65),'blue1':(3.6,-1.5),'blue2':(3.6,1.5),'blue3':(1.6,-1.2)})
        self._add('DEMO_COMPLETE',3.)

    def _setup(self,number,positions,ball,owner):
        self.round=number;self.robots=dict(positions,**KEEPERS);self.ball=ball
        self.owner=owner;self.last_contest=None;self.actions={}
        self._add('ROUND_SETUP_NOT_IN_PLAY',3.,setup_reset=True)

    def _add(self,label,seconds,targets=None,ball_end=None,setup_reset=False):
        end=copy.deepcopy(self.robots);end.update(targets or {})
        self.phases.append(dict(label=label,start=self.duration,duration=seconds,step=self.step,
            round=self.round,owner=self.owner,before=copy.deepcopy(self.robots),after=end,
            ball_before=self.ball,ball_after=self.ball if ball_end is None else ball_end,
            score=list(self.score),actions=copy.deepcopy(self.actions),contest=copy.deepcopy(self.last_contest),
            setup_reset=setup_reset))
        self.duration+=seconds;self.robots=end;self.ball=self.phases[-1]['ball_after']

    def _attack(self,owner,support):
        self.step+=1;self.owner=owner;direction=1 if team(owner)=='RED' else -1
        desired={r:(x,y,0 if team(r)=='RED' else math.pi) for r,(x,y) in support.items()}
        desired[owner]=(self.ball[0]-.28*direction,self.ball[1],0 if direction==1 else math.pi)
        self.actions={}
        for r in FIELD_PLAYERS:
            a=region(*self.robots[r][:2]);b=region(*desired.get(r,self.robots[r])[:2])
            if a!=b and tuple(sorted([a,b])) not in edges():raise ValueError('Non-adjacent move: '+r+': '+str((a,b)))
            self.actions[r]=dict(from_node=a,to_node=b,action='HOLD_REGION' if a==b else 'MOVE_TO_NEIGHBOUR')
        self._add('HEAD_LOOK_SETTLED_BALL',.6)
        facing={}
        for r,end in desired.items():
            a=self.robots[r];dx,dy=end[0]-a[0],end[1]-a[1]
            facing[r]=(a[0],a[1],math.atan2(dy,dx) if math.hypot(dx,dy)>1e-6 else a[2])
        self._add('ORIENT_TO_WALK',.5,facing)
        move={r:(end[0],end[1],self.robots[r][2]) for r,end in desired.items()}
        seconds=max(1.,max(math.dist(self.robots[r][:2],end[:2]) for r,end in move.items())/WALK_SPEED)
        self._add('WALK_ATTACK_AND_DEFENCE',seconds,move)
        self._add('ALIGN',.7,desired);self._add('STOP',.5)
        contest=self.contests.evaluate(self.round,self.step,owner,self.robots,self.ball)
        if contest:
            self.last_contest=contest
            if contest['turnover']:
                self.owner=contest['owner_after'];self._add('TURNOVER_ENDS_T',2.5)
                return False  # Next _attack starts T+1; this step cannot kick.
            self._add('CONTEST_RETAINED_NO_RETRY',2.)
        self._add('KICK_FORWARD',.6)
        landing=(self.ball[0]+direction*KICK_RANGE,self.ball[1]);goal=abs(landing[0])>=4.5
        if goal:landing=(4.55*direction,self.ball[1])
        self._add('BALL_ROLLING_ALL_HOLD',1.8,ball_end=landing)
        if goal:
            self.score[0 if direction==1 else 1]+=1;self._add('GOAL_'+team(owner),2.)
        else:self._add('BALL_SETTLED',.7)
        return True

    def sample(self,t):
        t=max(0.,min(t,self.duration));p=self.phases[-1]
        for candidate in self.phases:
            if t<candidate['start']+candidate['duration']:p=candidate;break
        u=min(1.,max(0.,(t-p['start'])/p['duration']))
        positions={r:(a[0]+u*(p['after'][r][0]-a[0]),a[1]+u*(p['after'][r][1]-a[1]),
                      a[2]+u*angle_delta(a[2],p['after'][r][2])) for r,a in p['before'].items()}
        rolling=p['label']=='BALL_ROLLING_ALL_HOLD';v=u*(2-u) if rolling else u
        ball=tuple(a+v*(b-a) for a,b in zip(p['ball_before'],p['ball_after']))
        distance=sum(min(1.,max(0.,(t-q['start'])/q['duration']))*
                     sum(math.dist(q['before'][r][:2],q['after'][r][:2]) for r in FIELD_PLAYERS) for q in self.phases)
        roles={r:('GK_HOLD' if r.endswith('4') else 'BALL_OWNER' if r==p['owner'] else
                  'SUPPORT' if team(r)==team(p['owner']) else 'DEFEND') for r in IDS}
        return dict(version=2,time=round(t,3),duration=self.duration,round=p['round'],step=p['step'],
                    phase=p['label'],owner=p['owner'],possession=team(p['owner']),robots=positions,
                    targets=p['after'],roles=roles,actions=p['actions'],ball=ball,
                    coordinates_frame='field_center',ball_region=region(*ball),
                    robot_regions={r:region(*xy[:2]) for r,xy in positions.items()},
                    ball_state='ROLLING' if rolling else 'SETTLED',score=p['score'],distance_m=round(distance,3),
                    contest=p['contest'],contest_probability=.4,setup_reset=p['setup_reset'],seed=SHOWCASE_SEED,
                    source='fixed-route seeded showcase; kinematics only; no learned policy or physical skills')
