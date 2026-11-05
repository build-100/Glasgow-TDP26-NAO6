#!/usr/bin/env python3
"""Deterministic high-level integration scenario; no learned policy or physics."""
import copy
import math

IDS = ['red1', 'red2', 'red3', 'red4', 'blue1', 'blue2', 'blue3', 'blue4']
INITIAL = [(-2.5,-0.7,0),(-0.6,1.4,0),(1.9,-1.6,0),(-4.2,0,0),
           (0.8,1.8,math.pi),(3,2,math.pi),(3,-2,math.pi),(4.2,0.9,math.pi)]
KICK_RANGE = 2.0
def region(x,y):
    if not (-4.5<=x<4.5 and -3<=y<3): return -1
    return int((y+3)//2)*3+int((x+4.5)//3)
def edges():
    return [(a,b) for a in range(9) for b in range(a+1,9)
            if abs(a//3-b//3)<=1 and abs(a%3-b%3)<=1]
def centre(n): return (-3+3*(n%3),-2+2*(n//3))

class Scenario:
    def __init__(self):
        self.phases=[]; self.duration=0.0
        robots=dict(zip(IDS,INITIAL)); ball=(-2.0,0.0)
        def add(label,seconds,step,owner,targets=None,ball_end=None):
            nonlocal robots,ball
            end=copy.deepcopy(robots); end.update(targets or {})
            self.phases.append(dict(label=label,start=self.duration,duration=seconds,
                step=step,owner=owner,before=copy.deepcopy(robots),after=end,
                ball_before=ball,ball_after=ball_end or ball))
            self.duration+=seconds; robots=end; ball=ball_end or ball
        add('SUPPORT_POSITIONING',3.0,1,'red1',{'red2':(-0.4,1.0,0),'red3':(1.9,-0.9,0)})
        for step,owner in enumerate(['red1','red2','red3','red3'],1):
            add('OBSERVE_SETTLED_BALL',0.8,step,owner)
            target=(ball[0]-0.23,ball[1],0)
            distance=math.hypot(target[0]-robots[owner][0],target[1]-robots[owner][1])
            add('APPROACH',max(2.0,distance/0.30),step,owner,{owner:target})
            add('ALIGN',1.0,step,owner)
            add('STOP',0.8,step,owner)
            add('KICK_FORWARD',0.5,step,owner)
            # Same nominal impulse/range for every kick; the final shot is
            # terminated at the goal line instead of simulating behind the goal.
            end=(min(ball[0]+KICK_RANGE,4.55),ball[1])
            add('BALL_ROLLING',2.5,step,owner,ball_end=end)
            add('GOAL' if step==4 else 'BALL_SETTLED',1.0,step,owner)
        add('COMPLETE',3.0,4,'red3')

    def sample(self,t):
        t=max(0.0,min(t,self.duration))
        phase=self.phases[-1]
        for p in self.phases:
            if t<p['start']+p['duration']:
                phase=p;break
        u=min(1.0,max(0.0,(t-phase['start'])/phase['duration']))
        positions={}
        for rid in IDS:
            a,b=phase['before'][rid],phase['after'][rid]
            x,y=a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1])
            yaw=math.atan2(b[1]-a[1],b[0]-a[0]) if a[:2]!=b[:2] and u<1 else b[2]
            # After approaching, ALIGN rotates continuously towards the kick.
            if phase['label']=='ALIGN' and rid==phase['owner']:
                prev=self.phases[self.phases.index(phase)-1]
                pa,pb=prev['before'][rid],prev['after'][rid]
                old=math.atan2(pb[1]-pa[1],pb[0]-pa[0]) if pa[:2]!=pb[:2] else 0
                yaw=old*(1-u)
            positions[rid]=(x,y,yaw)
        v=u*(2-u) if phase['label']=='BALL_ROLLING' else u
        ball=tuple(a+v*(b-a) for a,b in zip(phase['ball_before'],phase['ball_after']))
        distance=0.0
        for p in self.phases:
            k=min(1,max(0,(t-p['start'])/p['duration']))
            distance+=sum(k*math.hypot(p['after'][r][0]-p['before'][r][0],p['after'][r][1]-p['before'][r][1]) for r in IDS)
        return dict(time=round(t,3),duration=self.duration,step=phase['step'],phase=phase['label'],
                    owner=phase['owner'],robots=positions,targets=phase['after'],ball=ball,
                    coordinates_frame='field_center',ball_region=region(*ball),
                    robot_regions={r:region(p[0],p[1]) for r,p in positions.items()},
                    ball_state='ROLLING' if phase['label']=='BALL_ROLLING' else 'SETTLED',
                    score=[int(ball[0]>=4.5),0],distance_m=round(distance,3),
                    source='scripted high-level kinematics; no perception or gait controller')
