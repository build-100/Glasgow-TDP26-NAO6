"""Common bounded kinematic backend; interface for later physical controllers.

Pose and joint outputs are demonstration animation, not a validated NAO gait.
Completion, timeout and fall pre-emption are explicit; no RECEIVE skill exists.
"""
import math
import numpy as np

SKILLS = ('STAND','STOP','WALK','HEAD_LOOK','KICK_FORWARD','GET_UP')

def walk_sample(start, target, elapsed, speed=.30, timeout=20., fallen=False):
    start=np.asarray(start,dtype=float);target=np.asarray(target,dtype=float)
    if fallen:return start.copy(),'FALL_DETECTED_GET_UP_REQUIRED'
    distance=float(np.linalg.norm(target-start))
    if distance<1e-8:return target.copy(),'SUCCEEDED'
    reached=min(max(elapsed,0.),timeout)*speed
    if reached>=distance:return target.copy(),'SUCCEEDED'
    point=start+(target-start)*(reached/distance)
    return point,('TIMEOUT' if elapsed>=timeout else 'RUNNING')

def joint_pose(phase,elapsed,progress,is_actor=False,moving=False):
    q={'LShoulderPitch':1.4,'RShoulderPitch':1.4,'LShoulderRoll':.20,'RShoulderRoll':-.20,
       'LElbowRoll':-.4,'RElbowRoll':.4,'LHipPitch':-.05,'RHipPitch':-.05,
       'LKneePitch':.1,'RKneePitch':.1,'LAnklePitch':-.05,'RAnklePitch':-.05}
    if phase in ('WALK_ATTACK_AND_DEFENCE','WALK_ALIGN') and moving:
        s=math.sin(2*math.pi*1.4*elapsed)
        q.update(LHipPitch=-.05+.16*s,RHipPitch=-.05-.16*s,
                 LKneePitch=.10+.18*max(0.,-s),RKneePitch=.10+.18*max(0.,s),
                 LAnklePitch=-.05-.08*s,RAnklePitch=-.05+.08*s,
                 LShoulderPitch=1.4-.18*s,RShoulderPitch=1.4+.18*s)
    elif phase=='HEAD_LOOK_SETTLED_BALL':
        q['HeadYaw']=.28*math.sin(2*math.pi*progress)
        q['HeadPitch']=.12*math.sin(math.pi*progress)
    elif phase=='KICK_FORWARD' and is_actor:
        # Illustrative right-foot joint trajectory only; no dynamics claim.
        s=math.sin(math.pi*progress)
        q.update(RHipPitch=-.05-.38*s,RKneePitch=.1+.15*s,RAnklePitch=-.05+.14*s,
                 LShoulderRoll=.20+.08*s,RShoulderRoll=-.20-.08*s)
    return q
