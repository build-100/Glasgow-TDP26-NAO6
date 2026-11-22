# NAO6 low-level control model

The whole-robot reference for this study is [nao6_simplified.urdf](../urdf/nao6_simplified.urdf), with 21 independent moving coordinates. The global RViz demo uses the collision-visual derivative. The knee joint geometry and limits match across both descriptions.

The [offline knee model](knee_model.py) represents one phase of a possible forward kick. The high-level planner assigns a pitch region. Local behaviour selects the skill and the motion layer supplies a joint reference.

## Reproduce the two trials

From this folder, run `python3 knee_model.py knee_results` with Python 3.8 or later. Both trials follow 0.62 to 0.28 rad. The second adds 0.36 N m of opposing output torque from 0.48 to 0.80 s. The script exports two CSVs and a JSON summary. Expected completion times from trial start are 1.030 s and 1.075 s.

## Model boundary

RKneePitch is a local +Z revolute joint between RHipPitch_link and RKneePitch_link. The H25600 datasheet (p. 6) assigns a Type 5 motor and a 130.85 reduction ratio. The 12 V motor reference, efficiency, effective load, control gains and timing thresholds are offline assumptions. The model tests joint tracking. Whole-body balance, foot contact, ball travel and hardware execution need separate tests.
