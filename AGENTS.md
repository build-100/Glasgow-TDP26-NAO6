# Repository scope

Keep this repository focused on NAO6 model resources, ROS launch/adaptation and
a minimal high-level planning interface. Do not add private research papers,
full strategy/reward designs or earlier team slide packs without a new request.

Target Ubuntu 20.04 / ROS Noetic / Gazebo Classic 11. Keep global RViz and local
Gazebo demonstrations independently runnable. Preserve leg/head/arm joints,
inertia, collision geometry and camera/IMU frames. Hand/finger and wrist-yaw
articulation is fixed by the optional simplified description; use the full
description if the selected controller requires those joints. Preserve all
shoulder/elbow, leg and head motion. Do not invent a gait controller.

There is no separate RECEIVE skill: wait for ball settlement, WALK/HEAD_LOOK
to approach and align, STOP, then KICK_FORWARD. No moving-ball interception or
trapping is required by this simplified team plan.

Preserve upstream model licences and provenance. Original Python code is covered
by the repository's restricted-use notice; imported/adapted model assets retain
their upstream terms. Label static checks separately from runtime validation.

The team leader authorised publication of the preserved V1/V2 global demo packages
on 2026-11-05. Keep them under demos/v1 and demos/v2, with their own launch/config
and evidence. V2 uses fixed routes and P=0.40, with at most one eligible contest
per round and selected seed 15. A turnover ends T; the counterattack begins at T+1.
Do not present either scripted edition as trained-policy or physical skill validation.
Both packages use the existing collision-visual NAO6 model and hide path displays.
Future changes must preserve the published recordings and separate edition evidence.

The team leader authorised the matching V3/V4 public demo editions on 2026-11-10.
Keep them under demos/v3 and demos/v4. Preserve V1/V2 and original recordings.
Publish selected action exports, common kinematic execution, model/launch resources,
videos, losslessly compressed bags, evidence summaries and portable English instructions.
Keep GEVD/private research snapshots, learned weights, training/reward implementations,
machine-specific log/parameter files and source course documents outside this repository.
V3/V4 source execution reproduces fixed policy-derived plans, not new online inference.
V4 recordings retain their historical V3-SG label; this is disclosed in its README.
