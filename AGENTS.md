# Repository scope

Keep this repository focused on NAO6 model resources, ROS launch/adaptation and
a minimal high-level planning interface. Do not add private research papers,
full strategy/reward designs or earlier team slide packs without a new request.

Use `ros/nao6_description/urdf/nao6_simplified.urdf` as the team's shared model for joints and frames. The ROS Noetic whole-body launch loads this URDF. Webots R2023b runs its matching Glasgow `NaoV6.proto` and motion clips. Preserve the leg, head and shoulder/elbow joints, coupled hips, inertia, contact geometry and camera/IMU frames. Hands and wrist yaw are fixed in the shared description. Keep the full URDF as a reference and the collision-visual derivative for the global RViz view.

Keep the team model in the ROS package. Put single-robot simulation evidence under `demos/whole_body/` as low-level control validation. Use V1–V4 for the historical global RViz demos.

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
