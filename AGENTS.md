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
