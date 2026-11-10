# NAO6 RViz demonstrations

Four preserved integration milestones for Ubuntu 20.04 / ROS Noetic.

| Edition | Scenario and origin | Start here |
| --- | --- | --- |
| V1 | Initial red attack; blue and keepers stationary | [V1](v1/README.md) |
| V2 | Both teams move on fixed routes; selected one-shot 40% contests and counterattack | [V2](v2/README.md) |
| V3 | Selected VDN/Double-DQN versus independent-greedy match; first to three, 3:0 | [V3](v3/README.md) |
| V4 | Sequential marginal-gain baseline and adapted cooperative policy; winner-neutral showcase, 3:1 | [V4](v4/README.md) |

Each package provides models, ROS execution/replay source, launch/RViz configuration, compressed actual
recordings, video and validation reports. V1/V2 use fixed routes and inspection joint poses. V3/V4 provide
selected policy-derived action plans and illustrative joint animation, not public training or online
inference implementations. Their method-level comparison and evidence limitations are documented separately.

All retain eight namespaced NAO6 models, the fixed map, nine-region overlay and hidden path displays.
V3/V4 end when either side reaches three goals; both keepers stay fixed and have no gameplay effect.
No version validates physical humanoid balance, vision, kicking or autonomous hardware execution.

Use the README and restore script belonging to the same edition. V1/V2 use single `.bag.gz` files.
V3/V4 use lossless gzip parts plus SHA256 manifests; `prepare_recording.py` reconstructs the original bag.
V4's original recording retains its historical V3-SG label; the current live executor displays V4.

Private research, GEVD snapshots, learned weights, training/reward implementations, source course
documents, credentials and machine-specific log/parameter files are excluded. Recorded ROS decision
telemetry is retained as part of the authorised demonstration evidence. Original project material and
third-party model assets retain their separately declared licences.
