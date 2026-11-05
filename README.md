# Glasgow-TDP26-NAO6

Group repository for the University of Glasgow TDP course project.

This repository contains selected robot model resources, ROS launch files,
a minimal planning scaffold and two preserved global RViz demonstrations.
Private research and learned strategy/reward implementations are not included.

| Folder | Contents |
|---|---|
| `ros/nao6_description/` | Source/adapted URDFs, mesh assets, RViz configuration and Gazebo model-inspection scene |
| `scripts/` | Reproducible model preparation and structural checks |
| `src/nao_football/` | Basic graph and shared skill-interface scaffold |
| `configs/` | Field geometry |
| `tests/` | Structural/model and graph tests |
| [`demos/`](demos/README.md) | V1 and V2 source, models, launch files, videos, ROS bags and validation reports |

Start with [model setup and launch instructions](ros/nao6_description/README.md).
The platform target is Ubuntu 20.04, ROS Noetic and Gazebo Classic 11.

## Demonstrations

- [V1: initial global integration demo](demos/v1/README.md): red attack with stationary blue players.
- [V2: two-team fixed-route demo](demos/v2/README.md): both teams move, a one-shot 40% contest rule,
  possession turnover and a counterattack. A selected seed reproduces the presentation story.

Both folders include English reproduction instructions, recorded replay videos and losslessly compressed
ROS bags. Download/decompress the bag and use the matching replay launch/configuration.

**Validation status:** The recorded Ubuntu 20.04 / ROS Noetic runs verified eight RViz RobotModels,
unique pose streams, fixed map, TF chains and matching recorded/replayed messages. Scenario regression
checks also pass. These are scripted kinematic demonstrations with static inspection joint angles;
they do not establish a learned-policy advantage or physical gait. Gazebo skill execution, controller
compatibility, balanced walking, kicking, get-up behaviour and real-robot validation remain outstanding.

Original project material is intended for use only by team members authorised
by the repository owner: see [LICENSE.txt](LICENSE.txt). Third-party models and
their derivatives retain upstream terms, including the model package's
[upstream licence](ros/nao6_description/LICENSE.upstream). Public access cannot
prevent viewing, downloading or GitHub forks, and does not grant a general
reuse licence for original project material.
