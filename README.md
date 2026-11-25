# Glasgow-TDP26-NAO6

Group repository for the University of Glasgow TDP course project.

For a single robot that accepts walking, kicking and recovery commands, start with
the [NAO6 whole-body motion workbench](ros/nao6_description/control/whole_body/README.md).
It pairs the selected V6 joint description with the Glasgow project's Webots body and motion references.
The repository also contains ROS launch files, a minimal planning scaffold and four global RViz demonstrations.
Private research and learned strategy/reward implementations are not included.

| Folder | Contents |
|---|---|
| `ros/nao6_description/` | Source and simplified URDFs, Webots skill controller, ROS launch files and RViz configuration |
| `scripts/` | Reproducible model preparation and structural checks |
| `src/nao_football/` | Basic graph and shared skill-interface scaffold |
| `configs/` | Field geometry |
| `tests/` | Structural/model and graph tests |
| [`demos/`](demos/README.md) | V1–V4 demo execution source, models, launch files, videos, ROS bags and validation reports |

Start with [model setup and launch instructions](ros/nao6_description/README.md).
The team presentation uses Ubuntu 20.04, ROS Noetic and RViz. Webots R2023b runs the independent local motion trials.

## Demonstrations

[Watch the 38-second NAO6 whole-body demo](demos/whole_body/NAO6_Whole_Body_Demo.mp4) for forward walking, left and right turns, a fixed right-foot kick and recovery from two fallen positions. The [chapter guide](demos/whole_body/README.md) links each filmed trial to the model, controls and measured results.

- [V1: initial global integration demo](demos/v1/README.md): red attack with stationary blue players.
- [V2: two-team fixed-route demo](demos/v2/README.md): both teams move, a one-shot 40% contest rule,
  possession turnover and a counterattack. A selected seed reproduces the presentation story.
- [V3: policy-derived match with independent greedy](demos/v3/README.md): first-to-three, fixed inactive
  keepers and animated kinematic skills; the published package executes a selected match export.
- [V4: sequential marginal-gain baseline improvement](demos/v4/README.md): stronger coordinated heuristic,
  frozen-policy comparison followed by cooperative-only adaptation, and a winner-neutral 3:1 showcase.

All four folders include English reproduction instructions, recorded replay videos and losslessly compressed
ROS bags. V3/V4 bags are split into manageable parts; their restore scripts verify and reconstruct the original bag. Download/decompress the bag and use the matching replay launch/configuration.

**Validation status:** The recorded Ubuntu 20.04 / ROS Noetic runs verified eight RViz RobotModels,
unique pose streams, fixed map, TF chains and matching recorded/replayed messages. Scenario regression
checks also pass. V1/V2 are fixed-route demonstrations with inspection joint poses. V3/V4 execute frozen exports from local
policy experiments and add illustrative joint animation; method-level results and limits are documented
separately. The public packages contain no learned weights or training/reward implementation and do not
perform new policy inference. On Ubuntu 20.04, the separate one-robot Webots workbench passed
forward walking, a fixed right-foot kick and both fallen-start recovery trials. ROS Noetic also
returned measured joint positions and the robot's camera transform. See the
[measured results and test conditions](ros/nao6_description/control/whole_body/README.md#measured-results).
Physical NAO6 validation remains open.

Original project material is intended for use only by team members authorised
by the repository owner: see [LICENSE.txt](LICENSE.txt). Third-party models and
their derivatives retain upstream terms, including the model package's
[upstream licence](ros/nao6_description/LICENSE.upstream). Public access cannot
prevent viewing, downloading or GitHub forks, and does not grant a general
reuse licence for original project material.
