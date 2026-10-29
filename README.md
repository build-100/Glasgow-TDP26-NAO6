# Glasgow-TDP26-NAO6

Group repository for the University of Glasgow TDP course project.

This initial repository contains the selected robot model resources, ROS launch
files and a minimal planning scaffold. Detailed team strategy and research
implementations are not included.

| Folder | Contents |
|---|---|
| `ros/nao6_description/` | Source/adapted URDFs, mesh assets, RViz configuration and Gazebo model-inspection scene |
| `scripts/` | Reproducible model preparation and structural checks |
| `src/nao_football/` | Basic graph and shared skill-interface scaffold |
| `configs/` | Field geometry |
| `tests/` | Structural/model and graph tests |

Start with [model setup and launch instructions](ros/nao6_description/README.md).
The platform target is Ubuntu 20.04, ROS Noetic and Gazebo Classic 11.

**Validation status:** XML/model invariants and Python tests are checked locally.
ROS/Gazebo rendering, controller compatibility, balanced walking, kick and get-up
behaviour still require validation on the target Ubuntu installation.

Original project material is intended for use only by team members authorised
by the repository owner: see [LICENSE.txt](LICENSE.txt). Third-party models and
their derivatives retain upstream terms, including the model package's
[upstream licence](ros/nao6_description/LICENSE.upstream). Public access cannot
prevent viewing, downloading or GitHub forks, and does not grant a general
reuse licence for original project material.
