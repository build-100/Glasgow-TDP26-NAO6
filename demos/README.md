# NAO6 RViz demonstrations

Two preserved integration milestones for Ubuntu 20.04 / ROS Noetic.

| Edition | Scenario | Start here |
| --- | --- | --- |
| V1 | Red attack; blue and both keepers stationary | [V1 files, video and replay](v1/README.md) |
| V2 | Both teams move; one 40% contest opportunity per round; red counterattack after turnover | [V2 files, video and replay](v2/README.md) |

Each package includes source, eight namespaced NAO6 URDFs, launch/RViz files, a compressed ROS bag,
recorded replay video and validation reports. V2 uses fixed routes with a selected presentation seed;
neither edition is a learned-policy comparison. Global RViz root motion is kinematic, with static
inspection joint angles. Gazebo local skills and real-robot validation remain separate workstreams.

Both editions retain nine regions, shared local execution phases, ball settlement before approach,
unique robot poses/TF, fixed map, and hidden path displays. Public packages exclude private research,
reward/training implementations, source course documents and credentials.

See each edition for reproduction instructions and licence/provenance details.
