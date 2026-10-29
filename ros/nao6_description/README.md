# NAO6 model and launch package

## Selected source

The source is [NAOH25V60.urdf](https://github.com/RajasJoshi/UoG_Robotics_TDP4/blob/8a0949302aac5d610339fa02f3e4cfc437abb17d/Wenao/protos/NAOH25V60.urdf).
Commit: `8a0949302aac5d610339fa02f3e4cfc437abb17d`.
Original SHA256: `327dce62542ecd1327acf3fa7c5ec7b59274d44ce6c09a9ade3417cc39f132d6`.

This is the selected public V6 adaptation source, not a manufacturer-certified
release or an already validated Gazebo motion package. The original XML is
retained as `urdf/NAOH25V60_source.urdf`; assets are under `meshes/`. Original
hashes/URLs and upstream licence are included. No Webots-only PROTO/motion files
or ROS 2 launch files are used.

## Supplied descriptions

| File | Use |
|---|---|
| `NAOH25V60_source.urdf` | Unmodified provenance copy; original absolute mesh paths are not portable |
| `nao6_full.urdf` | Portable package mesh paths; all original joints retained |
| `nao6_simplified.urdf` | Same model, with only hand actuators/finger/thumb joints fixed at zero |
| `nao6_collision_visual.urdf` | Simplified kinematics/inertia/contact model, with collision primitives used as visuals for resource-independent inspection |

The simplified model retains all links and original inertias/collisions. It
preserves both legs, hip mimic relationship, both arms including wrists, head
movement and camera/IMU frames. Fixing hands reduces independent moving joints
from 25 to 23; it does not freeze shoulder/elbow balance or recovery motion.
The full model remains available for controllers that expect hand joints.

The collision-visual variant is deliberately a coarse structure view. It
preserves original collision geometry and does not pretend to be the detailed
NAO surface. Use the mesh variant to inspect detailed appearance where the
target renderer supports the upstream OGRE `.mesh` resources. Mesh/material
rendering still needs an Ubuntu runtime check.

No inertia, leg dimensions, joint limits, foot friction, active camera/IMU
plugins, transmissions or walking controller is silently invented by preparation.
The URDF camera/IMU frames preserve geometry but do not themselves publish sensor
data. Gazebo Classic does not automatically guarantee dynamic hip mimic coupling
from a URDF mimic tag; the chosen controller/backend must enforce/validate it.

## Reproduce preparation and checks

From repository root, Python 3.8 or later:

```bash
python3 scripts/prepare_model.py
python3 scripts/check_model.py
python3 -m unittest discover -s tests -v
```

The generator checks the source hash, changes portable mesh paths, removes empty
texture elements, optionally fixes hand/finger joints, and writes a machine-readable
adaptation report. It preserves the original source. Generated models are included
so the package does not require regeneration just to inspect it.

## Ubuntu 20.04 / ROS Noetic setup

Assuming Noetic is already installed, install the required ROS components:

```bash
sudo apt install ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher-gui ros-noetic-rviz ros-noetic-gazebo-ros-pkgs
mkdir -p ~/catkin_ws/src
```

Place or symlink this `nao6_description` directory into `~/catkin_ws/src/`, then:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch nao6_description display.launch
```

`display.launch` defaults to the resource-independent collision-visual model.
To inspect the detailed mesh version or controller-compatible full description:

```bash
roslaunch nao6_description display.launch variant:=simplified
roslaunch nao6_description display.launch variant:=full
```

The joint sliders are for description/TF inspection, not walking validation.
The supplied initial angles are an inspection pose, not a calibrated balance pose.

## Gazebo inspection

```bash
roslaunch nao6_description gazebo_model.launch
```

This opens a local plane/ball scene and spawns the collision-visual description
with physics **paused**. It is a model-inspection launch, not a skill controller.
There is no stabilising gait: unpausing can let the robot fall. No fake joint
state publisher is connected to this physical scene. The ball uses provisional
dimensions/material values; validate actual dynamics before using it as evidence.

The next integration task is a validated NAO motion backend for STAND/STOP,
WALK, HEAD_LOOK, KICK_FORWARD and GET_UP. RECEIVE is a move/stop/wait/re-detect
composite. `ros_control` alone would not supply balanced motion. Prefer available
school software; firmware/controller compatibility remains unresolved.

The global RViz planning demo and local Gazebo skill demos can run independently.
Record actual states/outcomes and screen video. `rosbag play` replays messages;
it does not by itself reconstruct a Gazebo world or reproduce contact dynamics.

## Validation status

Local checks verify source/resource hashes, XML trees, retained inertia/contact
data, joint selection and referenced mesh existence. They are not a ROS/Gazebo
runtime test or proof of stable walking/kicking/recovery on NAO6 hardware.
