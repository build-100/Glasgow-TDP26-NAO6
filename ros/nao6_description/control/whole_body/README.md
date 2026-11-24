# NAO6 whole-body motion

Start here to run one NAO on a floor with a ball. The global team display still uses ROS Noetic and RViz. The local actuator tests use Webots R2023b, the simulator used by the Glasgow reference project. Python 3.8 or newer is sufficient for preparation and offline checks.

## Run a trial

From the root of this team repository on a machine with Webots R2023b on its executable path:

```bash
python3 ros/nao6_description/control/whole_body/motion_workbench.py run --case walk
```

Replace `walk` with `kick`, `front` (prone) or `back` (supine) for the other three trials. Each starts in a new world. To open a world manually, use `prepare` in place of `run`. The command prints its path. Pass `--output NEW_EMPTY_FOLDER` to keep the trial at a chosen location. The prepared directory contains the robot, motion references, controller and textures and can be moved as a unit. An installed Webots binary can also be selected with `--webots /path/to/webots`.

On first use the workbench downloads only the pinned model and motion assets from the public upstream repository to a local cache and verifies their hashes. For an offline lab, use a prepared world, or give any command `--wenao /path/to/UoG_Robotics_TDP4/Wenao` from the pinned revision. Additional appearance files in a partial checkout are optional. The Webots PROTOs keep their original notices and are used with Webots.

## Which model runs

| Part | Team choice |
|---|---|
| Source description | [NAOH25V60.urdf at the pinned revision](https://github.com/RajasJoshi/UoG_Robotics_TDP4/blob/8a0949302aac5d610339fa02f3e4cfc437abb17d/Wenao/protos/NAOH25V60.urdf) |
| Joint contract | [nao6_simplified.urdf](../../urdf/nao6_simplified.urdf). Its 21 independent targets cover 11 leg coordinates, eight shoulder/elbow coordinates and two head coordinates. Both physical hip motors receive the left master target. The four wrist/hand channels in the kick file are omitted. |
| Simulated body | The upstream football world instantiates [NaoV6.proto](https://github.com/RajasJoshi/UoG_Robotics_TDP4/blob/8a0949302aac5d610339fa02f3e4cfc437abb17d/Wenao/protos/NaoV6.proto). Our world instantiates that same body with its wrist submodels, collision shapes, masses, joint motors, encoders, accelerometer and cameras. Its wrist and finger actuators hold zero. The original physical hinges remain present. |
| Controller | The upstream [motion catalogue](https://github.com/RajasJoshi/UoG_Robotics_TDP4/blob/8a0949302aac5d610339fa02f3e4cfc437abb17d/Wenao/controllers/Utils/Consts.py) and [player's motion dispatch and fall handling](https://github.com/RajasJoshi/UoG_Robotics_TDP4/blob/8a0949302aac5d610339fa02f3e4cfc437abb17d/Wenao/controllers/RedForwardA/RedForwardA.py) are the references. Our shared executor reads the actual timed clips and sends joint targets to Webots position servos. It checks encoders and torso posture before reporting completion. |
| Global display | [RViz demonstrations](../../../../demos/README.md) use their own global pose sources and recordings. The [right-knee study](../README.md) is a separate single-axis experiment. |

## Actions and acceptance

| Request | Source and adaptation | What to check |
|---|---|---|
| WALK | One `Forwards50` cycle, or a `TurnLeft10` / `TurnRight10` clip. Enter and return through `StandInit`. | For the forward trial the torso finishes upright and advances more than 0.02 m along its starting heading. Source clip names do not guarantee distance or turn angle. |
| KICK_FORWARD | A settled, aligned ball authorises one `RightShoot` with the right foot. A single reference is shared for passes and shots. | The kick trial checks ball placement and rest before kicking, joint completion, upright posture and more than 0.02 m of forward ball displacement. Range and strength require measurement. |
| GET_UP | Stable front/back orientation chooses `StandUpFromFront` or `StandUpFromBack`, followed by `StandInit`. Shoulders and elbows participate. | Each trial starts on the corresponding side. The torso must rise more than 0.10 m, finish above 0.22 m and remain upright. |
| STAND, STOP, HEAD_LOOK | Reference stand posture, interrupt and hold measured joint positions, or bounded head yaw/pitch. | STOP cancels the existing sequence. A side fall is reported without selecting a front/back recovery. |

The displacement/height thresholds and the trial ball are lab fixtures. A terminal skill result requires final encoder error at most 0.10 rad across driven joints, including the coupled follower, and stable upright posture for 0.20 s. STOP acknowledges a hold command immediately. In live mode a stable front/back fall interrupts locomotion and can launch one automatic recovery per fall. A timeout ends the skill and holds measured joint positions.

The adapter keeps sparse channels at their previous target, uses the intersection of URDF and Webots travel limits, and stretches segments as needed to remain below 85% of maximum joint speed. The simulated torque cap is the smaller of the URDF effort and Webots motor limits. The back-recovery reference contains 13 frames with inconsistent left/right coupled-hip commands (maximum difference 1.02 rad). The left master sets both motors. These changes can affect balance, contact and recovery success.

## ROS Noetic input and output

Build and source the catkin workspace containing `nao6_description`, then run the following with Webots and the ROS Python environment available. Keep this one-robot control test on a separate ROS master from a recorded match if both would publish the same robot topics.

```bash
roslaunch nao6_description whole_body.launch robot_id:=red1
rostopic echo /skill_result_red1
# In another terminal, after /posture_red1 reports upright:
rostopic pub -1 /skill_request_red1 std_msgs/String "data: '{\"robot_id\":\"red1\",\"sequence_id\":1,\"skill\":\"WALK\",\"primitive\":\"forward\",\"timeout_s\":20}'"
```

IDs are increasing integers for each running robot. Automatic recovery uses negative result IDs. WALK also accepts `left` or `right`. KICK_FORWARD requires a Boolean `ball_ready` from local behaviour. HEAD_LOOK accepts `yaw_rad` and `pitch_rad`. Scope `robot_id` to red1–red4 or blue1–blue4. The port publishes measured `/joint_states_<id>`, `/posture_<id>` and `/skill_result_<id>`. The region planner supplies a destination. Local navigation must use pose/ball feedback to select and repeat primitives and decide when the region has been reached.

## Evidence and reproduction

```bash
python3 ros/nao6_description/control/whole_body/motion_workbench.py inspect
python3 ros/nao6_description/control/whole_body/check_motion.py -v
```

Offline verification covered all seven pinned tracks, every adapted segment, actuator mapping, right-foot selection, front/back selection, STOP, timeout, and rejection of false completion with a stuck encoder or a torso still on the floor. Four physical trial worlds and the ROS world are generated by the same entry point. These checks use synthetic feedback. Each actual Webots trial writes `trial_result.json` with measured outcomes, event times and a pass flag. The ROS bridge, physical stability, contact and real NAO execution still require runtime trials on the target installation. An executed joint clip alone does not establish that the ball travelled or that the robot stood up.
