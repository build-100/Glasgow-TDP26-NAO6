# NAO6 whole-body motion

Start here to run one NAO on a floor with a ball. The global team display still uses ROS Noetic and RViz. The local actuator tests use Webots R2023b, the simulator used by the Glasgow reference project. Python 3.8 or newer is sufficient for preparation and offline checks.

## Run a trial

From the root of this team repository on a machine with Webots R2023b on its executable path:

```bash
python3 ros/nao6_description/control/whole_body/motion_workbench.py run --case walk
```

Replace `walk` with `turn_left`, `turn_right`, `kick`, `front` (prone) or `back` (supine). Each trial starts in a new world. To open a world manually, use `prepare` in place of `run`. The command prints its path. Pass `--output NEW_EMPTY_FOLDER` to keep the trial at a chosen location. The prepared directory contains the robot, motion references, controller and textures and can be moved as a unit. Select an installed Webots binary with `--webots /path/to/webots`.

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
| WALK | One `Forwards50` cycle, or a `TurnLeft10` / `TurnRight10` clip. Enter and return through `StandInit`. | The forward trial requires more than 0.02 m along the starting heading. Turning trials require a heading change of more than 2° in the requested direction. Each must finish upright. |
| KICK_FORWARD | A settled, aligned ball authorises one `RightShoot` with the right foot. Hip and ankle roll amplitudes are scaled to 55% of the reference during the kick to keep the torso over the support foot. Passes and shots share this reference. | The trial checks ball placement and rest before kicking, joint completion, upright posture and more than 0.02 m of forward ball displacement. |
| GET_UP | Filtered gravity selects the front or back clip. The executor enters through `StandInit`, holds the first recovery frame for 0.80 s and finishes in `StandInit`. In the front clip it holds the 2.50 s keyframe for another 0.45 s while support shifts from hands to feet. | The robot settles under gravity from a collision-free spawn. Recovery starts after the measured posture settles. The torso must rise more than 0.10 m, finish above 0.22 m and remain upright. |
| STAND, STOP, HEAD_LOOK | Reference stand posture, interrupt and hold measured joint positions, or bounded head yaw/pitch. | STOP cancels the existing sequence. A side fall is reported without selecting a front/back recovery. |

The displacement/height thresholds and the trial ball are lab fixtures. A terminal skill result requires final encoder error at most 0.10 rad across driven joints, including the coupled follower, and stable upright posture for 0.20 s. STOP acknowledges a hold command immediately. In live mode a stable front/back fall interrupts locomotion and can launch one automatic recovery per fall. A timeout ends the skill and holds measured joint positions.

The adapter holds sparse channels at their previous target, intersects URDF and Webots travel limits, and stretches segments as needed to stay within 85% of the smaller joint speed limit. Ordinary skills use the lower of URDF effort and the Webots motor rating. While lifting its body, the simulated robot uses the native Webots motor ratings. The ordinary caps resume as soon as recovery ends. These ratings describe the simulator. The back-recovery reference has 13 frames with inconsistent left/right coupled-hip commands (maximum difference 1.02 rad). The left master sets both targets.

## ROS Noetic input and output

Build and source the catkin workspace containing `nao6_description`, then run the following with Webots and the ROS Python environment available. Keep this one-robot control test on a separate ROS master from a recorded match if both would publish the same robot topics.

```bash
roslaunch nao6_description whole_body.launch robot_id:=red1
rostopic echo /skill_result_red1
# In another terminal, after /posture_red1 reports upright:
rostopic pub -1 /skill_request_red1 std_msgs/String "data: '{\"robot_id\":\"red1\",\"sequence_id\":1,\"skill\":\"WALK\",\"primitive\":\"forward\",\"timeout_s\":20}'"
```

IDs are increasing integers for each running robot. Automatic recovery uses negative result IDs. WALK also accepts `left` or `right`. KICK_FORWARD requires a Boolean `ball_ready` from local behaviour. HEAD_LOOK accepts `yaw_rad` and `pitch_rad`. Scope `robot_id` to red1–red4 or blue1–blue4. The port publishes measured `/joint_states_<id>`, `/posture_<id>` and `/skill_result_<id>`. The region planner supplies a destination. Local navigation must use pose/ball feedback to select and repeat primitives and decide when the region has been reached.

To observe automatic recovery, launch a fresh world with `start_posture:=front` or `start_posture:=back`. The onboard accelerometer chooses the action after startup. Monitor `/skill_result_red1` for a negative sequence ID and a terminal result. The robot state publisher consumes the encoder topic and broadcasts robot-prefixed TF frames, including `red1/base_link` to `red1/CameraTop`. Field localisation supplies the global base pose separately.

## Evidence and reproduction

```bash
python3 ros/nao6_description/control/whole_body/motion_workbench.py inspect
python3 ros/nao6_description/control/whole_body/check_motion.py -v
```

Offline checks cover all seven tracks, every adapted segment, actuator mapping, right-foot selection, front/back selection, posture filtering, STOP, timeout, and rejection of false completion with a stuck encoder or a horizontal torso. The simulated trials write a separate `trial_result.json` per world. The test supervisor measures body and ball displacement. The live ROS controller uses motor encoders, accelerometer feedback and the supplied skill request.

## Measured results

Ubuntu 20.04.6, ROS Noetic and Webots R2023b ran all six single-robot fixtures twice from a fresh world at a 16 ms physics step. All twelve runs passed and ended upright. Measurements, ROS events, source hashes and additional fallen-start checks are in the [validation record](validation.json).

| Trial | Measured outcome |
|---|---|
| Forward walk | Torso advanced **0.464 m** along its starting heading. |
| Left / right turn | Heading changed by **+4.82°** / **−7.42°**. Use updated pose feedback when chaining turns. |
| Right-foot kick | The stationary 0.07 kg, 0.10 m diameter ball advanced **1.018 m** by the scheduled observation 0.50 s after completion. The robot recovered its stance. |
| Prone recovery | Torso rose from **0.201 m to 0.321 m**. The controller reported success after **6.208 s** of simulation time. |
| Supine recovery | Torso rose from **0.058 m to 0.321 m**. The controller reported success after **6.608 s**. |

In the first implementation the kick displaced the ball but the robot fell sideways, and both recovery fixtures failed. The final comparison included five initial prone release heights between 0.28 m and 0.32 m. Adding the support-transfer hold improved completion from two of five to five of five. Three additional heights, 0.285 m, 0.305 m and 0.315 m, also passed after the timing was fixed. These trials vary the release height on one level surface.

Through `whole_body.launch`, the robot accepted STAND, HEAD_LOOK, WALK and STOP, published 22 encoder positions and supplied the head-camera TF chain. In separate front/back launches the accelerometer initiated GET_UP, which finished upright in both cases. A live upright sequence was also recorded and replayed on an isolated ROS master. The project still needs physical NAO6 trials to establish hardware balance and kick range.
