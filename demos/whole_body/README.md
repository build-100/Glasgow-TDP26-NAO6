# Low-level control demonstration on the team NAO6 model

[Watch or download the video (MP4, 38 s)](NAO6_Whole_Body_Demo.mp4). Six independent Webots R2023b trials show one NAO6 on a level floor. Each trial ends with the robot upright. A separate camera recorded the scene.

| Start | Action | Measured result |
|---|---|---|
| 0:00.0 | Walk forwards | The torso advanced 0.464 m. |
| 0:09.5 | Turn left | Heading changed by +4.82°. |
| 0:13.5 | Turn right | Heading changed by −7.42°. |
| 0:16.8 | Kick with the right foot | The ball moved 1.018 m by the sample taken 0.50 s after skill completion. |
| 0:22.0 | Rise from prone | The recovery took 6.208 s of simulation time. |
| 0:29.8 | Rise from supine | The recovery took 6.608 s of simulation time. |

The [simplified URDF](../../ros/nao6_description/urdf/nao6_simplified.urdf) defines 21 independent joint targets. The [motion workbench](../../ros/nao6_description/control/whole_body/README.md) runs the corresponding pinned Glasgow Webots body and motion clips. From the repository root with Webots R2023b available, run a fresh kick trial with:

```bash
python3 ros/nao6_description/control/whole_body/motion_workbench.py run --case kick
```

The [capture record](capture.json) lists the video checksum, chapter times, controller hashes and measured outcome for each run. The two recovery clips start from settled fallen poses. The [ROS validation record](../../ros/nao6_description/control/whole_body/validation.json) also covers automatic fall detection and recovery through the [ROS Noetic launch file](../../ros/nao6_description/launch/whole_body.launch). Physical NAO6 kick range and balance still require hardware tests.
