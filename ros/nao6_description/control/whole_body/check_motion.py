"""Offline regressions using the exact upstream assets. Run as shown in README.md."""
import argparse
import json
import math
import shlex
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock

from motion_workbench import PKG, inspect, matched_model, prepare, source_path
from nao6_motion import CLIPS, Executor, LEFT_HIP, Library, PostureFilter, RIGHT_HIP, SPEED_FRACTION
from webots_nao import Devices, Dispatcher


class WholeBodyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = PKG / "urdf" / "nao6_simplified.urdf"
        cls.model = matched_model(cls.urdf, SOURCE)
        cls.lib = Library(cls.model, SOURCE / "motions")
        cls.stand = cls.lib.compile(["StandInit"], {n: 0.0 for n in cls.model.names}).poses[-1]
        cls.stand = dict(cls.stand, RHipYawPitch=cls.stand[LEFT_HIP])

    def test_model_and_real_motion_data(self):
        stats = inspect(self.model, self.lib)
        self.assertEqual((len(self.model.names), len(self.model.limits)), (21, 22))
        self.assertAlmostEqual(self.model.limits["RHipRoll"].lower, -0.738274)
        self.assertEqual(stats["StandUpFromBack"]["coupled_conflict_frames"], 13)
        self.assertEqual(stats["StandUpFromBack"]["largest_hip_disagreement_rad"], 1.02)
        self.assertEqual(stats["RightShoot"]["frames"], 8)
        kick = self.lib.clips["RightShoot"]
        self.assertGreater(abs(kick[2][1]["RHipPitch"] - kick[1][1]["RHipPitch"]), 0.4)
        self.assertAlmostEqual(kick[2][1]["LHipPitch"], kick[1][1]["LHipPitch"])

    def test_every_pose_and_segment_on_all_seven_clips(self):
        for name in CLIPS:
            with self.subTest(name=name):
                t = self.lib.compile([name], self.stand)
                self.assertEqual(set(t.poses[0]), set(self.model.names))
                self.assertNotIn(RIGHT_HIP, t.poses[0])
                for i in range(1, len(t.times)):
                    dt = t.times[i] - t.times[i-1]
                    self.assertGreater(dt, 0)
                    for joint, q in t.poses[i].items():
                        lim = self.model.limits[joint]
                        self.assertTrue(lim.lower <= q <= lim.upper)
                        self.assertLessEqual(abs(q-t.poses[i-1][joint])/dt, lim.speed*SPEED_FRACTION + 1e-8)
                self.assertEqual(t.sample(t.duration+1), t.poses[-1])

    def test_feedback_completion_for_walk_kick_front_back(self):
        for skill, start in (("WALK","upright"), ("KICK_FORWARD","upright"),
                             ("GET_UP","front"), ("GET_UP","back")):
            with self.subTest(skill=skill, start=start):
                exe = Executor(self.lib)
                measured = self.stand.copy()
                exe.start(skill, 0.0, measured, start, ball_ready=True)
                for i in range(1250):
                    now = i * 0.016
                    # Synthetic one-tick-delayed encoders and a prescribed torso posture.
                    expected = start if now < exe.track.duration*0.8 else "upright"
                    pose, result = exe.tick(now, measured, expected)
                    measured = dict(pose, RHipYawPitch=pose[LEFT_HIP])
                    if result:
                        break
                self.assertEqual(result[0], "SUCCEEDED")

    def test_recovery_cannot_pass_lying_or_with_stuck_encoder(self):
        for attitude in ("back", "upright"):
            with self.subTest(attitude=attitude):
                exe = Executor(self.lib)
                measured = self.stand.copy()
                exe.start("GET_UP", 0, measured, "back", timeout=12)
                measured["RKneePitch"] = 1.8 if attitude == "upright" else measured["RKneePitch"]
                for now in (9.0, 10.0, 11.5):
                    self.assertIsNone(exe.tick(now, measured, attitude)[1])
                self.assertEqual(exe.tick(12.0, measured, attitude)[1][0], "FAILED")

    def test_detect_fall_once_and_cancel(self):
        log = []
        d = Dispatcher(Executor(self.lib), lambda *e: log.append(e))
        d.request({"skill":"WALK", "sequence_id":1}, 0, self.stand, "upright")
        d.request({"skill":"WALK", "sequence_id":1}, 0.1, self.stand, "upright")
        d.tick(0.3, self.stand, "front", True)
        self.assertEqual([(e[0], e[1]) for e in log], [(1,"RUNNING"), (1,"FAILED"), (-1,"RUNNING")])
        d.request({"skill":"STOP", "sequence_id":2}, 0.4, self.stand, "front")
        d.tick(0.5, self.stand, "front", True)
        self.assertEqual([(e[0], e[1]) for e in log[-2:]], [(-1,"CANCELLED"),(2,"SUCCEEDED")])
        self.assertIsNone(d.exe.active)

    def test_request_preconditions(self):
        exe = Executor(self.lib)
        with self.assertRaises(ValueError):
            exe.start("KICK_FORWARD", 0, self.stand, "upright")
        with self.assertRaises(ValueError):
            exe.start("HEAD_LOOK", 0, self.stand, "upright", yaw=4)
        with self.assertRaises(ValueError):
            exe.start("GET_UP", 0, self.stand, "side")
        exe.start("HEAD_LOOK", 0, self.stand, "upright", yaw=0.4, pitch=0.2)
        self.assertAlmostEqual(exe.track.poses[-1]["HeadYaw"], 0.4)

    def test_sensor_frame_and_debounce(self):
        for acc, expected in (((0,0,-9.81),"upright"), ((-9.81,0,0),"front"), ((9.81,0,0),"back"),
                              ((0,9.81,0),"side"), ((20,0,0),"moving")):
            filt = PostureFilter()
            self.assertEqual(filt.update(1, acc), "moving")
            self.assertEqual(filt.update(1.3, acc), expected)

    def test_concrete_device_commands_and_encoders(self):
        model = matched_model(self.urdf, SOURCE)
        mock_devices = {}
        for name, limit in model.limits.items():
            m = Mock()
            m.getMinPosition.return_value, m.getMaxPosition.return_value = limit.lower, limit.upper
            m.getMaxTorque.return_value, m.getMaxVelocity.return_value = limit.effort, limit.speed
            m.getPositionSensor.return_value.getValue.return_value = self.stand[name]
            mock_devices[name] = m
        for side in ("L","R"):
            for suffix in ["WristYaw"] + ["Phalanx"+str(i) for i in range(1,9)]:
                mock_devices[side+suffix] = Mock()
        mock_devices["accelerometer"] = Mock()
        robot = Mock()
        robot.getDevice.side_effect = mock_devices.__getitem__
        devices = Devices(robot, model, 16)
        measured, _ = devices.read()
        self.assertEqual(set(measured), set(model.limits))
        target = dict(self.stand, LHipYawPitch=-0.1)
        devices.write(target)
        for name in model.limits:
            mock_devices[name].setPosition.assert_called_with(target[LEFT_HIP if name == RIGHT_HIP else name])
        mock_devices["RWristYaw"].setPosition.assert_called_with(0.0)

    def test_four_independent_worlds_and_optional_ros_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            for case in ("walk", "kick", "front", "back", "ros"):
                path = prepare(SOURCE, self.urdf, Path(tmp)/case, case, "red1")
                world = path.read_text()
                self.assertIn('EXTERNPROTO "../protos/NaoV6.proto"', world)
                self.assertIn('controller "nao6_skills"', world)
                cfg = path.parents[1]/"controllers"/"nao6_skills"/"config.json"
                self.assertEqual(json.loads(cfg.read_text())["case"], case)
                compile(cfg.with_name("nao6_skills.py").read_text(), "nao6_skills.py", "exec")
                self.assertEqual(len(list((path.parents[1]/"motions").glob("*.motion"))), 7)
        args = ET.parse(PKG/"launch"/"whole_body.launch").find(".//node").get("args")
        argv = shlex.split(args.replace("$(arg wenao)", "/test with space/Wenao")
                           .replace("$(arg webots)", "/test with space/webots").replace("$(arg robot_id)", "red1"))
        self.assertIn("/test with space/Wenao", argv)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--wenao", default="auto")
    args, rest = parser.parse_known_args()
    SOURCE = source_path(args.wenao)
    unittest.main(argv=[sys.argv[0]]+rest)
