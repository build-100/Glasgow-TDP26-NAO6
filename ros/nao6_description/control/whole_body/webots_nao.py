"""Webots device adapter, one-robot trial runner and optional ROS Noetic skill port."""
import json
import math
from pathlib import Path
from queue import Empty, Queue

from nao6_motion import Executor, LEFT_HIP, Library, matched_model, PostureFilter, RIGHT_HIP


class Devices:
    def __init__(self, robot, model, step_ms):
        self.motors = {}
        self.encoders = {}
        for name, limit in model.limits.items():
            motor = robot.getDevice(name)
            if motor is None or motor.getPositionSensor() is None:
                raise RuntimeError("Missing Webots motor or encoder: " + name)
            model.intersect(name, motor.getMinPosition(), motor.getMaxPosition(),
                            motor.getMaxVelocity(), motor.getMaxTorque())
            motor.setVelocity(model.limits[name].speed)
            motor.setAvailableTorque(model.limits[name].effort)
            encoder = motor.getPositionSensor()
            encoder.enable(step_ms)
            self.motors[name], self.encoders[name] = motor, encoder
        if model.limits[LEFT_HIP] != model.limits[RIGHT_HIP]:
            raise RuntimeError("Live coupled hip limits differ")
        for side in ("L", "R"):
            names = [side + "WristYaw"] + [side + "Phalanx" + str(i) for i in range(1, 9)]
            for name in names:
                robot.getDevice(name).setPosition(0.0)
        self.acc = robot.getDevice("accelerometer")
        self.acc.enable(step_ms)

    def read(self):
        measured = {name: enc.getValue() for name, enc in self.encoders.items()}
        if not all(math.isfinite(v) for v in measured.values()):
            raise RuntimeError("Invalid joint encoder sample")
        return measured, self.acc.getValues()

    def write(self, target):
        for name, motor in self.motors.items():
            motor.setPosition(target[LEFT_HIP if name == RIGHT_HIP else name])


class RosPort:
    def __init__(self, robot_id):
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        self.ros, self.joint_type = rospy, JointState
        self.robot_id = robot_id
        self.inbox = Queue(maxsize=32)
        rospy.init_node("nao6_body_" + robot_id, anonymous=False, disable_signals=True)
        self.joints = rospy.Publisher("/joint_states_" + robot_id, JointState, queue_size=2)
        self.results = rospy.Publisher("/skill_result_" + robot_id, String, queue_size=32)
        self.attitude = rospy.Publisher("/posture_" + robot_id, String, queue_size=1, latch=True)
        self.sub = rospy.Subscriber("/skill_request_" + robot_id, String, self.receive, queue_size=20)
        self.previous = None

    def receive(self, msg):
        from queue import Full
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict) or request.get("robot_id") != self.robot_id:
                raise ValueError("robot_id mismatch")
            seq = request.get("sequence_id")
            if type(seq) is not int or seq < 0:
                raise ValueError("sequence_id must be a non-negative integer")
            self.inbox.put_nowait(request)
        except (ValueError, Full) as exc:
            self.ros.logwarn("Dropped invalid or unqueueable skill request: %s", exc)

    def observe(self, measured, attitude):
        msg = self.joint_type()
        msg.header.stamp = self.ros.Time.now()
        msg.name = list(measured)
        msg.position = [measured[n] for n in msg.name]
        self.joints.publish(msg)
        if self.previous != attitude:
            self.attitude.publish(attitude)
            self.previous = attitude

    def result(self, seq, state, now, reason):
        self.results.publish(json.dumps(dict(robot_id=self.robot_id, sequence_id=seq,
                                            status=state, stamp_s=now, reason=reason)))


class Dispatcher:
    def __init__(self, executor, emit):
        self.exe, self.emit = executor, emit
        self.current = None
        self.last_id = -1
        self.recovery_armed = True
        self.recovery_id = -1

    def request(self, req, now, measured, attitude):
        seq = req["sequence_id"]
        if seq <= self.last_id:
            # A repeated bag message must not restart an action or overwrite its result.
            return
        self.last_id = seq
        skill = req.get("skill")
        if skill == "STOP":
            if self.current is not None:
                self.emit(self.current, "CANCELLED", now, "preempted by STOP")
            self.exe.start("STOP", now, measured, attitude)
            self.current = None
            self.recovery_armed = False
            self.emit(seq, "SUCCEEDED", now, "held measured joint positions")
            return
        try:
            if any(req.get(k) is not None for k in ("target_x_m", "target_y_m", "target_yaw_rad")):
                raise ValueError("Resolve field targets to a local primitive before dispatch")
            self.exe.start(skill, now, measured, attitude, timeout=req.get("timeout_s", 20.0),
                           primitive=req.get("primitive", "forward"), ball_ready=req.get("ball_ready", False),
                           yaw=req.get("yaw_rad", 0.0), pitch=req.get("pitch_rad", 0.0))
        except (TypeError, ValueError) as exc:
            self.emit(seq, "FAILED", now, str(exc))
            return
        self.current = seq
        if skill == "GET_UP":
            self.recovery_armed = False
        self.emit(seq, "RUNNING", now, str(skill))

    def tick(self, now, measured, attitude, automatic):
        target, result = self.exe.tick(now, measured, attitude)
        if result:
            self.emit(self.current, result[0], now, result[1])
            self.current = None
        if attitude == "upright" and self.exe.active != "GET_UP":
            self.recovery_armed = True
        if automatic and self.recovery_armed and not self.exe.active and attitude in ("front", "back"):
            self.exe.start("GET_UP", now, measured, attitude)
            self.current = self.recovery_id
            self.recovery_id -= 1
            self.recovery_armed = False
            self.emit(self.current, "RUNNING", now, "automatic recovery from " + attitude)
        return target


def run(config):
    from controller import Robot, Supervisor
    live = config["case"] == "ros"
    robot = Robot() if live else Supervisor()
    model = matched_model(config["urdf"], config["wenao"])
    library = Library(model, Path(config["wenao"]) / "motions")
    step_ms = int(robot.getBasicTimeStep())
    devices = Devices(robot, model, step_ms)
    filt = PostureFilter()
    port = RosPort(config["robot_id"]) if live else None
    events = []

    def emit(seq, status, now, reason):
        if port:
            port.result(seq, status, now, reason)
        else:
            events.append(dict(sequence_id=seq, status=status, time_s=now, reason=reason))

    dispatch = Dispatcher(Executor(library), emit)
    if not live:
        torso = robot.getSelf()
        ball = robot.getFromDef("TEST_BALL")
        trial = config["case"]
        expected = {"front": "front", "back": "back"}.get(trial, "upright")
        requests = [{"skill": "GET_UP"}] if trial in ("front", "back") else [
            {"skill": "STAND"}, {"skill": "WALK" if trial == "walk" else "KICK_FORWARD", "ball_ready": True}]
        issued, finished = 0, None
        body_start = ball_start = None
        floor_origin = None
    while robot.step(step_ms) != -1:
        now = robot.getTime()
        measured, acceleration = devices.read()
        attitude = filt.update(now, acceleration)
        if live:
            if port.ros.is_shutdown():
                return
            port.observe(measured, attitude)
            # Consume a bounded batch even if a client is sending continuously.
            for _ in range(32):
                try:
                    req = port.inbox.get_nowait()
                except Empty:
                    break
                dispatch.request(req, now, measured, attitude)
        elif finished is None:
            if events and events[-1]["status"] == "FAILED":
                finished = now
            elif now >= 1.0 and issued < len(requests) and dispatch.current is None:
                ready = attitude == expected if issued == 0 else attitude == "upright"
                if ready:
                    body_start = list(torso.getPosition())
                    ball_start = list(ball.getPosition())
                    # Freeze the forward axis at skill dispatch for the trial metric.
                    orientation = torso.getOrientation()
                    floor_origin = (orientation[0], orientation[3])
                    norm = math.hypot(*floor_origin)
                    floor_origin = (1.0, 0.0) if norm < 1e-6 else tuple(x / norm for x in floor_origin)
                    if trial == "kick" and issued == 1:
                        dx, dy = ball_start[0] - body_start[0], ball_start[1] - body_start[1]
                        fx, fy = floor_origin
                        vx, vy, vz = ball.getVelocity()[:3]
                        requests[issued]["ball_ready"] = (0 < dx*fx + dy*fy < 0.23
                            and -0.13 < -dx*fy + dy*fx < 0 and math.sqrt(vx*vx+vy*vy+vz*vz) < 0.03)
                    dispatch.request(dict(requests[issued], sequence_id=issued), now, measured, attitude)
                    issued += 1
            elif issued == len(requests) and dispatch.current is None:
                finished = now
            if now > 50.0:
                finished = now
                emit(issued, "FAILED", now, "trial timed out waiting for initial pose or skill")
        devices.write(dispatch.tick(now, measured, attitude, automatic=live))
        if not live and finished is not None and now - finished >= 0.50:
            pos, ball_pos = torso.getPosition(), ball.getPosition()
            def forward_change(end, start):
                if start is None:
                    return None
                return sum((end[i] - start[i]) * floor_origin[i] for i in range(2))
            walk_m = forward_change(pos, body_start)
            ball_m = forward_change(ball_pos, ball_start)
            complete = bool(events and events[-1]["status"] == "SUCCEEDED" and issued == len(requests))
            if trial == "walk":
                objective = walk_m is not None and walk_m > 0.02
            elif trial == "kick":
                objective = ball_m is not None and ball_m > 0.02
            else:
                objective = body_start is not None and pos[2] - body_start[2] > 0.10 and pos[2] > 0.22
            passed = complete and attitude == "upright" and objective
            report = dict(case=trial, passed=passed, final_posture=attitude, events=events,
                          torso_forward_m=walk_m, ball_forward_m=ball_m, torso_height_m=pos[2],
                          torso_start_m=body_start, ball_start_m=ball_start, end_time_s=now)
            Path(config["report"]).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print(json.dumps(report), flush=True)
            robot.simulationQuit(0 if passed else 1)
            return
