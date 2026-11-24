"""Compile the pinned Glasgow Webots clips against the team's simplified NAO6 URDF."""
import bisect
import csv
import hashlib
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

REVISION = "8a0949302aac5d610339fa02f3e4cfc437abb17d"
# SHA256 of upstream text with LF line endings (also accepts Git CRLF checkouts).
CLIPS = {
    "StandInit": "e079f138636639d17e3f090ec2ab150dab8206346858e9dd887efc487bd7addc",
    "Forwards50": "f4ce52683c178ed6eb81d2dc1e197a5b1e3751cda38fd45826a12641861a9a61",
    "TurnLeft10": "bb46188cb4a38487d757dc7e2ce81ae5a9dfca3050b3fd7f6aa610c9148d5284",
    "TurnRight10": "2af9265ab6a7100559430a5a11a0a167a1ad33d55591f75622d6f7e3cb219394",
    "RightShoot": "3c1c58727f5f88b85d4148ea22b9c2cdd34acb94a8fc249dd9d7e0c8f30792a6",
    "StandUpFromFront": "88c33caef5bcf98d15588a16173324de80b49a9a4e05a4c076d531e9ae840a78",
    "StandUpFromBack": "3a760c5b16cc61e3f97683759b6bad7e5e0233d47331729db11b14a60b7d0636",
}
LEFT_HIP, RIGHT_HIP = "LHipYawPitch", "RHipYawPitch"
LOCKED = {"LHand", "RHand", "LWristYaw", "RWristYaw"}
SPEED_FRACTION = 0.85
KICK_ROLL_JOINTS = {"LHipRoll", "RHipRoll", "LAnkleRoll", "RAnkleRoll"}


def checked_bytes(path, expected):
    data = Path(path).read_bytes().replace(b"\r\n", b"\n")
    if hashlib.sha256(data).hexdigest() != expected:
        raise ValueError("Unrecognised asset: {}".format(path))
    return data


@dataclass(frozen=True)
class Limit:
    lower: float
    upper: float
    speed: float
    effort: float

    def bound(self, value):
        return min(self.upper, max(self.lower, value))


class Model:
    def __init__(self, urdf):
        tree = ET.parse(urdf)
        joints = {j.get("name"): j for j in tree.findall("joint")}
        self.limits = {}
        for name, joint in joints.items():
            if joint.get("type") == "revolute":
                lim = joint.find("limit")
                limit = Limit(*(float(lim.get(k)) for k in ("lower", "upper", "velocity", "effort")))
                if not (limit.lower < limit.upper and limit.speed > 0 and limit.effort > 0):
                    raise ValueError("Invalid URDF limits for " + name)
                self.limits[name] = limit
        mimic = joints[RIGHT_HIP].find("mimic")
        if (len(self.limits) != 22 or mimic is None or mimic.get("joint") != LEFT_HIP
                or float(mimic.get("multiplier", 1)) != 1 or float(mimic.get("offset", 0)) != 0
                or any(joints[n].get("type") != "fixed" for n in LOCKED)):
            raise ValueError("Expected the simplified H25 V6 model with its 1:1 hip linkage")
        self.names = tuple(n for n in self.limits if n != RIGHT_HIP)
        if self.limits[LEFT_HIP] != self.limits[RIGHT_HIP]:
            raise ValueError("The coupled hip motor limits must agree")

    def project(self, measured):
        if any(not math.isfinite(measured[n]) for n in self.names):
            raise ValueError("Non-finite encoder input")
        return {n: self.limits[n].bound(measured[n]) for n in self.names}

    def intersect(self, name, lower, upper, speed, effort):
        old = self.limits[name]
        # Webots uses (0, 0) for an unbounded rotary position actuator.
        if lower == upper == 0.0:
            lower, upper = old.lower, old.upper
        limit = Limit(max(lower, old.lower), min(upper, old.upper),
                      min(speed, old.speed), min(effort, old.effort))
        if not (limit.lower < limit.upper and limit.speed > 0 and limit.effort > 0):
            raise ValueError("Incompatible actuator limits: " + name)
        self.limits[name] = limit


def matched_model(urdf, wenao):
    model = Model(urdf)
    proto = checked_bytes(Path(wenao) / "protos" / "NaoV6.proto",
                          "224e53568068e9681ddbb40d80319845eabf00c9d8ae7cdbdfca309ba9ef9b9d")
    fields = ("minPosition", "maxPosition", "maxVelocity", "maxTorque")
    found = set()
    for block in re.findall(r"RotationalMotor\s*\{([^}]+)\}", proto.decode("utf-8")):
        match = re.search(r'\bname\s+"([^"]+)"', block)
        if match is None or match[1] not in model.limits:
            continue
        name = match[1]
        values = dict(re.findall(r"\b(minPosition|maxPosition|maxVelocity|maxTorque)\s+([-\d.]+)", block))
        model.intersect(name, *(float(values.get(k, 0)) for k in fields))
        found.add(name)
    if found != set(model.limits) or model.limits[LEFT_HIP] != model.limits[RIGHT_HIP]:
        raise ValueError("Incomplete or incompatible Webots actuator map")
    return model


class Track:
    def __init__(self, first):
        self.times = [0.0]
        self.poses = [first.copy()]

    def append(self, target, source_interval, model):
        previous = self.poses[-1]
        travel_time = max(abs(target[n] - previous[n]) / (model.limits[n].speed * SPEED_FRACTION)
                          for n in model.names)
        dt = max(source_interval, travel_time)
        if dt > 0:
            self.times.append(self.times[-1] + dt)
            self.poses.append(target.copy())

    @property
    def duration(self):
        return self.times[-1]

    def sample(self, elapsed):
        if elapsed <= 0:
            return self.poses[0].copy()
        i = bisect.bisect_right(self.times, elapsed)
        if i == len(self.times):
            return self.poses[-1].copy()
        a, b = self.poses[i - 1], self.poses[i]
        t = (elapsed - self.times[i - 1]) / (self.times[i] - self.times[i - 1])
        return {n: a[n] + t * (b[n] - a[n]) for n in a}


class Library:
    def __init__(self, model, folder):
        self.model = model
        self.clips = {}
        for name, digest in CLIPS.items():
            data = checked_bytes(Path(folder) / (name + ".motion"), digest)
            rows = iter(csv.reader(data.decode("utf-8").splitlines()))
            header = next(rows)
            columns = header[2:]
            if (header[:2] != ["#WEBOTS_MOTION", "V1.0"] or len(set(columns)) != len(columns)
                    or set(columns) - (set(model.limits) | LOCKED)):
                raise ValueError("Unexpected motion channels in " + name)
            frames = []
            for row in rows:
                if len(row) != len(header):
                    raise ValueError("Invalid frame width in " + name)
                minutes, seconds, ms = map(int, row[0].split(":"))
                stamp = 60 * minutes + seconds + ms / 1000.0
                if stamp < 0 or (frames and stamp <= frames[-1][0]):
                    raise ValueError("Non-increasing motion time in " + name)
                pose = {n: float(v) for n, v in zip(columns, row[2:]) if v != "*"}
                if not all(math.isfinite(v) for v in pose.values()):
                    raise ValueError("Invalid position in " + name)
                frames.append((stamp, pose))
            self.clips[name] = frames

    def compile(self, clips, measured):
        model = self.model
        track = Track(model.project(measured))
        for clip in clips:
            last_stamp = 0.0
            for stamp, pose in self.clips[clip]:
                target = track.poses[-1].copy()
                for name in model.names:
                    # Sparse channels keep the last value. The mechanical master is the left hip.
                    if name in pose:
                        angle = pose[name]
                        if clip == "RightShoot" and name in KICK_ROLL_JOINTS:
                            # Keep the lateral transfer over the stance foot during the swing.
                            angle *= 0.55
                        target[name] = model.limits[name].bound(angle)
                track.append(target, stamp - last_stamp, model)
                if clip.startswith("StandUp") and stamp == 0:
                    track.append(target, 0.80, model)
                elif clip == "StandUpFromFront" and stamp == 2.50:
                    # Let the feet take the load before the source clip extends both legs.
                    track.append(target, 0.45, model)
                last_stamp = stamp
        return track


def posture(acceleration):
    # NaoV6.proto rotates the accelerometer by pi about body X. The model's
    # InertialUnit disables an axis, so it cannot provide a complete pitch reading.
    ax, ay, az = acceleration
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if not math.isfinite(norm) or not 7.0 <= norm <= 12.5:
        return "moving"
    if -az / norm > 0.88:
        return "upright"
    if abs(ax) / norm > 0.80:
        return "front" if ax < 0 else "back"
    if abs(ay) / norm > 0.80:
        return "side"
    return "moving"


class PostureFilter:
    def __init__(self):
        self.candidate, self.since = None, 0.0
        self.gravity = None
        self.previous_time = None

    def update(self, now, acceleration):
        if not all(math.isfinite(v) for v in acceleration):
            self.gravity = self.candidate = self.previous_time = None
            return "moving"
        if self.gravity is None or now <= self.previous_time:
            self.gravity = tuple(acceleration)
            self.candidate = None
        else:
            # Ground contact produces alternating acceleration peaks even when lying still.
            memory = 0.875 ** ((now - self.previous_time) / 0.016)
            self.gravity = tuple(memory * g + (1.0 - memory) * a for g, a in zip(self.gravity, acceleration))
        self.previous_time = now
        value = posture(self.gravity)
        if value != self.candidate:
            self.candidate, self.since = value, now
        return value if now - self.since >= 0.20 else "moving"


WALKS = {"forward": "Forwards50", "left": "TurnLeft10", "right": "TurnRight10"}


class Executor:
    """One local skill at a time. Each tick takes measured positions, including the hip follower."""
    def __init__(self, library):
        self.library = library
        self.active = None
        self.track = None
        self.hold = None
        self.settled_since = None

    def start(self, skill, now, measured, attitude, timeout=20.0, primitive="forward",
              ball_ready=False, yaw=0.0, pitch=0.0):
        if skill == "STOP":
            self.active = None
            self.hold = self.library.model.project(measured)
            return
        if self.active:
            raise ValueError("A skill is running")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("A positive finite timeout is required")
        if skill == "GET_UP":
            if attitude not in ("front", "back"):
                raise ValueError("GET_UP requires a stable front or back fall")
            clips = ["StandInit", "StandUpFromFront" if attitude == "front" else "StandUpFromBack", "StandInit"]
        elif attitude != "upright":
            raise ValueError("Upright posture required")
        elif skill == "WALK" and primitive in WALKS:
            clips = ["StandInit", WALKS[primitive], "StandInit"]
        elif skill == "KICK_FORWARD":
            if ball_ready is not True:
                raise ValueError("Confirm the settled ball is aligned with the right foot")
            clips = ["StandInit", "RightShoot", "StandInit"]
        elif skill == "STAND":
            clips = ["StandInit"]
        elif skill == "HEAD_LOOK":
            if not all(math.isfinite(v) and self.library.model.limits[n].bound(v) == v
                       for n, v in (("HeadYaw", yaw), ("HeadPitch", pitch))):
                raise ValueError("Head target outside URDF limits")
            clips = []
        else:
            raise ValueError("Unsupported skill or WALK primitive")
        track = self.library.compile(clips, measured)
        if skill == "HEAD_LOOK":
            target = dict(track.poses[-1], HeadYaw=yaw, HeadPitch=pitch)
            track.append(target, 0.0, self.library.model)
        self.track = track
        self.active, self.start_time, self.timeout = skill, now, timeout
        self.settled_since = None

    def tick(self, now, measured, attitude):
        if self.hold is None:
            self.hold = self.library.model.project(measured)
        if not self.active:
            return self.hold, None
        elapsed = now - self.start_time
        result = None
        fallen = attitude in ("front", "back", "side")
        if elapsed >= self.timeout:
            result = ("FAILED", "skill timed out")
        elif fallen and self.active != "GET_UP":
            result = ("FAILED", "fall detected: " + attitude)
        else:
            self.hold = self.track.sample(elapsed)
            error = max(abs(measured[n] - self.hold[n]) for n in self.hold)
            error = max(error, abs(measured[RIGHT_HIP] - self.hold[LEFT_HIP]))
            if elapsed >= self.track.duration and attitude == "upright" and error <= 0.10:
                if self.settled_since is None:
                    self.settled_since = now
                if now - self.settled_since >= 0.20:
                    result = ("SUCCEEDED", "final encoders and upright posture settled")
            else:
                self.settled_since = None
        if result:
            self.active = None
            if result[0] != "SUCCEEDED":
                self.hold = self.library.model.project(measured)
        return self.hold, result
