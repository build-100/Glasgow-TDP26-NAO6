#!/usr/bin/env python3
"""Inspect or run reproducible single-robot trials with the Glasgow Webots model."""
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from urllib.request import urlopen
from pathlib import Path


def package_dir():
    here = Path(__file__).resolve().parent
    if (here / "nao6_motion.py").is_file():
        return here.parents[1]
    import rospkg
    return Path(rospkg.RosPack().get_path("nao6_description"))


PKG = package_dir()
CODE = PKG / "control" / "whole_body"
sys.path.insert(0, str(CODE))
from nao6_motion import CLIPS, LEFT_HIP, RIGHT_HIP, REVISION, Library, checked_bytes, matched_model

PROTOS = {
    "NaoV6.proto": "224e53568068e9681ddbb40d80319845eabf00c9d8ae7cdbdfca309ba9ef9b9d",
    "NaoLeftWristH25Realistic.proto": "63426314cdf50653a148e20ef11b2b0432e4afa966bc8b3ed967e087551ca096",
    "NaoRightWristH25Realistic.proto": "a5896ad55698a344168abbb9ecff1e6034c79f7cb3bc90e9be1cfb35f9fb2acb",
}
SOURCE_URDF = "327dce62542ecd1327acf3fa7c5ec7b59274d44ce6c09a9ade3417cc39f132d6"
TEXTURES = {"chest_button.png": "59f5785820abe5afa31fe0c97ccc93e17d88805d",
            "ear.jpg": "e3c239d528570ed0b7b0347bcdd3034fcb38442c"}


def asset_hash(data, expected):
    if len(expected) == 40:  # Pinned Git blob ID for the two binary textures.
        return hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()
    return hashlib.sha256(data.replace(b"\r\n", b"\n")).hexdigest()


def source_path(setting):
    if setting != "auto":
        return Path(setting).expanduser().resolve()
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    root = cache / "nao6_skill_lab" / REVISION / "Wenao"
    assets = {"protos/NAOH25V60.urdf": SOURCE_URDF}
    assets.update(("protos/" + name, sha) for name, sha in PROTOS.items())
    assets.update(("motions/" + name + ".motion", sha) for name, sha in CLIPS.items())
    assets.update(("protos/textures/" + name, sha) for name, sha in TEXTURES.items())
    base = "https://raw.githubusercontent.com/RajasJoshi/UoG_Robotics_TDP4/" + REVISION + "/Wenao/"
    for name, digest in assets.items():
        dest = root / name
        if dest.is_file() and asset_hash(dest.read_bytes(), digest) == digest:
            continue
        with urlopen(base + name, timeout=25) as reply:
            data = reply.read(2_000_000)
        if asset_hash(data, digest) != digest:
            raise ValueError("Upstream content does not match the pinned asset: " + name)
        dest.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=dest.parent, delete=False) as tmp:
            tmp.write(data)
        os.replace(tmp.name, dest)
    return root


def inspect(model, library):
    initial = library.compile(["StandInit"], {n: 0.0 for n in model.names}).poses[-1]
    result = {}
    for name in CLIPS:
        frames = library.clips[name]
        conflicts = [abs(p[LEFT_HIP] - p[RIGHT_HIP]) for _, p in frames if LEFT_HIP in p and RIGHT_HIP in p]
        excursions = [abs(p[n] - model.limits[n].bound(p[n])) for _, p in frames for n in model.names if n in p]
        track = library.compile([name], initial)
        result[name] = dict(frames=len(frames), source_end_s=frames[-1][0],
                            adapted_s_from_stand=round(track.duration, 3),
                            coupled_conflict_frames=sum(d > 1e-8 for d in conflicts),
                            largest_hip_disagreement_rad=round(max(conflicts, default=0), 4),
                            limited_samples=sum(d > 1e-8 for d in excursions))
    return result


def prepare(wenao, urdf, output, trial, robot_id):
    # Only locally installed upstream assets enter the temporary Webots project.
    # Preserve the three original PROTOs and their Webots-only licence headers.
    source = wenao / "protos"
    for name, digest in PROTOS.items():
        checked_bytes(source / name, digest)
    if not re.fullmatch(r"(red|blue)[1-4]", robot_id):
        raise ValueError("robot_id must be red1..red4 or blue1..blue4")
    if output is None:
        root = Path(tempfile.mkdtemp(prefix="nao6_motion_"))
    else:
        root = Path(output).resolve()
        if root.exists() and any(root.iterdir()):
            raise ValueError("Choose an empty trial directory: " + str(root))
        root.mkdir(parents=True, exist_ok=True)
    controllers = root / "controllers" / "nao6_skills"
    worlds = root / "worlds"
    (root / "protos" / "textures").mkdir(parents=True)
    controllers.mkdir(parents=True)
    worlds.mkdir()
    for name in PROTOS:
        shutil.copyfile(source / name, root / "protos" / name)
    for name, digest in TEXTURES.items():
        texture = source / "textures" / name
        if texture.is_file():
            if asset_hash(texture.read_bytes(), digest) != digest:
                raise ValueError("Unrecognised appearance texture: " + name)
            shutil.copyfile(texture, root / "protos" / "textures" / name)
        else:
            print("Optional appearance texture missing: " + name, file=sys.stderr)
    for name in ("nao6_motion.py", "webots_nao.py"):
        shutil.copyfile(CODE / name, controllers / name)
    (root / "motions").mkdir()
    (root / "urdf").mkdir()
    for name in CLIPS:
        shutil.copyfile(wenao / "motions" / (name + ".motion"), root / "motions" / (name + ".motion"))
    shutil.copyfile(urdf, root / "urdf" / "nao6_simplified.urdf")
    shutil.copyfile(PKG / "LICENSE.upstream", root / "LICENSE.upstream")
    config = dict(case=trial, robot_id=robot_id)
    (controllers / "config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    controller = ("from pathlib import Path\nimport json\nfrom webots_nao import run\n"
                  "root = Path(__file__).resolve().parents[2]\n"
                  "cfg = json.loads(Path(__file__).with_name('config.json').read_text(encoding='utf-8'))\n"
                  "cfg.update(wenao=str(root), urdf=str(root / 'urdf' / 'nao6_simplified.urdf'),\n"
                  "           report=str(root / 'trial_result.json'))\nrun(cfg)\n")
    (controllers / "nao6_skills.py").write_text(controller, encoding="utf-8")
    angle = {"front": 1.5708, "back": -1.5708}.get(trial, 0)
    height = 0.19 if angle else 0.334
    ball_x = 0.15 if trial in ("kick", "ros") else 1.50
    is_supervisor = "FALSE" if trial == "ros" else "TRUE"
    world = f"""#VRML_SIM R2023b utf8
EXTERNPROTO "../protos/NaoV6.proto"
WorldInfo {{ basicTimeStep 16 coordinateSystem "ENU" }}
Viewpoint {{ orientation 0.45 0.20 0.87 2.1 position 1.3 -1.6 0.95 }}
Background {{ skyColor [0.72 0.79 0.85] }}
DirectionalLight {{ direction -0.4 0.3 -1 }}
Solid {{
  translation 0 0 -0.025
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.12 0.45 0.22 roughness 1 metalness 0 }}
    geometry DEF FLOOR Box {{ size 4 3 0.05 }} }} ]
  boundingObject USE FLOOR
}}
DEF TEST_BALL Solid {{
  translation {ball_x} -0.055 0.051
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.95 0.57 0.15 roughness 1 metalness 0 }}
    geometry DEF BALL Sphere {{ radius 0.05 }} }} ]
  boundingObject USE BALL
  physics Physics {{ density -1 mass 0.07 }}
}}
DEF PLAYER NaoV6 {{
  translation 0 0 {height}
  rotation 0 1 0 {angle}
  name "{robot_id}"
  controller "nao6_skills"
  supervisor {is_supervisor}
}}
"""
    path = worlds / "skill_lab.wbt"
    path.write_text(world, encoding="utf-8")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("inspect", "prepare", "run"))
    parser.add_argument("--wenao", default="auto", help="Pinned Wenao directory or auto (verified cache)")
    parser.add_argument("--case", choices=("walk", "kick", "front", "back", "ros"), default="walk")
    parser.add_argument("--robot-id", default="red1")
    parser.add_argument("--output", help="Empty directory for a new trial (default: temporary directory)")
    parser.add_argument("--webots", default="webots", help="Installed Webots executable")
    # roslaunch passes node name and logging remaps to the wrapper.
    args = parser.parse_args([s for s in sys.argv[1:] if not s.startswith(("__name:=", "__log:="))])
    wenao = source_path(args.wenao)
    urdf = PKG / "urdf" / "nao6_simplified.urdf"
    checked_bytes(wenao / "protos" / "NAOH25V60.urdf", SOURCE_URDF)
    model = matched_model(urdf, wenao)
    library = Library(model, wenao / "motions")
    if args.mode == "inspect":
        print(json.dumps(inspect(model, library), indent=2))
        return 0
    path = prepare(wenao, urdf, args.output, args.case, args.robot_id)
    print(path, flush=True)
    if args.mode == "prepare":
        return 0
    live = args.case == "ros"
    try:
        result = subprocess.run([args.webots, "--batch", "--mode=" + ("realtime" if live else "fast"),
                                 str(path)], check=False, timeout=None if live else 180)
    except subprocess.TimeoutExpired:
        sys.exit("Webots produced no final trial result within 180 wall-clock seconds")
    if result.returncode or live:
        return result.returncode
    report = path.parents[1] / "trial_result.json"
    return 0 if report.is_file() and json.loads(report.read_text(encoding="utf-8"))["passed"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as exc:
        sys.exit(str(exc))
