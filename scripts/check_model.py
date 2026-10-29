"""Structural checks, not a ROS/Gazebo or physical stability test."""
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros/nao6_description"


def signature(element):
    if element is None:
        return None
    return (element.tag, tuple(sorted(element.attrib.items())),
            tuple(signature(child) for child in element))


def stats(root):
    joints = root.findall("joint")
    moving = [j for j in joints if j.get("type") != "fixed"]
    return {"links": len(root.findall("link")), "joints": len(joints),
            "independent_moving_joints": sum(j.find("mimic") is None for j in moving),
            "inertials": len(root.findall("link/inertial")),
            "collisions": len(root.findall("link/collision")),
            "mass_kg": round(sum(float(m.get("value")) for m in root.findall("link/inertial/mass")), 8)}


def check_tree(root):
    links = {l.get("name") for l in root.findall("link")}
    assert len(links) == len(root.findall("link")), "Duplicate link name"
    parents = {}
    for j in root.findall("joint"):
        parent, child = j.find("parent").get("link"), j.find("child").get("link")
        assert parent in links and child in links
        assert child not in parents, "Link has multiple parents"
        parents[child] = parent
    assert links - set(parents) == {"base_link"}, "Unexpected tree root"
    for link in links:
        seen = set()
        while link in parents:
            assert link not in seen, "Joint cycle"
            seen.add(link)
            link = parents[link]
        assert link == "base_link"
    assert {"CameraTop", "CameraBottom", "Accelerometer", "Gyrometer"} <= links


def check():
    source_path = PACKAGE / "urdf/NAOH25V60_source.urdf"
    source = ET.parse(source_path).getroot()
    report = json.loads((PACKAGE / "adaptation_report.json").read_text(encoding="utf-8"))
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == report["source_sha256"]
    originals = {l.get("name"): l for l in source.findall("link")}
    original_joints = {j.get("name"): j for j in source.findall("joint")}
    result = {}
    for name in ("full", "simplified", "collision_visual"):
        file = PACKAGE / ("urdf/nao6_" + name + ".urdf")
        assert hashlib.sha256(file.read_bytes()).hexdigest() == report["variants"][name]["sha256"]
        root = ET.parse(file).getroot()
        check_tree(root)
        assert set(originals) == {l.get("name") for l in root.findall("link")}
        for link in root.findall("link"):
            original = originals[link.get("name")]
            assert signature(link.find("inertial")) == signature(original.find("inertial"))
            assert [signature(c) for c in link.findall("collision")] == [signature(c) for c in original.findall("collision")]
        changed = set(report["variants"][name]["fixed_hand_and_wrist_joints"])
        for joint in root.findall("joint"):
            if joint.get("name") not in changed:
                assert signature(joint) == signature(original_joints[joint.get("name")])
            else:
                assert joint.get("type") == "fixed" and joint.find("mimic") is None
        for mesh in root.findall(".//mesh"):
            prefix = "package://nao6_description/"
            assert mesh.get("filename").startswith(prefix)
            assert (PACKAGE / mesh.get("filename")[len(prefix):]).is_file()
        result[name] = stats(root)
        assert result[name]["mass_kg"] == stats(source)["mass_kg"]
        assert result[name]["independent_moving_joints"] == (25 if name == "full" else 21)
    manifest = json.loads((PACKAGE / "source_manifest.json").read_text(encoding="utf-8"))
    for item in manifest["files"]:
        if item["path"] == "LICENSE":
            file = PACKAGE / "LICENSE.upstream"
        elif item["path"].endswith("NAOH25V60.urdf"):
            file = source_path
        else:
            file = PACKAGE / "meshes" / Path(item["path"]).name
        assert hashlib.sha256(file.read_bytes()).hexdigest() == item["sha256"]
    for file in list((PACKAGE / "launch").glob("*.launch")) + list((PACKAGE / "worlds").glob("*.world")) + [PACKAGE / "package.xml"]:
        ET.parse(file)
    return result


if __name__ == "__main__":
    print(json.dumps(check(), indent=2))
    print("Structural checks passed. No ROS/Gazebo runtime validation was performed.")
