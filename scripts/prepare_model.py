"""Reproducible adaptation; retain upstream model terms and original geometry."""
import copy
import hashlib
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros/nao6_description"
URDF = PACKAGE / "urdf"
SOURCE_HASH = "327dce62542ecd1327acf3fa7c5ec7b59274d44ce6c09a9ade3417cc39f132d6"
HAND_PATTERN = re.compile(r"^[LR](?:Hand|Finger\d+|Thumb\d+|WristYaw)$")


def make_variant(source, fix_hands=False, collision_visuals=False):
    root = copy.deepcopy(source)
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", "package://nao6_description/meshes/" + mesh.get("filename").split("/")[-1])
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "texture" and not child.get("filename"):
                parent.remove(child)
    fixed = []
    if fix_hands:
        for joint in root.findall("joint"):
            if HAND_PATTERN.fullmatch(joint.get("name", "")):
                joint.set("type", "fixed")
                for child in list(joint):
                    if child.tag in {"axis", "limit", "mimic", "dynamics", "safety_controller", "calibration"}:
                        joint.remove(child)
                fixed.append(joint.get("name"))
    if collision_visuals:
        for link in root.findall("link"):
            for visual in list(link.findall("visual")):
                link.remove(visual)
            for collision in link.findall("collision"):
                visual = ET.SubElement(link, "visual")
                for child in collision:
                    if child.tag in {"origin", "geometry"}:
                        visual.append(copy.deepcopy(child))
                material = ET.SubElement(visual, "material", name="inspection_grey")
                ET.SubElement(material, "color", rgba="0.75 0.78 0.82 1")
    return root, fixed


def prepare():
    source_path = URDF / "NAOH25V60_source.urdf"
    if hashlib.sha256(source_path.read_bytes()).hexdigest() != SOURCE_HASH:
        raise RuntimeError("Unexpected source URDF; review provenance before adapting.")
    source = ET.parse(source_path).getroot()
    report = {"source_sha256": SOURCE_HASH, "runtime_validated": False, "variants": {}}
    for name, fixed, primitives in [("full", False, False), ("simplified", True, False),
                                     ("collision_visual", True, True)]:
        model, changed = make_variant(source, fixed, primitives)
        header = ("<?xml version='1.0' encoding='utf-8'?>\n"
                  "<!-- Adapted from the pinned upstream NAO6 source; see LICENSE.upstream.\n"
                  "     Changes: portable mesh paths, empty texture cleanup, optional fixed hands/wrists\n"
                  "     and collision-based visuals. No leg/arm/head inertias or contacts removed. -->\n")
        data = (header + ET.tostring(model, encoding="unicode") + "\n").encode("utf-8")
        (URDF / ("nao6_" + name + ".urdf")).write_bytes(data)
        report["variants"][name] = {"sha256": hashlib.sha256(data).hexdigest(),
                                     "fixed_hand_and_wrist_joints": changed,
                                     "collision_based_visuals": primitives}
    (PACKAGE / "adaptation_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("Prepared full, hand/wrist-simplified and collision-visual descriptions.")


if __name__ == "__main__":
    prepare()
