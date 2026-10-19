#!/usr/bin/env python3
"""Derive the motor-1-forward model without changing the original X model."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

models = Path(__file__).resolve().parents[1] / "models"
source = models / "transform_quad"
target = models / "transform_quad_plus"
target.mkdir(exist_ok=True)
tree = ET.parse(source / "model.sdf")
model = tree.getroot().find("model")
model.set("name", "transform_quad_plus")
# Gazebo FLU +45 degrees is PX4 FRD -45 degrees. Descendants follow joints.
angle = math.pi / 4
for joint in model.findall("joint"):
    if joint.findtext("parent") != "base_link":
        continue
    pose = joint.find("pose")
    assert pose.get("relative_to") == "base_link"
    x, y, z, roll, pitch, yaw = map(float, pose.text.split())
    pose.text = " ".join(map(str, (
        math.cos(angle) * x - math.sin(angle) * y,
        math.sin(angle) * x + math.cos(angle) * y,
        z, roll, pitch, yaw + angle)))
ET.indent(tree, space="  ")
tree.write(target / "model.sdf", encoding="utf-8", xml_declaration=True)
config = ET.parse(source / "model.config")
config.getroot().find("name").text = "transform_quad_plus"
config.getroot().find("description").text = "Motor 1 forward synchronous morphing quadrotor"
config.write(target / "model.config", encoding="utf-8", xml_declaration=True)
