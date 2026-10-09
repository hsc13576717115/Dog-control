#!/usr/bin/env python3
"""M1 fixtures only; these pads are not a complete competition course."""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def fixture(height, mode="pads"):
    if height not in (0.0, 0.03, 0.05):
        raise ValueError("M1 pad height must be 0/0.03/0.05 m")
    spec = {
        "version": "qr-m1-v1",
        "height": height,
        "foot_radius": 0.026,
        "pads": [
            {"id": 1, "center": [0.0, -0.18], "size": [0.2, 0.2]},
            {"id": 2, "center": [0.0, 0.18], "size": [0.2, 0.2]},
        ],
    }

    if mode == "ground":
        if height != 0:
            raise ValueError("ground fixture height must be zero")
        spec["pads"] = [
            {"id": 0, "center": [0.0, 0.0], "size": [4.0, 4.0], "height": 0.0}
        ]
    elif mode == "platform":
        if height not in (0.03, 0.05):
            raise ValueError("M2 local platform height must be 30/50 mm")
        spec["version"] = "qr-m2-platform-v1"
        spec["scope"] = "low platform transfer, not a rule obstacle"
        spec["pads"] = [
            {"id": 0, "center": [-0.85, 0.0], "size": [2.3, 4.0], "height": 0.0},
            {"id": 1, "center": [0.7, 0.0], "size": [0.8, 0.8], "height": height},
        ]
    elif mode != "pads":
        raise ValueError("unknown fixture mode")
    return spec


def generate(height, output, mode="pads"):
    spec = fixture(height, mode)
    root = ET.Element("sdf", version="1.6")
    world = ET.SubElement(root, "world", name="qr_m1")
    for uri in ["model://sun", "model://ground_plane"]:
        ET.SubElement(ET.SubElement(world, "include"), "uri").text = uri
    physics = ET.SubElement(world, "physics", name="default", type="ode")
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_update_rate").text = "1000"
    plugin = ET.SubElement(
        world, "plugin", name="evaluation_state", filename="libgazebo_ros_state.so"
    )
    ros = ET.SubElement(plugin, "ros")
    ET.SubElement(ros, "namespace").text = "/evaluation"
    ET.SubElement(plugin, "update_rate").text = "100"
    for pad in spec["pads"]:
        pad_height = pad.get("height", height)
        if pad_height == 0:
            continue
        model = ET.SubElement(world, "model", name=f"pad_{pad['id']}")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = (
            f"{pad['center'][0]} {pad['center'][1]} {pad_height/2} 0 0 0"
        )
        link = ET.SubElement(model, "link", name="support")
        for kind in ["collision", "visual"]:
            item = ET.SubElement(link, kind, name=kind)
            box = ET.SubElement(ET.SubElement(item, "geometry"), "box")
            ET.SubElement(box, "size").text = (
                f"{pad['size'][0]} {pad['size'][1]} {pad_height}"
            )
    ET.indent(root)
    Path(output).write_text(ET.tostring(root, encoding="unicode"))
    Path(str(output) + ".json").write_text(json.dumps(spec, indent=2))
    return spec


def audit(xml, rules):
    root = ET.parse(xml).getroot()
    rules = yaml.safe_load(Path(rules).read_text())
    geoms = {x.get("name"): x for x in root.findall("./worldbody/geom")}
    xs = sorted(
        float(g.get("pos").split()[0])
        for n, g in geoms.items()
        if n.startswith("stepping_stones_01_0")
        and float(g.get("pos").split()[1]) == 1.9
    )
    spacing = xs[1] - xs[0] if len(xs) > 1 else None
    expected = rules["obstacles"]["stones"]["same_row_clear_gap"] + 0.2
    missing = [
        x.get("file")
        for x in root.iter()
        if x.get("file") and not (Path(xml).parent / x.get("file")).exists()
    ]
    return {
        "source_sha256": hashlib.sha256(Path(xml).read_bytes()).hexdigest(),
        "missing_assets": missing,
        "stone_center_spacing_m": spacing,
        "rule_center_spacing_m": expected,
        "dimensions_match": spacing is not None and abs(spacing - expected) < 1e-6,
        "robot_present": root.find(".//joint") is not None,
        "competition_validated": False,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--height", type=float, default=0.0)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    generate(a.height, a.output)
