"""Golden support-MJCF closure: keep valid linkage pins, reject invented ones.

Run: python -m maker2.tests.golden_support_relations
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from maker2.mjcf_builder import _validated_support_relations
from maker2.model import KinematicModel, LinkSpec, MateSpec, PortSpec, PoseSpec


def _model(*, small_xyz=(40.0 * math.cos(0.4), -40.0 * math.sin(0.4), 0.0),
           small_diameter=6.1):
    links = [
        LinkSpec("crank", "crank", dof="spin", driver=True),
        LinkSpec("rod", "rod", dof="free"),
        LinkSpec("piston", "piston", dof="slide"),
    ]
    poses = [
        PoseSpec("place_crank", "", "crank", (0.0, 0.0, 0.0)),
        # Rotated rod catches the world-to-local site conversion regression.
        PoseSpec("place_rod", "", "rod", (0.0, 0.0, 0.0), (0.0, 0.0, 0.4)),
        PoseSpec("place_piston", "", "piston", (0.04, 0.0, 0.0)),
    ]
    ports = {
        "crank": [PortSpec("pin", "shaft", (0.0, 0.0, 0.0), diameter_mm=6.0,
                           depth_mm=4.0)],
        "rod": [
            PortSpec("big", "bore", (0.0, 0.0, 0.0), diameter_mm=6.1,
                     depth_mm=4.0),
            PortSpec("small", "bore", small_xyz, diameter_mm=small_diameter,
                     depth_mm=4.0),
        ],
        "piston": [PortSpec("wrist", "shaft", (0.0, 0.0, 0.0), diameter_mm=6.0,
                            depth_mm=4.0)],
    }
    relations = [
        MateSpec("big_pin", "revolute", "crank", "pin", "rod", "big"),
        MateSpec("small_pin", "revolute", "piston", "wrist", "rod", "small"),
        # Bearing-like ideal support must never be retained by this support helper.
        MateSpec("fake_bearing", "journal_bearing", "crank", "pin", "rod", "big"),
    ]
    return KinematicModel("support_linkage", "crank", links, poses,
                          ports_by_link=ports, relations=relations)


def _world(model):
    world = ET.Element("worldbody")
    for link in model.links:
        ET.SubElement(world, "body", {"name": link.name})
    return world


def main():
    model = _model()
    xml = ET.Element("mujoco")
    count, accepted, rejected = _validated_support_relations(
        model, xml, _world(model), log_fn=lambda *_: None)
    assert count == 2, (count, rejected)
    assert len(accepted) == 2, accepted
    assert not rejected, rejected
    connects = xml.findall("./equality/connect")
    assert len(connects) == 2

    bad = _model(small_xyz=(80.0, 0.0, 0.0), small_diameter=8.0)
    bad_xml = ET.Element("mujoco")
    count, accepted, rejected = _validated_support_relations(
        bad, bad_xml, _world(bad), log_fn=lambda *_: None)
    assert count == 1, (count, rejected)
    assert len(rejected) == 1 and rejected[0]["relation"] == "small_pin", rejected
    assert "differ" in rejected[0]["reason"] or "diameter" in rejected[0]["reason"]

    print("golden support relations: PASS")


if __name__ == "__main__":
    main()
