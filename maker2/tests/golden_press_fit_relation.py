"""Golden relation-port press-fit exemption for compound shaft bodies.

Run: python -m maker2.tests.golden_press_fit_relation
"""
from __future__ import annotations

import tempfile
from pathlib import Path

from build123d import Align, Cylinder, export_stl

from maker2.mjcf_builder import _is_press_fit_overlap
from maker2.model import KinematicModel, LinkSpec, MateSpec, PortSpec, PoseSpec


def main():
    run = Path(tempfile.mkdtemp(prefix="golden_press_relation_"))
    meshes = run / "meshes"
    meshes.mkdir()
    # Compound shaft outer envelope is 16mm, but the named seat is exactly 3mm radius.
    shaft = Cylinder(3.0, 12.0, align=(Align.CENTER, Align.CENTER, Align.MIN)) + Cylinder(
        16.0, 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    wheel = Cylinder(20.0, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(
        2.995, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    export_stl(shaft, meshes / "shaft.stl")
    export_stl(wheel, meshes / "wheel.stl")

    links = [LinkSpec("shaft", "compound shaft", mesh_filename="meshes/shaft.stl", dof="spin"),
             LinkSpec("wheel", "wheel", mesh_filename="meshes/wheel.stl", dof="spin",
                      mount="shaft")]
    model = KinematicModel(
        "press", "shaft", links,
        [PoseSpec("place_shaft", "", "shaft"), PoseSpec("place_wheel", "shaft", "wheel")],
        ports_by_link={
            "shaft": [PortSpec("seat", "shaft", diameter_mm=6.0, depth_mm=6.0)],
            "wheel": [PortSpec("bore", "bore", diameter_mm=5.99, depth_mm=6.0)],
        },
        relations=[MateSpec("wheel_press", "press_fit", "shaft", "seat", "wheel", "bore")])
    assert _is_press_fit_overlap(model, str(meshes), links[0], links[1])
    model.relations[0].mate_type = "coaxial"
    assert not _is_press_fit_overlap(model, str(meshes), links[0], links[1])
    print("golden press-fit relation: PASS")


if __name__ == "__main__":
    main()
