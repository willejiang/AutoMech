"""Golden checks for real-solid overlap with non-watertight annular exports.
Run: python -m maker2.tests.golden_solid_overlap
"""
from __future__ import annotations

import numpy as np
import trimesh

from maker2.precheck import _solid_intersection_frac


def annular_clearance():
    housing=trimesh.creation.annulus(r_min=11.25,r_max=16.0,height=40.0)
    spacer=trimesh.creation.annulus(r_min=6.0,r_max=6.4,height=16.0)
    # Reproduce the CadQuery STL pathology: a valid annulus represented by two open shells.
    spacer.update_faces(spacer.face_normals[:,2]==0)
    spacer.remove_unreferenced_vertices()
    assert housing.is_watertight and not spacer.is_watertight
    assert _solid_intersection_frac(housing,spacer)<.01


def real_collision():
    housing=trimesh.creation.annulus(r_min=11.25,r_max=16.0,height=40.0)
    rod=trimesh.creation.cylinder(radius=6.4,height=16.0)
    rod.apply_translation((9.0,0.0,0.0))
    assert _solid_intersection_frac(housing,rod)>.30


if __name__=='__main__':
    annular_clearance();real_collision()
    print('golden real-solid overlap: PASS')
