"""check_stl: the authoritative per-link success gate.

yourdfpy only *warns* on a missing or 0-byte mesh, so the URDF assembling
"successfully" proves nothing about whether a worker actually filled its STL.
This module reads each STL back with trimesh and is the single source of truth
for "did this link get built": the file exists, is non-empty, loads, has faces,
and is not degenerate (collapsed to a point or zero area). Watertightness is
recorded for information but NOT required — a valid open-shell export still
renders and assembles.
"""

from __future__ import annotations

import os

from .model import StlReport


# A mesh whose largest bounding-box extent is below this (in mm) is treated as a
# collapsed point/line rather than a real part.
_MIN_EXTENT_MM = 1e-6
_MIN_AREA = 1e-9


def check_stl(path: str) -> StlReport:
    """Validate one exported STL and return a structured StlReport."""
    report = StlReport()

    if not path or not os.path.isfile(path):
        report.error = "STL file does not exist"
        return report
    report.exists = True

    report.size_bytes = os.path.getsize(path)
    if report.size_bytes == 0:
        report.error = "STL file is empty (0 bytes)"
        return report

    try:
        import trimesh
        mesh = trimesh.load(path, force="mesh")
    except Exception as e:
        report.error = f"trimesh could not load STL: {type(e).__name__}: {e}"
        return report

    try:
        num_faces = int(len(mesh.faces))
        num_vertices = int(len(mesh.vertices))
        extents = mesh.extents
        area = float(mesh.area)
        watertight = bool(mesh.is_watertight)
    except Exception as e:
        report.error = f"loaded object is not a usable mesh: {type(e).__name__}: {e}"
        return report

    report.loadable = True
    report.num_faces = num_faces
    report.num_vertices = num_vertices
    report.watertight = watertight

    if num_faces == 0 or extents is None:
        report.bbox_mm = (0.0, 0.0, 0.0)
        report.error = "mesh has no faces / no spatial extent"
        return report

    ex = tuple(float(v) for v in extents)
    report.bbox_mm = ex

    if max(ex) <= _MIN_EXTENT_MM or area <= _MIN_AREA:
        report.degenerate = True
        report.error = (f"degenerate geometry (faces={num_faces}, "
                        f"area={area:.3g}, extents={ex})")
        return report

    report.degenerate = False
    return report
