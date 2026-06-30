"""Visualize the assembled URDF with yourdfpy + trimesh.

``URDF.load`` resolves ``meshes/*.stl`` relative to the URDF directory and
assembles them per the joint transforms, so loading the post-fill URDF *is* the
assembly. The static milestone renders at default joint angles (one coherent
product); articulation is a later seam via ``robot.update_cfg({...})``.

Interactive display and PNG rendering both need an OpenGL context (pyglet); on a
headless box they will fail, which is why the orchestrator exposes --no-viz.
"""

from __future__ import annotations

import os

import numpy as np
import yourdfpy


# A 3/4 isometric view: tilt down ~60deg about X, spin 45deg about Z. The default
# camera (angles all zero) looks straight down -Z, which renders a flat top face
# as a featureless fill — useless for a preview. set_camera auto-fits the camera
# distance to the scene bounds when distance is left None.
_ISO_ANGLES = (np.deg2rad(60.0), 0.0, np.deg2rad(45.0))


def load_robot(urdf_path: str) -> "yourdfpy.URDF":
    """Load the URDF with meshes + scene graph (the assembled product)."""
    return yourdfpy.URDF.load(urdf_path, load_meshes=True,
                              build_scene_graph=True, force_mesh=True)


def _frame_isometric(scene) -> None:
    """Aim the scene camera at the geometry from an isometric angle, fit to bounds."""
    scene.set_camera(angles=_ISO_ANGLES)


def show(urdf_path: str) -> None:
    """Open an interactive viewer of the assembled product (needs a display)."""
    robot = load_robot(urdf_path)
    _frame_isometric(robot.scene)
    robot.scene.show()


def render_png(urdf_path: str, out_path: str,
               resolution: tuple[int, int] = (1280, 960)) -> str:
    """Render the assembled product to a PNG (needs an OpenGL context)."""
    robot = load_robot(urdf_path)
    _frame_isometric(robot.scene)
    png = robot.scene.save_image(resolution=resolution, visible=True)
    with open(out_path, "wb") as f:
        f.write(png)
    return out_path


# Six axis-aligned camera presets (euler angles, radians) for the judge's views.
# rx tilts the camera (0 = look down -Z = top; pi/2 = horizontal; pi = bottom);
# the third angle orbits the four horizontal views 90deg apart. set_camera auto-
# fits the distance to the scene bounds. The labels follow the usual CAD
# convention closely enough -- what matters is six distinct orthogonal sides for
# the evaluator to inspect.
_SIX_VIEWS = {
    "front":  (np.pi / 2, 0.0, 0.0),
    "left":   (np.pi / 2, 0.0, np.pi / 2),
    "back":   (np.pi / 2, 0.0, np.pi),
    "right":  (np.pi / 2, 0.0, -np.pi / 2),
    "top":    (0.0, 0.0, 0.0),
    "bottom": (np.pi, 0.0, 0.0),
}


def render_six_views(urdf_path: str, out_dir: str,
                     resolution: tuple[int, int] = (1024, 768)) -> dict[str, str]:
    """Render the assembled product from 6 axis-aligned views to PNG files.

    Returns ``{view_name: png_path}`` for the views written (front/back/left/
    right/top/bottom). Loads the URDF once and re-aims the same scene camera per
    view. Like render_png this needs an OpenGL context; the caller treats any
    failure as "no images" and degrades to a text-only judge.
    """
    robot = load_robot(urdf_path)
    os.makedirs(out_dir, exist_ok=True)
    paths: dict[str, str] = {}
    for name, angles in _SIX_VIEWS.items():
        robot.scene.set_camera(angles=angles)
        png = robot.scene.save_image(resolution=resolution, visible=True)
        out_path = os.path.join(out_dir, f"view_{name}.png")
        with open(out_path, "wb") as f:
            f.write(png)
        paths[name] = out_path
    return paths
