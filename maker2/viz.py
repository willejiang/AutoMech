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
    """Load the URDF with meshes + scene graph (the assembled product).

    Pin the mesh filename handler to the URDF's OWN directory so a relative
    ``meshes/*.stl`` resolves against the sub's run dir, NOT the process CWD. Without
    this, loading from a different CWD (e.g. the web app's repo root) fails every mesh
    ('Unable to resolve filename'), yielding a robot with no geometry — which silently
    breaks the conflict subcheck (it then compares empty meshes and reports a bogus,
    unchanging overlap that no debugger pose edit can clear)."""
    import os
    base = os.path.dirname(os.path.abspath(urdf_path))
    return yourdfpy.URDF.load(urdf_path, load_meshes=True,
                              build_scene_graph=True, force_mesh=True,
                              filename_handler=lambda fname: os.path.join(base, fname))


# Fallback palette (RGB 0..255) used ONLY for a mesh that somehow loaded with no
# per-face color. Normally the URDF carries a <material> per link (authored by the
# manager, or a builder palette fallback), and yourdfpy applies it on load — so
# this is just a last resort to avoid a stray gray part.
_PALETTE = [
    (210, 70, 60), (70, 130, 200), (90, 180, 90), (230, 170, 50),
    (150, 100, 200), (60, 190, 190), (220, 120, 170), (160, 160, 80),
    (120, 110, 100), (200, 200, 210), (180, 90, 60), (80, 150, 120),
]

# A near-gray default trimesh assigns when a mesh has no material; if every face
# matches it, the link rendered colorless and we substitute a palette color.
_DEFAULT_GRAY = (102, 102, 102)


def _has_real_color(geom) -> bool:
    fc = getattr(getattr(geom, "visual", None), "face_colors", None)
    if fc is None or len(fc) == 0:
        return False
    r, g, b = (int(x) for x in fc[0][:3])
    return (r, g, b) != _DEFAULT_GRAY


def _colorize_fallback(robot) -> None:
    """Tint only meshes that loaded WITHOUT a real material color.

    The URDF normally supplies a per-link <material>, so most parts already have
    their color by the time this runs. This just rescues any part that came in as
    the default gray, so the judge never sees an ambiguous colorless link."""
    import numpy as _np
    for i, (name, geom) in enumerate(sorted(robot.scene.geometry.items())):
        if _has_real_color(geom):
            continue
        try:
            c = _PALETTE[i % len(_PALETTE)]
            rgba = _np.array([c[0], c[1], c[2], 255], dtype=_np.uint8)
            geom.visual.face_colors = _np.tile(rgba, (len(geom.faces), 1))
        except Exception:
            pass


def _frame_isometric(scene) -> None:
    """Aim the scene camera at the geometry from an isometric angle, fit to bounds."""
    scene.set_camera(angles=_ISO_ANGLES)


def show(urdf_path: str) -> None:
    """Open an interactive viewer of the assembled product (needs a display)."""
    robot = load_robot(urdf_path)
    _frame_isometric(robot.scene)
    robot.scene.show()


def _save_image_retry(scene, resolution, attempts: int = 4):
    """Render the scene to a PNG offscreen, retrying transient GL glitches.

    trimesh's offscreen path occasionally raises ZeroDivisionError (a race where
    the hidden window reports height=0 in on_resize) or a GL error; these are
    transient, so a few retries turn a flaky render into a reliable one and keep
    the judge from falling back to a blind text-only verdict."""
    last = None
    for _ in range(attempts):
        try:
            png = scene.save_image(resolution=resolution, visible=False)
            if png and len(png) > 512:
                return png
        except Exception as e:  # ZeroDivisionError, GL errors, window races
            last = e
    if last is not None:
        raise last
    raise RuntimeError("save_image returned empty output")


def render_png(urdf_path: str, out_path: str,
               resolution: tuple[int, int] = (1280, 960)) -> str:
    """Render the assembled product to a PNG (needs an OpenGL context)."""
    robot = load_robot(urdf_path)
    _colorize_fallback(robot)
    _frame_isometric(robot.scene)
    png = _save_image_retry(robot.scene, resolution)
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
    _colorize_fallback(robot)
    os.makedirs(out_dir, exist_ok=True)
    paths: dict[str, str] = {}
    for name, angles in _SIX_VIEWS.items():
        robot.scene.set_camera(angles=angles)
        try:
            # Offscreen + retry: an on-screen window (visible=True) is racy on
            # headless Windows and a transient ZeroDivisionError/GL glitch would
            # otherwise drop the judge to a blind text-only verdict. Render each
            # view robustly; if one view still fails, keep the others (the judge
            # only needs a few good views, not all six).
            png = _save_image_retry(robot.scene, resolution)
        except Exception as e:
            print(f"[viz] view '{name}' failed after retries: {e}")
            continue
        out_path = os.path.join(out_dir, f"view_{name}.png")
        with open(out_path, "wb") as f:
            f.write(png)
        paths[name] = out_path
    return paths
