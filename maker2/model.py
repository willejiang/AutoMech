"""The data contract shared by manager, workers, URDF builder, and orchestrator.

These dataclasses ARE the integration contract. The manager produces a
KinematicModel (links + joints); the URDF builder turns it into model.urdf and
scaffolds one empty meshes/<link>.stl per link; each worker fills exactly one
of those STLs.

Units convention (load-bearing — this is what makes independently-built meshes
line up with the manager's joint origins):

  * Workers build geometry in MILLIMETERS, in the link's LOCAL frame, with the
    link's joint-attachment point at the local origin (0, 0, 0), primary axis
    along +Z unless `origin_note` says otherwise.
  * STL files are unitless; the exported numbers ARE the mm values.
  * The URDF references each mesh with scale=(0.001, 0.001, 0.001) so the mm
    geometry renders at meter scale.
  * The manager authors every JOINT ORIGIN (`xyz_m`) in METERS, as the vector
    from the parent link's origin to where the child link's origin attaches.

Workers never position parts relative to siblings — all spatial relationships
live in the joints, which are 100% the manager's responsibility.
"""

from __future__ import annotations

from dataclasses import dataclass, field


# The single source of truth for the units/origin rule, injected verbatim into
# every worker prompt so each worker builds against the exact same convention
# the manager assumed when authoring joint origins.
UNITS_CONVENTION = (
    "Build the part in MILLIMETERS using FreeCAD's native units. Work in the "
    "part's OWN local frame with its joint-attachment point at the origin "
    "(0, 0, 0); orient the primary axis along +Z unless told otherwise. Do NOT "
    "offset the part to position it next to other parts — assembly placement is "
    "handled separately via joint origins. Export exactly one solid."
)


@dataclass
class LinkSpec:
    """One rigid part. A worker fills `mesh_filename` with built geometry."""

    name: str                                       # URDF-safe: ^[a-z][a-z0-9_]*$
    description: str                                # brief for the worker LLM
    shape_hint: str = ""                            # "cylinder" | "box" | free text
    size_mm: dict = field(default_factory=dict)     # approx bbox, {"radius":20,"height":60}
    origin_note: str = ""                           # e.g. "attach point at origin, +Z up"
    mesh_filename: str = ""                         # RELATIVE "meshes/<name>.stl"


@dataclass
class JointSpec:
    """One joint connecting parent -> child. Authored entirely by the manager."""

    name: str
    type: str                                       # fixed|revolute|prismatic|continuous
    parent: str                                     # LinkSpec.name
    child: str                                      # LinkSpec.name
    xyz_m: tuple = (0.0, 0.0, 0.0)                  # origin translation, METERS
    rpy_rad: tuple = (0.0, 0.0, 0.0)               # origin rotation, radians
    axis: tuple = (0.0, 0.0, 1.0)                  # joint axis (non-fixed types)
    lower: float | None = None                      # revolute/prismatic limit (rad/m)
    upper: float | None = None
    effort: float = 10.0
    velocity: float = 1.0


@dataclass
class KinematicModel:
    """The manager's decomposition: a single-rooted tree of links + joints."""

    name: str
    root_link: str
    links: list[LinkSpec] = field(default_factory=list)
    joints: list[JointSpec] = field(default_factory=list)

    def link_by_name(self, name: str) -> "LinkSpec | None":
        return next((l for l in self.links if l.name == name), None)

    def child_joints(self, parent_name: str) -> list["JointSpec"]:
        return [j for j in self.joints if j.parent == parent_name]


@dataclass
class StlReport:
    """Result of validating one STL file (see validation.check_stl)."""

    exists: bool = False
    size_bytes: int = 0
    loadable: bool = False
    num_faces: int = 0
    num_vertices: int = 0
    bbox_mm: tuple = (0.0, 0.0, 0.0)               # extents along x, y, z
    watertight: bool = False                        # recorded, not required
    degenerate: bool = True                         # True until proven otherwise
    error: str = ""

    @property
    def ok(self) -> bool:
        return (self.exists and self.size_bytes > 0 and self.loadable
                and self.num_faces > 0 and not self.degenerate)

    def summary(self) -> str:
        """One-line human/LLM-readable summary for retry feedback."""
        if self.ok:
            bx, by, bz = self.bbox_mm
            return (f"OK: {self.num_faces} faces, "
                    f"bbox~{bx:.1f}x{by:.1f}x{bz:.1f} mm, "
                    f"watertight={self.watertight}")
        if not self.exists:
            return "FAIL: STL file was not created"
        if self.size_bytes == 0:
            return "FAIL: STL file is empty (0 bytes)"
        if not self.loadable:
            return f"FAIL: STL not loadable ({self.error})"
        if self.degenerate:
            return "FAIL: STL geometry is degenerate (zero extent or area)"
        return f"FAIL: {self.error or 'unknown'}"


@dataclass
class WorkerTask:
    """One unit of work handed to a worker: build the STL for `link`."""

    link: LinkSpec
    abs_mesh_path: str                              # native Windows abs path to fill
    units_convention: str = UNITS_CONVENTION


@dataclass
class WorkerResult:
    """Outcome of a worker's build+validate+retry loop for one link."""

    link_name: str
    success: bool
    attempts: int = 0
    abs_mesh_path: str = ""
    error: str = ""
    stl_report: "StlReport | None" = None
    code: str = ""                                  # final FreeCAD body the worker ran


@dataclass
class RunContext:
    """Absolute (native Windows) paths for one orchestrator run."""

    project_slug: str
    run_dir: str
    urdf_path: str
    meshes_dir: str
    logs_dir: str
    model_json_path: str


@dataclass
class JudgeVerdict:
    """The evaluator's verdict on one generated CAD (saved as judge.json).

    ``passed`` ends the generate->judge->refine loop; when it is False,
    ``suggestions`` is the concrete change list fed back to the manager for the
    next iteration. ``raw`` keeps the model's full parsed JSON for the record.
    """

    passed: bool
    reasons: str = ""
    suggestions: str = ""
    raw: dict = field(default_factory=dict)

