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
    """One rigid part. A worker fills `mesh_filename` with built geometry.

    Pure-contact DOF (maker2-mujoco-contact): a part's motion is a property of the
    PART, not a joint. `dof` says how this link may move relative to its pose-parent:
      "fixed" -> welded to the parent/base (no DOF).
      "spin"  -> rotates on an implied axle along `spin_axis` (a hinge in MuJoCo).
      "free"  -> a free-floating body (6-DOF; settles under gravity/contact).
    `driver=True` tags the ONE part the physics test actuates on its own DOF. There
    are NO motors and NO joints between parts — transmission is by tooth contact.
    """

    name: str                                       # URDF-safe: ^[a-z][a-z0-9_]*$
    description: str                                # brief for the worker LLM
    shape_hint: str = ""                            # "cylinder" | "box" | free text
    size_mm: dict = field(default_factory=dict)     # approx bbox, {"radius":20,"height":60}
    origin_note: str = ""                           # e.g. "attach point at origin, +Z up"
    color: tuple = ()                               # display RGBA 0..1, () -> palette fallback
    mesh_filename: str = ""                         # RELATIVE "meshes/<name>.stl"
    dof: str = "fixed"                              # "fixed" | "spin" | "free"
    spin_axis: tuple = (0.0, 0.0, 1.0)             # rotation axis for dof=="spin"
    driver: bool = False                            # the ONE part the physics test drives


@dataclass
class PoseSpec:
    """A parent-relative placement EDGE — where `child`'s local origin sits relative
    to `parent`'s origin. No DOF, no limits (motion lives on LinkSpec.dof). Replaces
    the old JointSpec as the authoring contract; the legacy joint view is synthesized
    from poses+dof by KinematicModel.joints for the PyBullet/URDF path during
    migration (see maker2-mujoco-contact plan Phase 1)."""

    name: str
    parent: str                                     # LinkSpec.name ("" for a root)
    child: str                                      # LinkSpec.name
    xyz_m: tuple = (0.0, 0.0, 0.0)                  # origin translation, METERS
    rpy_rad: tuple = (0.0, 0.0, 0.0)               # origin rotation, radians


@dataclass
class JointSpec:
    """LEGACY kinematic edge. No longer authored by the manager — synthesized from
    PoseSpec+LinkSpec.dof by KinematicModel.joints so the URDF builder + PyBullet
    runner keep working during the MuJoCo migration. Phase 4 retires it from sim."""

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
    driver: bool = False                            # the INPUT joint a user drives


@dataclass
class KinematicModel:
    """The manager's decomposition: a FOREST of links placed by parent-relative
    poses. Pure contact — no joint tree, no single-root requirement. `mesh_pairs`
    names the gear pairs MEANT to couple by tooth contact (the transmission-fail
    detector needs this, since no joint encodes a mesh)."""

    name: str
    root_link: str
    links: list[LinkSpec] = field(default_factory=list)
    poses: list[PoseSpec] = field(default_factory=list)
    mesh_pairs: list = field(default_factory=list)  # [(drive_link, driven_link), ...]
    # Hierarchy side-channel: when a subassembly manager builds under a boss frame
    # contract, its realized interface-frame placements land here (not part of the
    # URDF contract; model_to_dict/_validate_model ignore it). See maker2/boss.py.
    frames_realized: list = field(default_factory=list, compare=False, repr=False)

    def link_by_name(self, name: str) -> "LinkSpec | None":
        return next((l for l in self.links if l.name == name), None)

    def pose_to_child(self, child_name: str) -> "PoseSpec | None":
        return next((p for p in self.poses if p.child == child_name), None)

    def child_poses(self, parent_name: str) -> list["PoseSpec"]:
        return [p for p in self.poses if p.parent == parent_name]

    @property
    def joints(self) -> list["JointSpec"]:
        """LEGACY VIEW: synthesize URDF/PyBullet joints from poses + link dof so the
        old sim/URDF path keeps working during the MuJoCo migration. A pose whose
        CHILD link is dof=="spin" becomes a continuous joint on that link's spin_axis;
        dof=="free" also maps to continuous (best-effort; the real 6-DOF lives in
        MuJoCo); dof=="fixed" is a fixed joint. Poses with an empty parent (forest
        roots) are skipped — the URDF builder welds those to the base separately."""
        out: list[JointSpec] = []
        dof_by_link = {l.name: l for l in self.links}
        for p in self.poses:
            if not p.parent:
                continue
            link = dof_by_link.get(p.child)
            d = link.dof if link else "fixed"
            axis = tuple(link.spin_axis) if link else (0.0, 0.0, 1.0)
            jtype = "continuous" if d in ("spin", "free") else "fixed"
            out.append(JointSpec(
                name=p.name, type=jtype, parent=p.parent, child=p.child,
                xyz_m=tuple(p.xyz_m), rpy_rad=tuple(p.rpy_rad),
                axis=axis if jtype != "fixed" else (0.0, 0.0, 1.0),
                driver=bool(link.driver) if link else False))
        return out

    def child_joints(self, parent_name: str) -> list["JointSpec"]:
        return [j for j in self.joints if j.parent == parent_name]


# --------------------------------------------------------------------------- #
# Hierarchy contract (boss -> managers -> assembler). A BOSS splits a big
# machine into SUBASSEMBLIES (each one manager's job) and authors the INTERFACE/
# FRAME CONTRACT: named mount frames in GLOBAL coordinates + the SEAMS that join
# subassemblies (welds, and gear-mesh/power couplings). The assembler stitches
# the per-subassembly KinematicModels into one final KinematicModel using this.
# See maker2/boss.py and the plan at .claude/plans/precious-humming-wand.md.
# --------------------------------------------------------------------------- #

@dataclass
class MountFrame:
    """A named interface frame a subassembly exposes, in GLOBAL METERS. The boss
    fixes where it is; the manager must place a real link there (reported back in
    `frames_realized`). `role`: mount (structural), power_in/power_out (a shaft
    end), mesh (a gear whose teeth couple across a seam).

    For a power_in/power_out/mesh frame the boss also fixes the HARD interface
    geometry the managers must not offset: `axis` is the shaft/gear axis and
    `shaft_dia_mm` the shaft (or gear pitch) diameter, so both sides size a mating
    shaft/bore/gear to the SAME numbers. These are the immovable hard points of the
    interface contract (Session B item 1a)."""

    name: str
    xyz_m: tuple = (0.0, 0.0, 0.0)                   # GLOBAL translation, METERS
    rpy_rad: tuple = (0.0, 0.0, 0.0)                # GLOBAL rotation, radians
    axis: tuple = (0.0, 0.0, 1.0)                   # frame primary axis (e.g. shaft axis)
    shaft_dia_mm: float = 0.0                        # hard shaft/gear-pitch diameter (MM),
                                                     # 0 = not a shaft/gear interface
    link: str = ""                                   # realized link (filled by the manager)
    role: str = "mount"                              # mount|power_in|power_out|mesh


@dataclass
class SubassemblySpec:
    """One subassembly the boss carves out: a brief for its manager + the frames
    it must expose. `est_link_budget` keeps each manager under the output cap."""

    id: str                                          # URDF-safe slug, unique
    brief: str                                       # the manager's product prompt
    function: str = ""                               # what it does (for planning)
    frames: list = field(default_factory=list)      # list[MountFrame], GLOBAL coords
    input_tags: list = field(default_factory=list)  # frame names that are power inputs
    output_tags: list = field(default_factory=list) # frame names that are power outputs
    est_link_budget: int = 30                        # keep <=35 so one manager fits
    instances: list = field(default_factory=list)   # for IDENTICAL repeated subs (4 rotors,
                                                     # 6 legs): [{xyz_m:[3], rpy_rad:[3]}, ...]
                                                     # one GLOBAL pose per copy. Built ONCE;
                                                     # the assembler instantiates N times.
                                                     # empty/len<=1 = a single instance.


@dataclass
class SeamSpec:
    """How two subassemblies join. `kind`:
      "weld"  -> a fixed structural joint parent_frame -> child_frame.
      "power" -> a motion crossing (gear mesh or shared shaft). For a gear MESH
                 seam the structural link is still a weld between the housings;
                 `mesh_pair` names the (drive_link, driven_link) that couple by
                 tooth contact, and the geometric pre-check verifies their center
                 distance == summed pitch radii."""

    id: str
    kind: str                                        # "weld" | "power"
    parent_sub: str
    parent_frame: str
    child_sub: str
    child_frame: str
    joint_type: str = "fixed"                         # fixed for weld; continuous/revolute for a shared-DOF power seam
    axis: tuple = (0.0, 0.0, 1.0)
    lower: float | None = None
    upper: float | None = None
    effort: float = 10.0
    velocity: float = 1.0
    driver: bool = False                             # is this the machine's single power input?
    owner_sub: str = ""                              # for a power seam, which sub owns the driving link
    mesh_pair: tuple = ()                            # (drive_link, driven_link) for a gear-mesh seam


@dataclass
class SubassemblyPlan:
    """The boss's decomposition: subassemblies + the seams that connect them into
    ONE machine rooted at `root_sub` about a single global origin."""

    name: str
    root_sub: str
    global_origin_note: str = ""
    subassemblies: list = field(default_factory=list)   # list[SubassemblySpec]
    seams: list = field(default_factory=list)           # list[SeamSpec]

    def sub_by_id(self, sub_id: str) -> "SubassemblySpec | None":
        return next((s for s in self.subassemblies if s.id == sub_id), None)


@dataclass
class FrameContract:
    """What one manager is handed: its subassembly's frames (GLOBAL coords) + the
    shared global origin, so it places the declared frames at the declared spots."""

    sub_id: str
    frames: list                                     # list[MountFrame]
    global_origin_note: str = ""
    input_tags: list = field(default_factory=list)
    output_tags: list = field(default_factory=list)
    neighbors: list = field(default_factory=list)    # [{id, function, brief}] of the
                                                     # OTHER subs, so this manager knows
                                                     # what the rest of the machine builds
    appearance_summary: str = ""                     # optional coarse whole-machine
                                                     # layout text (1c), for proportion
                                                     # context; "" when disabled


@dataclass
class SubResult:
    """One subassembly's build outcome (Stage B produces; the assembler + loop
    consume). `sub_frames` is the manager's realized frame placements."""

    id: str
    ctx: object = None                               # RunContext for this sub's run dir
    model: object = None                             # KinematicModel
    results: list = field(default_factory=list)      # list[WorkerResult]
    sub_frames: list = field(default_factory=list)   # realized frames [{frame, link, local_xyz_m, local_rpy_rad}]
    ok: bool = False
    error: str = ""


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

