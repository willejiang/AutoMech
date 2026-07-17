"""Solver-neutral cross-subassembly constraint IR (meters/radians externally)."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class EntityKind(str, Enum):
    POINT_3D = "point_3d"
    LINE_3D = "line_3d"


class ConstraintKind(str, Enum):
    COINCIDENT = "coincident"
    DISTANCE = "distance"
    POINT_ON_LINE = "point_on_line"
    PROJECTED_DISTANCE = "projected_distance"


@dataclass
class EntitySpec:
    id: str
    kind: EntityKind
    initial_m: tuple = ()
    refs: tuple = ()
    fixed: bool = False
    provenance: dict = field(default_factory=dict)


@dataclass
class ConstraintSpec:
    id: str
    kind: ConstraintKind
    entities: tuple
    value_m: float = 0.0
    enforced_by_solver: bool = True
    provenance: dict = field(default_factory=dict)


@dataclass
class RigidStageSpec:
    sub_id: str
    seed_transform: list
    local_anchor_m: tuple
    local_axis: tuple
    target_anchor_m: tuple
    target_axis: tuple
    gear_centers_local_m: dict = field(default_factory=dict)
    mount_seam_id: str = ""
    housing_frame: str = ""
    shaft_frame: str = ""


@dataclass
class AssemblyConstraintProblem:
    stages: dict = field(default_factory=dict)
    entities: list = field(default_factory=list)
    constraints: list = field(default_factory=list)
    gear_pairs: list = field(default_factory=list)
    expected_dof: int = 0
    units: str = "m"
    base_id: str = ""
    diagnostics: dict = field(default_factory=dict)


@dataclass
class ConstraintSolveResult:
    status: str
    raw_status: int = -1
    dof: int = -1
    points_m: dict = field(default_factory=dict)
    failed_constraint_ids: list = field(default_factory=list)
    diagnostics: dict = field(default_factory=dict)


@dataclass
class PlacementResult:
    placements: dict
    solve: ConstraintSolveResult
    residuals: dict = field(default_factory=dict)
    backend: str = "slvs"
