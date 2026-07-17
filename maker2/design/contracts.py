"""Immutable design hardpoint contracts and legacy frame adapters."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping

from .ir import canonical_data, fingerprint


def _tuple4(matrix) -> tuple[tuple[float, ...], ...]:
    value = tuple(tuple(float(x) for x in row) for row in matrix)
    if len(value) != 4 or any(len(row) != 4 for row in value):
        raise ValueError("transform must be 4x4")
    return value


def _valid_transform(matrix) -> bool:
    try:
        m = _tuple4(matrix)
    except (TypeError, ValueError):
        return False
    if any(not math.isfinite(v) for row in m for v in row) or any(abs(a - b) > 1e-9 for a, b in zip(m[3], (0, 0, 0, 1))):
        return False
    r = [row[:3] for row in m[:3]]
    dots = [[sum(r[k][i] * r[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return all(abs(dots[i][j] - (1.0 if i == j else 0.0)) <= 1e-6 for i in range(3) for j in range(3))


@dataclass(frozen=True)
class Hardpoint:
    id: str
    sub_id: str
    role: str
    world_transform: tuple[tuple[float, ...], ...]
    local_transform: tuple[tuple[float, ...], ...]
    axis: tuple[float, float, float]
    plane: str = ""
    parameters: tuple[tuple[str, float], ...] = ()
    provenance: tuple[str, ...] = ()

    def __post_init__(self):
        object.__setattr__(self, "world_transform", _tuple4(self.world_transform))
        object.__setattr__(self, "local_transform", _tuple4(self.local_transform))
        object.__setattr__(self, "axis", tuple(float(v) for v in self.axis))


@dataclass(frozen=True)
class FunctionalEnvelope:
    sub_id: str
    minimum_m: tuple[float, float, float]
    maximum_m: tuple[float, float, float]


@dataclass(frozen=True)
class SubassemblyHardpointView:
    sub_id: str
    root_transform: tuple[tuple[float, ...], ...]
    hardpoints: tuple[Hardpoint, ...]
    envelope: FunctionalEnvelope | None
    contract_hash: str
    compiler_version: str
    catalog_version: str

    def by_role(self, role: str) -> tuple[Hardpoint, ...]:
        return tuple(h for h in self.hardpoints if h.role == role)


@dataclass(frozen=True)
class HardpointContract:
    root_transforms: tuple[tuple[str, tuple[tuple[float, ...], ...]], ...]
    hardpoints: tuple[Hardpoint, ...]
    envelopes: tuple[FunctionalEnvelope, ...]
    compiler_version: str
    catalog_version: str
    design_hash: str
    contract_hash: str = field(default="")

    def __post_init__(self):
        roots = tuple((sid, _tuple4(matrix)) for sid, matrix in self.root_transforms)
        object.__setattr__(self, "root_transforms", roots)
        payload = {"root_transforms": roots, "hardpoints": self.hardpoints,
                   "envelopes": self.envelopes, "compiler_version": self.compiler_version,
                   "catalog_version": self.catalog_version, "design_hash": self.design_hash}
        expected = fingerprint(payload, "contract_v1")
        if self.contract_hash and self.contract_hash != expected:
            raise ValueError("hardpoint contract hash does not match its contents")
        object.__setattr__(self, "contract_hash", expected)

    def validate(self) -> tuple[str, ...]:
        errors = []
        roots = dict(self.root_transforms)
        if len(roots) != len(self.root_transforms):
            errors.append("duplicate subassembly root")
        for sid, transform in self.root_transforms:
            if not sid or not _valid_transform(transform):
                errors.append(f"invalid root transform:{sid}")
        ids = set()
        for hardpoint in self.hardpoints:
            if hardpoint.id in ids:
                errors.append(f"duplicate hardpoint:{hardpoint.id}")
            ids.add(hardpoint.id)
            if hardpoint.sub_id not in roots:
                errors.append(f"unknown hardpoint subassembly:{hardpoint.id}")
            if not _valid_transform(hardpoint.world_transform) or not _valid_transform(hardpoint.local_transform):
                errors.append(f"invalid hardpoint transform:{hardpoint.id}")
            norm = math.sqrt(sum(v * v for v in hardpoint.axis))
            if abs(norm - 1.0) > 1e-6:
                errors.append(f"invalid hardpoint axis:{hardpoint.id}")
        return tuple(errors)

    def view(self, sub_id: str) -> SubassemblyHardpointView:
        roots = dict(self.root_transforms)
        if sub_id not in roots:
            raise KeyError(sub_id)
        envelope = next((e for e in self.envelopes if e.sub_id == sub_id), None)
        return SubassemblyHardpointView(sub_id, roots[sub_id],
                                        tuple(h for h in self.hardpoints if h.sub_id == sub_id),
                                        envelope, self.contract_hash,
                                        self.compiler_version, self.catalog_version)

    def to_dict(self) -> dict:
        return canonical_data(self)


def to_frame_contract(view: SubassemblyHardpointView, *, global_origin_note: str = ""):
    """Migration adapter; the frozen view remains authoritative."""
    from maker2.model import FrameContract, MountFrame
    frames = []
    for hardpoint in view.hardpoints:
        world = hardpoint.world_transform
        params = dict(hardpoint.parameters)
        frames.append(MountFrame(name=hardpoint.id, xyz_m=tuple(world[i][3] for i in range(3)),
                                 axis=hardpoint.axis, shaft_dia_mm=params.get("diameter_mm", 0.0),
                                 role=hardpoint.role))
    return FrameContract(sub_id=view.sub_id, frames=frames, global_origin_note=global_origin_note)
