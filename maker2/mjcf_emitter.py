"""Restricted output API used by agent-authored MJCF compiler scripts."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np


def _fmt(values) -> str:
    return " ".join(f"{float(x):.10g}" for x in values)


def _quat_wxyz(matrix) -> list[float]:
    import trimesh.transformations as tf
    m = np.eye(4); m[:3, :3] = matrix[:3, :3]
    return [float(x) for x in tf.quaternion_from_matrix(m)]


class MJCFEmitter:
    """Mechanical-policy-free MJCF writer.

    It supplies measured mass/mesh/frame data from facts. The compiler script alone chooses
    body parents, coordinates, constraints, transmissions and contact exclusions.
    """

    def __init__(self, facts: dict):
        self.facts = facts
        sim = facts["simulation"]
        self.root = ET.Element("mujoco", {"model": facts["model"].get("name") or "assembly"})
        ET.SubElement(self.root, "compiler", {"angle": "radian", "coordinate": "local"})
        ET.SubElement(self.root, "option", {
            "gravity": _fmt(sim["gravity"]), "timestep": f"{sim['timestep']:.10g}",
            "solver": sim["solver"], "iterations": str(sim["iterations"]),
            "cone": "elliptic"})
        ET.SubElement(self.root, "size", {"memory": "256M"})
        self.asset = ET.SubElement(self.root, "asset")
        self.world = ET.SubElement(self.root, "worldbody")
        ET.SubElement(self.world, "light", {"pos": "0 0 3", "dir": "0 0 -1"})
        self.ground = ET.SubElement(self.world, "geom", {
            "name": "ground", "type": "plane", "size": "5 5 .1",
            "friction": "1 .05 .005", "contype": "1", "conaffinity": "0"})
        self.equality = ET.SubElement(self.root, "equality")
        self.contact = ET.SubElement(self.root, "contact")
        self.bodies: dict[str, ET.Element] = {}
        self.geoms: dict[str, ET.Element] = {}
        self.joints: dict[str, ET.Element] = {}
        self.constraints: dict[str, ET.Element] = {}
        self.decisions: list[dict] = []
        self.excludes: list[dict] = []
        self.ground_excludes: list[dict] = []
        self.support_patches: list[dict] = []
        self._topology_plan: dict | None = None

    def topology_plan(self, plan: dict):
        if not isinstance(plan, dict) or not plan:
            raise ValueError("topology_plan must be a non-empty object")
        self._topology_plan = plan

    def body(self, name: str, parent: str = ""):
        if name in self.bodies:
            raise ValueError(f"duplicate body '{name}'")
        link = self.facts["links"].get(name)
        if link is None:
            raise ValueError(f"unknown body '{name}'")
        parent_el = self.world if not parent else self.bodies.get(parent)
        if parent_el is None:
            raise ValueError(f"body '{name}' parent '{parent}' has not been emitted")
        world = np.asarray(link["world_frame"]["matrix"], dtype=float)
        if parent:
            parent_world = np.asarray(self.facts["links"][parent]["world_frame"]["matrix"],
                                      dtype=float)
            local = np.linalg.inv(parent_world) @ world
        else:
            local = world
        body = ET.SubElement(parent_el, "body", {
            "name": name, "pos": _fmt(local[:3, 3]), "quat": _fmt(_quat_wxyz(local))})
        inertia = np.asarray(link["inertia_kg_m2"], dtype=float)
        ET.SubElement(body, "inertial", {
            "pos": _fmt(link["com_m"]), "mass": f"{link['mass_kg']:.10g}",
            "fullinertia": _fmt([inertia[0,0], inertia[1,1], inertia[2,2],
                                  inertia[0,1], inertia[0,2], inertia[1,2]])})
        mesh_name = f"mesh_{name}"
        ET.SubElement(self.asset, "mesh", {
            "name": mesh_name, "file": link["mesh_path"], "scale": ".001 .001 .001"})
        geom = ET.SubElement(body, "geom", {
            "name": f"geom_{name}", "type": "sdf", "mesh": mesh_name,
            "friction": _fmt(link["friction"]), "solref": ".002 1",
            "margin": ".0002", "condim": "4", "contype": "1",
            "conaffinity": "1"})
        self.bodies[name] = body
        self.geoms[name] = geom

    def joint(self, body: str, name: str, kind: str = "hinge", axis=(0,0,1),
              pos_mm=(0,0,0), frame: str = "local"):
        if name in self.joints:
            raise ValueError(f"duplicate joint '{name}'")
        if kind not in ("hinge", "slide"):
            raise ValueError("joint kind must be hinge or slide")
        if frame not in ("local", "world"):
            raise ValueError("joint frame must be local or world")
        target = self.bodies.get(body)
        if target is None:
            raise ValueError(f"joint '{name}' references unknown body '{body}'")
        joint_axis = np.asarray(axis, dtype=float)
        joint_pos_m = np.asarray(pos_mm, dtype=float) / 1000.0
        if frame == "world":
            body_world = np.asarray(self.facts["links"][body]["world_frame"]["matrix"],
                                    dtype=float)
            joint_pos_m = (np.linalg.inv(body_world)
                           @ np.array([*joint_pos_m, 1.0], dtype=float))[:3]
            joint_axis = body_world[:3, :3].T @ joint_axis
        joint = ET.SubElement(target, "joint", {"name": name, "type": kind,
            "axis": _fmt(joint_axis), "pos": _fmt(joint_pos_m)})
        self.joints[name] = joint

    def freejoint(self, body: str, name: str):
        if name in self.joints:
            raise ValueError(f"duplicate joint '{name}'")
        target = self.bodies.get(body)
        if target is None:
            raise ValueError(f"freejoint '{name}' references unknown body '{body}'")
        self.joints[name] = ET.SubElement(target, "freejoint", {"name": name})

    def weld(self, name: str, body1: str, body2: str, reason: str,
             sources: list[str], fact_ids: list[str]):
        self._constraint("weld", name, {"body1": body1, "body2": body2},
                         reason, sources, fact_ids)

    def connect(self, name: str, body1: str, body2: str, anchor_m,
                reason: str, sources: list[str], fact_ids: list[str]):
        self._constraint("connect", name, {"body1": body1, "body2": body2,
                         "anchor": _fmt(anchor_m)}, reason, sources, fact_ids)

    def joint_equality(self, name: str, joint1: str, joint2: str, ratio: float,
                       offset: float = 0.0, reason: str = "", sources=None, fact_ids=None):
        """Constrain ``joint2 = offset + ratio * joint1``.

        MuJoCo defines a joint equality as ``joint1 = polycoef(joint2)``. Swap
        the XML references so the emitter ABI follows the mechanical convention
        used by TransmissionSpec: first coordinate drives, second coordinate follows.
        """
        self._constraint("joint", name, {"joint1": joint2, "joint2": joint1,
                         "polycoef": _fmt([offset, ratio, 0, 0, 0])},
                         reason, sources or [], fact_ids or [])

    def _constraint(self, kind, name, attrs, reason, sources, fact_ids):
        if name in self.constraints:
            raise ValueError(f"duplicate constraint '{name}'")
        if not reason or not sources or not fact_ids:
            raise ValueError(f"constraint '{name}' needs reason, sources and fact_ids")
        node = ET.SubElement(self.equality, kind, {"name": name, **attrs})
        self.constraints[name] = node
        self.decisions.append({"generated_kind": "constraint", "generated_name": name,
            "constraint_type": kind, "reason": reason,
            "source_entity_ids": list(sources), "fact_ids": list(fact_ids)})

    def exclude(self, body1: str, body2: str, reason: str,
                sources: list[str], fact_ids: list[str]):
        if not reason or not sources or not fact_ids:
            raise ValueError("exclude needs reason, sources and fact_ids")
        pair = sorted((body1, body2))
        if any(x["pair"] == pair for x in self.excludes):
            raise ValueError(f"duplicate exclude {pair}")
        ET.SubElement(self.contact, "exclude", {"body1": body1, "body2": body2})
        self.excludes.append({"pair": pair, "reason": reason,
                              "source_entity_ids": list(sources),
                              "fact_ids": list(fact_ids)})

    def exclude_ground(self, body: str, reason: str,
                       sources: list[str], fact_ids: list[str]):
        """Disable only body-ground contact while preserving body-body collision."""
        geom = self.geoms.get(body)
        if geom is None:
            raise ValueError(f"ground exclude references unknown body '{body}'")
        if not reason or not sources or not fact_ids:
            raise ValueError("ground exclude needs reason, sources and fact_ids")
        if any(row["body"] == body for row in self.ground_excludes):
            raise ValueError(f"duplicate ground exclude for '{body}'")
        geom.set("conaffinity", "2")
        self.ground_excludes.append({"body": body, "reason": reason,
                                     "source_entity_ids": list(sources),
                                     "fact_ids": list(fact_ids)})

    def decision(self, entity_id: str, action: str, generated_nodes: list[str],
                 reason: str, fact_ids: list[str]):
        if action not in ("emitted", "represented_by"):
            raise ValueError("decision action must be emitted or represented_by")
        self.decisions.append({"entity_id": entity_id, "action": action,
            "generated_nodes": list(generated_nodes), "reason": reason,
            "fact_ids": list(fact_ids)})

    def support_patch(self, action: str, name: str, reason: str):
        if action not in ("remove_constraint", "free_body"):
            raise ValueError("unknown support patch action")
        self.support_patches.append({"action": action, "name": name, "reason": reason})

    def finish(self) -> tuple[str, dict]:
        if self._topology_plan is None:
            raise ValueError("compiler did not declare topology_plan")
        if not len(self.equality): self.root.remove(self.equality)
        if not len(self.contact): self.root.remove(self.contact)
        ET.indent(self.root)
        manifest = {"manifest_version": 3, "engine": "mujoco-agent",
                    "topology_plan": self._topology_plan,
                    "decisions": self.decisions, "excludes": self.excludes,
                    "ground_excludes": self.ground_excludes,
                    "support_patches": self.support_patches,
                    "bodies": sorted(self.bodies), "joints": sorted(self.joints),
                    "constraints": sorted(self.constraints)}
        return ET.tostring(self.root, encoding="unicode"), manifest
