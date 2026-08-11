"""Derive the gravity-support probe only from an accepted agent MJCF manifest."""
from __future__ import annotations

import copy
import json
import xml.etree.ElementTree as ET
from pathlib import Path


class SupportDerivativeError(RuntimeError):
    pass


def _quat(frame: dict) -> str:
    return " ".join(f"{float(x):.10g}" for x in frame["quat_wxyz"])


def _pos(frame: dict) -> str:
    return " ".join(f"{float(x):.10g}" for x in frame["xyz_m"])


def _parent_map(root):
    return {child: parent for parent in root.iter() for child in parent}


def derive_support_mjcf(accepted_mjcf: str | Path, accepted_manifest: str | Path,
                        facts_path: str | Path, output_path: str | Path) -> tuple[str, str]:
    """Apply only support patches explicitly selected by the topology compiler agent."""
    root = ET.parse(accepted_mjcf).getroot()
    manifest = json.loads(Path(accepted_manifest).read_text(encoding="utf-8"))
    facts = json.loads(Path(facts_path).read_text(encoding="utf-8"))
    world = root.find("worldbody")
    if world is None:
        raise SupportDerivativeError("accepted MJCF has no worldbody")
    plan = manifest.get("topology_plan") or {}
    ground = str(plan.get("support_ground") or facts["model"].get("root_link") or "")
    if ground not in facts["links"]:
        raise SupportDerivativeError("topology_plan must name a valid support_ground")
    patches = manifest.get("support_patches") or []
    seen = set()
    for patch in patches:
        key = (patch.get("action"), patch.get("name"))
        if key in seen:
            raise SupportDerivativeError(f"duplicate support patch {key}")
        seen.add(key)
        if not patch.get("reason"):
            raise SupportDerivativeError(f"support patch {key} has no reason")
        action, name = key
        if action == "remove_constraint":
            matches = [node for node in root.findall("./equality/*") if node.get("name") == name]
            if len(matches) != 1:
                raise SupportDerivativeError(
                    f"remove_constraint '{name}' resolved {len(matches)} times")
            root.find("equality").remove(matches[0])
        elif action == "free_body":
            matches = [node for node in world.iter("body") if node.get("name") == name]
            if len(matches) != 1:
                raise SupportDerivativeError(f"free_body '{name}' resolved {len(matches)} times")
            body = matches[0]
            if name == ground:
                raise SupportDerivativeError("support_ground cannot be freed")
            parent = _parent_map(root).get(body)
            if parent is None:
                raise SupportDerivativeError(f"body '{name}' has no XML parent")
            parent.remove(body)
            frame = facts["links"][name]["world_frame"]
            body.set("pos", _pos(frame)); body.set("quat", _quat(frame))
            for node in list(body):
                if node.tag in ("joint", "freejoint"):
                    body.remove(node)
            body.insert(0, ET.Element("freejoint", {"name": f"support_free_{name}"}))
            world.append(body)
        else:
            raise SupportDerivativeError(f"unsupported support patch action '{action}'")
    equality = root.find("equality")
    if equality is not None and not len(equality):
        root.remove(equality)
    ET.indent(root)
    output = Path(output_path)
    output.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")
    try:
        import mujoco
        import numpy as np
        model = mujoco.MjModel.from_xml_path(str(output))
        data = mujoco.MjData(model); mujoco.mj_forward(model, data)
        if not all(np.isfinite(x).all() for x in (data.qpos, data.qvel, data.qacc, data.xpos)):
            raise SupportDerivativeError("support derivative initial state is non-finite")
    except SupportDerivativeError:
        raise
    except Exception as exc:
        raise SupportDerivativeError(
            f"support derivative does not load: {type(exc).__name__}: {exc}") from exc
    return str(output), ground
