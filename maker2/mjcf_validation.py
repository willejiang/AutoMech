"""Acceptance-only gates for agent-authored MJCF compiler candidates."""
from __future__ import annotations

import ast
import json
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

POLICY_VERSION = 5
_ALLOWED_CALLS = {"range", "len", "sorted", "list", "dict", "set", "tuple", "float",
                  "int", "str", "bool", "enumerate", "zip", "min", "max", "abs", "sum"}
_ALLOWED_OUT_METHODS = {"topology_plan", "body", "joint", "freejoint", "weld", "connect",
                        "joint_equality", "exclude", "exclude_ground", "decision",
                        "support_patch"}
_ALLOWED_DATA_METHODS = {"get", "items", "keys", "values", "append", "extend",
                         "update", "setdefault", "startswith", "endswith"}
_FORBIDDEN = (ast.Import, ast.ImportFrom, ast.With, ast.AsyncWith, ast.Try, ast.Raise,
              ast.ClassDef, ast.Lambda, ast.Global, ast.Nonlocal)


class MJCFValidationError(RuntimeError):
    pass


def validate_compiler_source(source: str) -> list[str]:
    errors = []
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        return [f"syntax error: {exc}"]
    if len(source) > 60000:
        errors.append("compiler source exceeds 60000 characters")
    functions = [x for x in tree.body if isinstance(x, ast.FunctionDef)]
    if [x.name for x in functions] != ["compile_mjcf"]:
        errors.append("source must define exactly one top-level compile_mjcf(facts, out)")
    elif [x.arg for x in functions[0].args.args] != ["facts", "out"]:
        errors.append("compile_mjcf signature must be (facts, out)")
    for node in ast.walk(tree):
        if isinstance(node, _FORBIDDEN):
            errors.append(f"forbidden syntax: {type(node).__name__}")
        elif isinstance(node, ast.Attribute) and node.attr.startswith("__"):
            errors.append("dunder access is forbidden")
        elif isinstance(node, ast.Name) and node.id in {"open", "exec", "eval", "compile",
                                                        "globals", "locals", "__import__"}:
            errors.append(f"forbidden name: {node.id}")
        elif isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name) and node.func.id not in _ALLOWED_CALLS:
                # Calls to locally-defined helper functions are intentionally disallowed;
                # one compact compile function makes AST validation predictable.
                if node.func.id != "compile_mjcf":
                    errors.append(f"forbidden call: {node.func.id}")
            elif isinstance(node.func, ast.Attribute):
                is_out = (isinstance(node.func.value, ast.Name)
                          and node.func.value.id == "out"
                          and node.func.attr in _ALLOWED_OUT_METHODS)
                is_data = node.func.attr in _ALLOWED_DATA_METHODS
                if not (is_out or is_data):
                    errors.append(f"forbidden method call: {node.func.attr}")
    return sorted(set(errors))


def execute_compiler(source: str, facts: dict, *, timeout: int = 20) -> tuple[str, dict]:
    """Execute candidate source in a disposable Python process with a wall-clock bound."""
    errors = validate_compiler_source(source)
    if errors:
        raise MJCFValidationError("; ".join(errors))
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="maker2_mjcf_compiler_") as temp:
        temp_path = Path(temp)
        (temp_path/"source.py").write_text(source, encoding="utf-8")
        (temp_path/"facts.json").write_text(json.dumps(facts), encoding="utf-8")
        runner = '''
import json, sys
from pathlib import Path
root=Path(sys.argv[1]);sys.path.insert(0,str(root))
from maker2.mjcf_emitter import MJCFEmitter
source=Path(sys.argv[2]).read_text(encoding="utf-8")
facts=json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
safe={name:getattr(__builtins__,name) for name in %r}
ns={"__builtins__":safe};exec(compile(source,"<mjcf_compiler>","exec"),ns)
out=MJCFEmitter(facts);ns["compile_mjcf"](facts,out)
xml,manifest=out.finish();Path(sys.argv[4]).write_text(xml,encoding="utf-8")
Path(sys.argv[5]).write_text(json.dumps(manifest),encoding="utf-8")
''' % sorted(_ALLOWED_CALLS)
        (temp_path/"runner.py").write_text(runner, encoding="utf-8")
        env = {"PYTHONIOENCODING": "utf-8", "PYTHONDONTWRITEBYTECODE": "1"}
        try:
            result = subprocess.run([sys.executable, str(temp_path/"runner.py"), str(root),
                str(temp_path/"source.py"), str(temp_path/"facts.json"),
                str(temp_path/"model.mjcf"), str(temp_path/"manifest.json")],
                cwd=temp, env=env, capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            raise MJCFValidationError(f"compiler execution timed out after {timeout}s") from exc
        if result.returncode != 0:
            raise MJCFValidationError((result.stderr or result.stdout or
                                       "compiler subprocess failed")[-2000:])
        return ((temp_path/"model.mjcf").read_text(encoding="utf-8"),
                json.loads((temp_path/"manifest.json").read_text(encoding="utf-8")))


def _unique_named(root, tag: str) -> tuple[set[str], list[str]]:
    seen, dup = set(), []
    for node in root.iter(tag):
        name = node.get("name")
        if not name:
            continue
        if name in seen: dup.append(name)
        seen.add(name)
    return seen, dup


def _declared_exclude_exemption(facts: dict, pair: tuple[str, str]) -> str | None:
    """Return the sole authored and measured geometry exemption for this exact pair."""
    from .mjcf_facts import query_port_fit

    relations = {}
    for relation in facts["model"].get("relations") or []:
        relation_pair = tuple(sorted((relation.get("base_part", ""),
                                      relation.get("incoming_part", ""))))
        if relation_pair == pair:
            relations[f"relation/{relation.get('name', '')}"] = relation

    for relation_id, relation in relations.items():
        mate_type = relation.get("mate_type")
        if mate_type not in ("press_fit", "journal_bearing", "ball_bearing"):
            continue
        fit = query_port_fit(facts, relation.get("base_part", ""),
                             relation.get("base_port", ""),
                             relation.get("incoming_part", ""),
                             relation.get("incoming_port", ""))
        clearance = fit.get("diametral_clearance_mm") if fit.get("ok") else None
        if mate_type == "press_fit":
            return relation_id
        if mate_type in ("journal_bearing", "ball_bearing") and clearance is not None \
                and float(clearance) >= 0.0:
            return relation_id

    declared_mesh_pairs = {tuple(sorted(x[:2])) for x in
                           (facts["model"].get("mesh_pairs") or []) if len(x) >= 2}
    planetary_pairs = set()
    for stage in facts["model"].get("planetary_stages") or []:
        for planet in stage.get("planets") or []:
            gear = planet.get("gear", "")
            if gear:
                planetary_pairs.add(tuple(sorted((stage.get("sun", ""), gear))))
                planetary_pairs.add(tuple(sorted((stage.get("ring", ""), gear))))
    for relation_id, relation in relations.items():
        if relation.get("mate_type") in ("gear_spur_external", "gear_spur_internal",
                                         "gear_external", "gear_internal") and pair in (
                                             declared_mesh_pairs | planetary_pairs):
            return relation_id
    return None


def _validate_exclude_geometry(facts: dict, pair: tuple[str, str],
                               exclude: dict) -> str | None:
    """Reject an exclude when overlapping AABBs lack proof of solid separation."""
    from .mjcf_facts import query_pair_geometry

    exemption = _declared_exclude_exemption(facts, pair)
    if exemption is not None:
        if exemption not in (exclude.get("source_entity_ids") or []):
            return f"exclude {list(pair)} does not cite its exact declared exemption '{exemption}'"
        return None

    geometry = query_pair_geometry(facts, pair[0], pair[1])
    extents = geometry.get("aabb_overlap_extents_mm") or []
    aabb_positive = len(extents) == 3 and all(float(x) > 0.0 for x in extents)
    overlap = geometry.get("solid_overlap_mm3")
    if aabb_positive and (overlap is None or float(overlap) > 0.0):
        state = "unavailable" if overlap is None else f"positive ({float(overlap):.10g} mm^3)"
        return (f"exclude {list(pair)} is unsupported by geometry facts: AABB overlap is "
                f"positive on all axes and exact solid overlap is {state}; sampled positive "
                "surface distance does not prove separation")
    return None


def _dynamic_transmission_probe(path: Path, topology: dict, facts: dict) -> dict:
    """Compare identical finite-effort drives with contact on and off.

    This is evidence generation, not an auto-exclude rule. It identifies exact active
    pairs only when the declared transmission works without contact but contact stalls
    or distorts it, so the compiler agent can revise its pair decisions.
    """
    transmissions = topology.get("transmissions") or []
    coordinate_map = topology.get("coordinate_map") or {}
    driver_link = next((row.get("name") for row in facts["model"].get("links", [])
                        if row.get("driver")), None)
    driver_joint = coordinate_map.get(driver_link) if driver_link else None
    has_closure = bool(topology.get("closure_edges"))
    if not driver_joint or (not transmissions and not has_closure):
        return {"ran": False, "reason": "no mapped driver or dynamic transmission/closure"}

    import mujoco
    import numpy as np

    def run(disable_contact: bool) -> dict:
        model = mujoco.MjModel.from_xml_path(str(path))
        data = mujoco.MjData(model)
        if disable_contact:
            model.geom_contype[:] = 0
            model.geom_conaffinity[:] = 0
        try:
            driver = model.joint(driver_joint)
        except Exception:
            return {"ok": False, "error": f"unknown driver joint '{driver_joint}'"}
        driver_qadr = int(model.jnt_qposadr[driver.id])
        driver_dof = int(model.jnt_dofadr[driver.id])
        joints = {driver_joint}
        normalized = []
        for tx in transmissions:
            driving = tx.get("driving_joint") or tx.get("joint1")
            driven = tx.get("driven_joint") or tx.get("joint2")
            ratio = tx.get("ratio")
            if not driving or not driven or ratio is None:
                continue
            joints.update((driving, driven))
            normalized.append((tx.get("name") or f"{driving}->{driven}",
                               driving, driven, float(ratio)))
        qadr = {}
        for name in joints:
            try:
                joint = model.joint(name)
                qadr[name] = int(model.jnt_qposadr[joint.id])
            except Exception:
                return {"ok": False, "error": f"transmission references unknown joint '{name}'"}
        mujoco.mj_forward(model, data)
        settle_steps = min(500, max(20, int(0.05 / model.opt.timestep)))
        for _ in range(settle_steps):
            mujoco.mj_step(model, data)
        baseline = {name: float(data.qpos[address]) for name, address in qadr.items()}
        stats = {}
        steps = min(2500, max(200, int(0.35 / model.opt.timestep)))
        rate, kp, kd, force = 3.0, 25.0, 1.0, 0.5
        start = baseline[driver_joint]
        for step in range(steps):
            target = start + rate * (step + 1) * model.opt.timestep
            effort = kp * (target - float(data.qpos[driver_qadr]))
            effort -= kd * (float(data.qvel[driver_dof]) - rate)
            data.qfrc_applied[driver_dof] = max(-force, min(force, effort))
            mujoco.mj_step(model, data)
            if not disable_contact:
                for contact_index in range(data.ncon):
                    contact = data.contact[contact_index]
                    body1 = int(model.geom_bodyid[contact.geom1])
                    body2 = int(model.geom_bodyid[contact.geom2])
                    pair = tuple(sorted((model.body(body1).name or "world",
                                         model.body(body2).name or "world")))
                    row = stats.setdefault(pair, {"count": 0, "max_penetration_mm": 0.0,
                                                  "max_force_n": 0.0,
                                                  "impulse_ns": 0.0})
                    wrench = np.zeros(6)
                    mujoco.mj_contactForce(model, data, contact_index, wrench)
                    contact_force = float(np.linalg.norm(wrench[:3]))
                    row["count"] += 1
                    row["max_penetration_mm"] = max(
                        row["max_penetration_mm"], max(0.0, -float(contact.dist)) * 1000.0)
                    row["max_force_n"] = max(row["max_force_n"], contact_force)
                    row["impulse_ns"] += contact_force * model.opt.timestep
        deltas = {name: float(data.qpos[address]) - baseline[name]
                  for name, address in qadr.items()}
        residuals = []
        for name, driving, driven, ratio in normalized:
            expected = ratio * deltas[driving]
            error = deltas[driven] - expected
            scale = max(abs(expected), 0.01)
            residuals.append({"name": name, "driving_joint": driving,
                              "driven_joint": driven, "ratio": ratio,
                              "driving_delta": deltas[driving],
                              "driven_delta": deltas[driven],
                              "normalized_residual": abs(error) / scale})
        active = [{"pair": list(pair), **row} for pair, row in
                  sorted(stats.items(), key=lambda item: item[1]["impulse_ns"], reverse=True)]
        commanded_travel = rate * steps * model.opt.timestep
        return {"ok": True, "driver_travel": abs(deltas[driver_joint]),
                "commanded_travel": commanded_travel,
                "joint_deltas": deltas, "transmission_residuals": residuals,
                "active_contacts": active[:20]}

    contact_on = run(False)
    contact_off = run(True)
    errors = []
    if not contact_on.get("ok") or not contact_off.get("ok"):
        errors.append(contact_on.get("error") or contact_off.get("error") or
                      "dynamic transmission probe failed")
    else:
        off_travel = float(contact_off.get("driver_travel") or 0.0)
        on_travel = float(contact_on.get("driver_travel") or 0.0)
        commanded = float(contact_off.get("commanded_travel") or 0.0)
        if has_closure and commanded > 0.05 and off_travel < commanded * 0.5:
            errors.append("closed mechanism stalls even with contact disabled; check body tree, "
                          "closure anchor, and world/local joint frame conversion")
        if has_closure and off_travel > 0.05 and on_travel < off_travel * 0.8:
            errors.append("contact materially stalls the closed mechanism relative to the "
                          "no-contact control; inspect active body/world pairs")
        elif off_travel > 0.05 and on_travel < max(0.01, off_travel * 0.1):
            errors.append("contact stalls the mapped driver relative to the no-contact control")
        warnings = []
        off_by_name = {row["name"]: row for row in contact_off["transmission_residuals"]}
        for row in contact_on["transmission_residuals"]:
            control = off_by_name.get(row["name"], {})
            off_residual = float(control.get("normalized_residual") or 0.0)
            on_residual = float(row.get("normalized_residual") or 0.0)
            if off_residual <= 0.15 and on_residual > max(0.25, off_residual + 0.15):
                errors.append(f"contact distorts transmission '{row['name']}' "
                              f"(residual {on_residual:.3g} vs no-contact {off_residual:.3g})")
            elif off_residual > 0.15:
                # A short finite-effort control can still contain a soft-equality
                # transient. Longer scenario metrics or exact-qpos checks judge the
                # final ratio; this A/B gate only attributes CONTACT-specific damage.
                warnings.append(f"no-contact short-probe residual for '{row['name']}' is "
                                f"{off_residual:.3g}; not used as contact attribution")
    return {"ran": True, "ok": not errors, "errors": errors,
            "warnings": warnings if 'warnings' in locals() else [],
            "contact_on": contact_on, "contact_off": contact_off}


def validate_candidate(xml_text: str, manifest: dict, facts: dict,
                       output_path: str | Path, *, run_smoke: bool = True) -> dict:
    errors = []
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError as exc:
        return {"ok": False, "errors": [f"XML parse error: {exc}"]}
    allowed = {"mujoco", "compiler", "option", "size", "asset", "mesh", "worldbody",
               "body", "joint", "freejoint", "inertial", "geom", "site", "light",
               "equality", "weld", "connect", "contact", "exclude"}
    for node in root.iter():
        if node.tag not in allowed:
            errors.append(f"forbidden MJCF tag <{node.tag}>")
    bodies, dup_bodies = _unique_named(root, "body")
    joints, dup_joints = _unique_named(root, "joint")
    meshes, dup_meshes = _unique_named(root, "mesh")
    if dup_bodies: errors.append(f"duplicate bodies: {dup_bodies}")
    if dup_joints: errors.append(f"duplicate joints: {dup_joints}")
    if dup_meshes: errors.append(f"duplicate meshes: {dup_meshes}")
    expected_bodies = set(facts["links"])
    if bodies != expected_bodies:
        errors.append(f"body coverage mismatch missing={sorted(expected_bodies-bodies)} "
                      f"extra={sorted(bodies-expected_bodies)}")
    for mesh in root.findall("./asset/mesh"):
        file = (mesh.get("file") or "").replace("\\", "/")
        allowed_files = {x["mesh_path"] for x in facts["links"].values()}
        if file not in allowed_files or file.startswith("/") or ".." in file.split("/"):
            errors.append(f"mesh path is not an allowed fact path: {file}")
    for eq in root.findall("./equality/*"):
        for attr, known in (("body1", bodies), ("body2", bodies),
                            ("joint1", joints), ("joint2", joints)):
            if eq.get(attr) and eq.get(attr) not in known:
                errors.append(f"constraint '{eq.get('name')}' references unknown {attr} "
                              f"'{eq.get(attr)}'")
    xml_excludes = {tuple(sorted((x.get("body1", ""), x.get("body2", ""))))
                    for x in root.findall("./contact/exclude")}
    manifest_excludes = {tuple(sorted(x.get("pair") or []))
                         for x in manifest.get("excludes") or []}
    if xml_excludes != manifest_excludes:
        errors.append("XML excludes do not exactly match manifest excludes")

    # The agent must apply the legacy builder's proven pair-by-pair contact
    # classification, not merely mention it in prose. Every contact-relevant authored
    # relation needs one keep/exclude decision, and every emitted exclude must be the
    # exact pair selected by that inventory.
    topology = manifest.get("topology_plan") or {}
    contact_decisions = topology.get("contact_decisions")
    contact_by_pair = {}
    if not isinstance(contact_decisions, list):
        errors.append("topology_plan.contact_decisions must be a pair-decision list")
    else:
        for index, decision in enumerate(contact_decisions):
            pair = tuple(sorted(decision.get("pair") or []))
            action = decision.get("action")
            if len(pair) != 2 or pair[0] == pair[1] or any(name not in bodies for name in pair):
                errors.append(f"contact decision {index} has invalid body pair {list(pair)}")
                continue
            if pair in contact_by_pair:
                errors.append(f"duplicate contact decision for pair {list(pair)}")
                continue
            contact_by_pair[pair] = decision
            if action not in ("keep", "exclude"):
                errors.append(f"contact decision {list(pair)} has invalid action '{action}'")
            if not decision.get("reason") or not decision.get("source_entity_ids") or not decision.get("fact_ids"):
                errors.append(f"contact decision {list(pair)} lacks provenance")
            for entity_id in decision.get("source_entity_ids") or []:
                if entity_id not in facts["entity_ids"]:
                    errors.append(f"contact decision {list(pair)} references unknown source '{entity_id}'")
            for fact_id in decision.get("fact_ids") or []:
                if fact_id not in set(facts["entity_ids"]) and not fact_id.startswith(
                        ("pair/", "fit/", "path/", "nearby/")):
                    errors.append(f"contact decision {list(pair)} references unknown fact '{fact_id}'")
        decided_excludes = {pair for pair, decision in contact_by_pair.items()
                            if decision.get("action") == "exclude"}
        if decided_excludes != manifest_excludes:
            errors.append("contact_decisions exclude set does not exactly match manifest excludes")

        contact_relation_types = {"press_fit", "journal_bearing", "ball_bearing",
                                  "gear_spur_external", "gear_spur_internal",
                                  "gear_external", "gear_internal", "pin", "revolute"}
        for relation in facts["model"].get("relations") or []:
            if relation.get("mate_type") not in contact_relation_types:
                continue
            pair = tuple(sorted((relation.get("base_part", ""),
                                 relation.get("incoming_part", ""))))
            relation_id = f"relation/{relation.get('name', '')}"
            decision = contact_by_pair.get(pair)
            if decision is None:
                errors.append(f"contact decision missing for relation '{relation_id}' pair {list(pair)}")
            elif relation_id not in (decision.get("source_entity_ids") or []):
                errors.append(f"contact decision {list(pair)} does not cite '{relation_id}'")
    entity_decisions = [x for x in manifest.get("decisions") or [] if x.get("entity_id")]
    counts = {}
    generated_nodes = (set(bodies) | set(joints)
                       | {x.get("name") for x in root.findall("./equality/*") if x.get("name")})
    known_fact_ids = set(facts["entity_ids"])
    for decision in entity_decisions:
        entity_id = decision["entity_id"]
        counts[entity_id] = counts.get(entity_id, 0) + 1
        if entity_id not in facts["entity_ids"]:
            errors.append(f"manifest has decision for unknown entity '{entity_id}'")
        if not decision.get("reason") or not decision.get("fact_ids"):
            errors.append(f"decision '{entity_id}' lacks reason/fact_ids")
        if not decision.get("generated_nodes"):
            errors.append(f"decision '{entity_id}' does not resolve to a generated node")
        for node in decision.get("generated_nodes") or []:
            raw = node.split("/", 1)[-1]
            if raw not in generated_nodes:
                errors.append(f"decision '{entity_id}' references unknown generated node '{node}'")
        for fact_id in decision.get("fact_ids") or []:
            if fact_id not in known_fact_ids and not fact_id.startswith(("pair/", "fit/", "path/", "nearby/")):
                errors.append(f"decision '{entity_id}' references unknown fact '{fact_id}'")
    missing = set(facts["entity_ids"]) - set(counts)
    duplicate = sorted(k for k, v in counts.items() if v != 1)
    if missing: errors.append(f"manifest decisions missing: {sorted(missing)}")
    if duplicate: errors.append(f"duplicate manifest decisions: {duplicate}")
    topology = manifest.get("topology_plan") or {}
    if not topology:
        errors.append("manifest lacks topology_plan")
    coordinate_map = topology.get("coordinate_map")
    if not isinstance(coordinate_map, dict):
        errors.append("topology_plan lacks coordinate_map object")
    else:
        links = facts["model"].get("links") or []
        required_links = {x["name"] for x in links if x.get("dof") in ("spin", "slide", "free")}
        required_links.update(x["name"] for x in links if x.get("driver"))
        output = facts["model"].get("output_link")
        if output: required_links.add(output)
        required_links.update(facts["model"].get("watch_links") or [])
        for link in sorted(required_links):
            if link not in coordinate_map:
                errors.append(f"coordinate_map missing required link '{link}'")
            elif coordinate_map[link] is not None and coordinate_map[link] not in joints:
                errors.append(f"coordinate_map '{link}' references unknown joint "
                              f"'{coordinate_map[link]}'")
    for exclude in manifest.get("excludes") or []:
        pair = exclude.get("pair") or []
        valid_pair = len(pair) == 2 and not any(name not in bodies for name in pair)
        if not valid_pair:
            errors.append(f"exclude {pair} references unknown body")
        if not exclude.get("reason") or not exclude.get("source_entity_ids") or not exclude.get("fact_ids"):
            errors.append(f"exclude {pair} lacks provenance")
        for entity_id in exclude.get("source_entity_ids") or []:
            if entity_id not in facts["entity_ids"]:
                errors.append(f"exclude {pair} references unknown source '{entity_id}'")
        if valid_pair:
            geometry_error = _validate_exclude_geometry(
                facts, tuple(sorted((str(pair[0]), str(pair[1])))), exclude)
            if geometry_error:
                errors.append(geometry_error)
    ground_excludes = manifest.get("ground_excludes") or []
    ground_exclude_bodies = set()
    for ground_exclude in ground_excludes:
        body = ground_exclude.get("body")
        if body not in bodies:
            errors.append(f"ground exclude references unknown body '{body}'")
            continue
        if body in ground_exclude_bodies:
            errors.append(f"duplicate ground exclude for '{body}'")
        ground_exclude_bodies.add(body)
        if not ground_exclude.get("reason") or not ground_exclude.get(
                "source_entity_ids") or not ground_exclude.get("fact_ids"):
            errors.append(f"ground exclude '{body}' lacks provenance")
        for entity_id in ground_exclude.get("source_entity_ids") or []:
            if entity_id not in facts["entity_ids"]:
                errors.append(f"ground exclude '{body}' references unknown source '{entity_id}'")
        for fact_id in ground_exclude.get("fact_ids") or []:
            if fact_id not in set(facts["entity_ids"]) and not fact_id.startswith(
                    ("pair/", "fit/", "path/", "nearby/", "dynamic/")):
                errors.append(f"ground exclude '{body}' references unknown fact '{fact_id}'")
    for body in bodies:
        geom = root.find(f".//geom[@name='geom_{body}']")
        if geom is None:
            continue
        ground_disabled = geom.get("conaffinity") == "2"
        if ground_disabled != (body in ground_exclude_bodies):
            errors.append(f"ground collision mask for '{body}' does not match manifest")

    patches = manifest.get("support_patches") or []
    patch_keys = set()
    constraint_names = {x.get("name") for x in root.findall("./equality/*") if x.get("name")}
    for patch in patches:
        key = (patch.get("action"), patch.get("name"))
        if key in patch_keys:
            errors.append(f"duplicate support patch {key}")
        patch_keys.add(key)
        if not patch.get("reason"):
            errors.append(f"support patch {key} lacks reason")
        if patch.get("action") == "remove_constraint" and patch.get("name") not in constraint_names:
            errors.append(f"support patch references unknown constraint '{patch.get('name')}'")
        elif patch.get("action") == "free_body" and patch.get("name") not in bodies:
            errors.append(f"support patch references unknown body '{patch.get('name')}'")
        elif patch.get("action") not in ("remove_constraint", "free_body"):
            errors.append(f"unknown support patch action '{patch.get('action')}'")
    report = {"ok": not errors, "errors": errors, "policy_version": POLICY_VERSION,
              "bodies": sorted(bodies), "joints": sorted(joints)}
    if errors:
        return report
    path = Path(output_path)
    path.write_text(xml_text, encoding="utf-8")
    try:
        import mujoco
        import numpy as np
        model = mujoco.MjModel.from_xml_path(str(path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        finite = all(np.isfinite(x).all() for x in (data.qpos, data.qvel, data.qacc, data.xpos))
        if not finite:
            errors.append("initial MuJoCo state is non-finite")
        if run_smoke and finite:
            for step in range(min(100, max(1, int(.02/model.opt.timestep)))):
                mujoco.mj_step(model, data)
                if not all(np.isfinite(x).all() for x in
                           (data.qpos, data.qvel, data.qacc, data.xpos)):
                    errors.append(f"non-finite state during bounded smoke at step {step}")
                    break
        report.update({"nq": int(model.nq), "nv": int(model.nv), "finite": not errors})
        if run_smoke and not errors:
            dynamic_probe = _dynamic_transmission_probe(path, topology, facts)
            report["dynamic_transmission_probe"] = dynamic_probe
            if dynamic_probe.get("ran") and not dynamic_probe.get("ok"):
                errors.extend(f"dynamic transmission probe: {message}"
                              for message in dynamic_probe.get("errors") or [])
    except Exception as exc:
        errors.append(f"MuJoCo load/smoke failed: {type(exc).__name__}: {exc}")
    report["ok"] = not errors
    report["errors"] = errors
    return report
