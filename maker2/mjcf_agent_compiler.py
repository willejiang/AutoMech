"""Bounded LLM loop that compiles one immutable KinematicModel into accepted MJCF."""
from __future__ import annotations

import json
import shutil
from pathlib import Path

from .agent_tool_runtime import run_agent_tool_loop, tool_definition
from .llm.conversation import Conversation
from .mjcf_facts import (aabb_surface_distance_mm, extract_mjcf_facts, facts_hash,
                         query_pair_geometry, query_port_fit)
from .mjcf_validation import POLICY_VERSION, execute_compiler, validate_candidate
from .prompts.mjcf_compiler_prompt import (MJCF_COMPILER_PROMPT_VERSION,
                                            MJCF_COMPILER_SYSTEM,
                                            build_mjcf_compiler_user)


class MJCFCompilerError(RuntimeError):
    def __init__(self, message: str, *, report=None):
        super().__init__(message)
        self.report = report or {}


def _tools() -> list[dict]:
    return [
        tool_definition("read_mjcf_facts", "Read a bounded immutable facts page. Available sections: index, model, links, ports, poses, relations, motion_joints, transmissions, planetary_stages, entity_ids, simulation. offset/limit paginate entries; response includes next_offset.",
            {"section": {"type": "string"}, "offset": {"type": "integer"},
             "limit": {"type": "integer"}}, ["section"]),
        tool_definition("query_pair_geometry", "Measure relative geometry for a proposed pair.",
            {"a": {"type": "string"}, "b": {"type": "string"}}, ["a", "b"]),
        tool_definition("query_port_fit", "Measure axes and signed clearance for two named ports.",
            {"a": {"type": "string"}, "port_a": {"type": "string"},
             "b": {"type": "string"}, "port_b": {"type": "string"}},
            ["a", "port_a", "b", "port_b"]),
        tool_definition("query_nearby_parts", "List candidate bodies by placed world-AABB surface gap. radius_mm may expand candidate discovery but cannot shrink it below half the target part's bounding-box diagonal; response reports effective_radius_mm. This prevents long parts from being missed by tiny or origin-based radii.",
            {"part": {"type": "string"}, "radius_mm": {"type": "number"}},
            ["part", "radius_mm"]),
        tool_definition("query_motion_path", "Return authored-graph paths without choosing a tree.",
            {"source": {"type": "string"}, "target": {"type": "string"}},
            ["source", "target"]),
        tool_definition("read_current_compiler", "Read the current compiler source and revision.", {}, []),
        tool_definition("submit_compiler_source", "Atomically replace compiler source.",
            {"source": {"type": "string"}, "expected_revision": {"type": "integer"}},
            ["source", "expected_revision"]),
        tool_definition("run_mjcf_gate", "Execute and deterministically validate current compiler.", {}, []),
        tool_definition("read_mjcf_gate_report", "Read the latest complete gate report.", {}, []),
    ]


def _summary(facts: dict) -> str:
    model = facts["model"]
    return json.dumps({"name": model.get("name"), "links": list(facts["links"]),
        "relations": [x["name"] for x in model.get("relations", [])],
        "motion_joints": [x["name"] for x in model.get("motion_joints", [])],
        "transmissions": [x["name"] for x in model.get("transmissions", [])],
        "planetary_stages": [x["name"] for x in model.get("planetary_stages", [])],
        "driver": next((x for x in model["links"] if x.get("driver")), None),
        "output_link": model.get("output_link"), "watch_links": model.get("watch_links", [])},
        indent=2)


def _motion_path(facts: dict, source: str, target: str) -> dict:
    graph = {}
    model = facts["model"]
    for joint in model.get("motion_joints", []):
        graph.setdefault(joint.get("parent") or "__world__", set()).add(joint["child"])
        graph.setdefault(joint["child"], set()).add(joint.get("parent") or "__world__")
    for relation in model.get("relations", []):
        a, b = relation["base_part"], relation["incoming_part"]
        graph.setdefault(a, set()).add(b); graph.setdefault(b, set()).add(a)
    queue = [(source, [source])]; seen = {source}
    while queue:
        node, path = queue.pop(0)
        if node == target: return {"ok": True, "path": path}
        for nxt in sorted(graph.get(node, ())):
            if nxt not in seen: seen.add(nxt); queue.append((nxt, path+[nxt]))
    return {"ok": False, "path": []}


def compile_agent_mjcf(model, ctx, *, settings=None, metrics=None, log_fn=print,
                       client=None, max_rounds: int = 28, max_candidates: int = 7) -> str:
    root = Path(ctx.run_dir)
    facts = extract_mjcf_facts(model, ctx, settings)
    client = client or settings.make_client(
        getattr(settings, "mjcf_compiler_max_tokens", 32000), thinking="extended")
    key = facts_hash(facts, prompt_version=MJCF_COMPILER_PROMPT_VERSION,
                     policy_version=POLICY_VERSION, model_id=client.model)
    cache_root = Path(getattr(settings, "mjcf_compiler_cache_dir", "") or
                      (Path.home()/".cache"/"physcad"/"mjcf_agent"))
    cache = cache_root/key
    candidate = root/"model.agent.candidate.mjcf"
    accepted = Path(ctx.urdf_path).with_suffix(".mjcf")
    manifest_path = root/"builder_manifest.json"
    report_path = root/"mjcf_gate_report.json"
    source_path = root/"mjcf_compiler.py"
    trace_path = root/"mjcf_agent_trace.json"
    if (cache/"mjcf_compiler.py").exists():
        source = (cache/"mjcf_compiler.py").read_text(encoding="utf-8")
        try:
            xml, manifest = execute_compiler(source, facts)
            report = validate_candidate(xml, manifest, facts, candidate)
            if report.get("ok"):
                candidate.replace(accepted)
                source_path.write_text(source, encoding="utf-8")
                manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
                report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
                if metrics is not None: metrics["builder_manifest"] = str(manifest_path)
                log_fn(f"[mjcf-agent] cache hit {key[:12]}: accepted")
                return str(accepted)
        except Exception:
            pass

    state = {"revision": 0, "source": "", "report": {"ok": False,
             "errors": ["no compiler submitted"]}, "candidates": 0}

    def read_facts(section, offset=0, limit=25):
        model_facts = facts["model"]
        link_rows = {}
        for name, row in facts["links"].items():
            link_rows[name] = {key: row.get(key) for key in (
                "entity_id", "dof", "spin_axis", "slide_axis", "driver",
                "mount", "extra_mounts", "mass_kg",
            )}
            link_rows[name]["world_xyz_m"] = row["world_frame"]["xyz_m"]
            link_rows[name]["world_quat_wxyz"] = row["world_frame"]["quat_wxyz"]
        port_rows = {}
        for link_name, ports in facts["ports"].items():
            port_rows[link_name] = {}
            for port_name, row in ports.items():
                port_rows[link_name][port_name] = {key: row.get(key) for key in (
                    "entity_id", "type", "axis", "diameter_mm", "depth_mm",
                    "pitch_radius_mm", "normal_sign",
                )}
                port_rows[link_name][port_name]["world_xyz_m"] = row["world_frame"]["xyz_m"]
        sections = {
            "model": {key: model_facts.get(key) for key in
                      ("name", "root_link", "mesh_pairs", "output_link", "watch_links")},
            "links": link_rows,
            "ports": port_rows,
            "poses": model_facts.get("poses", []),
            "relations": model_facts.get("relations", []),
            "motion_joints": model_facts.get("motion_joints", []),
            "transmissions": model_facts.get("transmissions", []),
            "planetary_stages": model_facts.get("planetary_stages", []),
            "entity_ids": facts["entity_ids"],
            "simulation": facts["simulation"],
        }
        if section == "index":
            return json.dumps({"sections": {
                name: len(value) if isinstance(value, (list, dict)) else 1
                for name, value in sections.items()
            }}, indent=2)
        if section not in sections:
            return json.dumps({"error": f"unknown section '{section}'",
                               "available_sections": ["index", *sections]})
        value = sections[section]
        offset = max(0, int(offset or 0))
        limit = max(1, min(int(limit or 25), 25))
        if isinstance(value, dict):
            all_items = list(value.items())
            selected = all_items[offset:offset+limit]
            page = dict(selected)
        elif isinstance(value, list):
            all_items = value
            selected = all_items[offset:offset+limit]
            page = selected
        else:
            all_items = [value]
            selected = all_items[offset:offset+1]
            page = selected[0] if selected else None
        # A single tool round can contain several parallel reads. Keep every result
        # bounded so Conversation's pair-preserving truncation cannot discard the
        # initiating user message and leave the next gateway request empty.
        while len(selected) > 1 and len(json.dumps(page, indent=2)) > 16000:
            selected = selected[:-1]
            page = dict(selected) if isinstance(value, dict) else selected
        consumed = len(selected)
        next_offset = offset + consumed if offset + consumed < len(all_items) else None
        return json.dumps({"section": section, "offset": offset,
                           "returned": consumed, "total": len(all_items),
                           "next_offset": next_offset, "items": page}, indent=2)

    def nearby(part, radius_mm):
        if part not in facts["links"]:
            return json.dumps({"ok": False, "reason": "unknown part"})
        target = facts["links"][part]
        intrinsic_radius = sum(float(x)**2 for x in target["extents_mm"])**0.5 / 2.0
        requested_radius = max(0.0, float(radius_mm))
        radius = max(requested_radius, intrinsic_radius)
        out = []
        for name, row in facts["links"].items():
            if name == part:
                continue
            distance = aabb_surface_distance_mm(target, row)
            if distance <= radius:
                out.append({"part": name, "aabb_surface_distance_mm": distance,
                            "fact_id": f"nearby/{'/'.join(sorted((part, name)))}"})
        return json.dumps({"ok": True, "part": part,
                           "requested_radius_mm": requested_radius,
                           "effective_radius_mm": radius,
                           "items": sorted(out, key=lambda x: x["aabb_surface_distance_mm"])},
                          indent=2)

    def submit(source, expected_revision):
        if int(expected_revision) != state["revision"]:
            return f"revision mismatch: current={state['revision']}"
        # AST gate before mutating source.
        from .mjcf_validation import validate_compiler_source
        errors = validate_compiler_source(source)
        if errors: return json.dumps({"accepted": False, "errors": errors})
        state["source"] = source; state["revision"] += 1
        source_path.write_text(source, encoding="utf-8")
        return json.dumps({"accepted": True, "revision": state["revision"]})

    def run_gate():
        if not state["source"]: return json.dumps(state["report"])
        if state["candidates"] >= max_candidates:
            return json.dumps({"ok": False, "errors": ["candidate budget exhausted"]})
        state["candidates"] += 1
        try:
            xml, manifest = execute_compiler(state["source"], facts)
            report = validate_candidate(xml, manifest, facts, candidate)
            state["report"] = report
            if report.get("ok"):
                state["manifest"] = manifest
                state["xml"] = xml
        except Exception as exc:
            state["report"] = {"ok": False, "errors": [f"{type(exc).__name__}: {exc}"]}
        report_path.write_text(json.dumps(state["report"], indent=2), encoding="utf-8")
        return json.dumps(state["report"], indent=2)

    executors = {
        "read_mjcf_facts": read_facts,
        "query_pair_geometry": lambda a,b: json.dumps(query_pair_geometry(facts,a,b), indent=2),
        "query_port_fit": lambda a,port_a,b,port_b: json.dumps(
            query_port_fit(facts,a,port_a,b,port_b), indent=2),
        "query_nearby_parts": nearby,
        "query_motion_path": lambda source,target: json.dumps(
            _motion_path(facts,source,target), indent=2),
        "read_current_compiler": lambda: json.dumps(
            {"revision": state["revision"], "source": state["source"]}),
        "submit_compiler_source": submit,
        "run_mjcf_gate": run_gate,
        "read_mjcf_gate_report": lambda: json.dumps(state["report"], indent=2),
    }
    conversation = Conversation(); conversation.add_user_message(
        build_mjcf_compiler_user(_summary(facts)))
    result = run_agent_tool_loop(client, conversation, MJCF_COMPILER_SYSTEM, _tools(), executors,
        max_rounds=max_rounds, log_fn=log_fn, require_tool_call=True,
        history_max_chars=300000,
        text_only_nudge="Use the tools to inspect facts, submit a compiler, and run the gate now.")
    trace_path.write_text(json.dumps({"rounds": result.rounds,
        "executions": [{"name": x.name, "arguments": dict(x.arguments),
                        "is_error": x.is_error, "result": x.result[:4000]}
                       for x in result.executions], "report": state["report"]}, indent=2),
        encoding="utf-8")
    if not state["report"].get("ok"):
        raise MJCFCompilerError("MJCF agent did not produce an accepted compiler",
                                report=state["report"])
    candidate.replace(accepted)
    manifest = state["manifest"]
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    cache.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_path, cache/"mjcf_compiler.py")
    if metrics is not None: metrics["builder_manifest"] = str(manifest_path)
    log_fn(f"[mjcf-agent] accepted revision {state['revision']} after "
           f"{state['candidates']} candidate(s)")
    return str(accepted)
