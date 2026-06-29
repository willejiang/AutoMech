#!/usr/bin/env python3
"""Shared evaluator helpers: which simulator backend, and which VLM model.

- model_id(): the worker frontend stores the chosen model in Supabase; it also
  mirrors it to ~/.physcad_vlm.json ({"vlm_model": "anthropic/claude-opus-4.8"})
  so the evaluator uses the SAME model. Falls back to AZURE_VLM_DEPLOYMENT.
- select_backend(): normalize the strategy_selector's sim_backend choice.
"""
import json
import os
from pathlib import Path

BACKENDS = ("isaac_sim", "pybullet", "openfoam")
_VLM_FILE = Path.home() / ".physcad_vlm.json"


def model_id(default="anthropic/claude-opus-4.8"):
    """VLM id: worker's shared choice file > AZURE_VLM_DEPLOYMENT env > default."""
    try:
        m = json.loads(_VLM_FILE.read_text()).get("vlm_model")
        if m:
            return m
    except Exception:
        pass
    return os.environ.get("AZURE_VLM_DEPLOYMENT", default)


def select_backend(decision, fallback="pybullet"):
    """Read sim_backend from a strategy decision; default to pybullet (no GPU needed)."""
    b = (decision or {}).get("sim_backend", fallback)
    return b if b in BACKENDS else fallback
