"""Deterministic, harness-independent PhysCAD benchmark primitives."""

from . import metrics
from .contract import (
    CONTRACT_ID, ContractError, Evidence, FileRecord, IngestedSubmission,
    PrimitiveResult, ResourceLimits, SubmissionManifest, TriState,
)
from .ingest import ingest_submission
from .replay import ReplayProfile, ReplayResult, replay_model, task_profile

__all__ = [
    "CONTRACT_ID", "ContractError", "Evidence", "FileRecord", "IngestedSubmission",
    "PrimitiveResult", "ReplayProfile", "ReplayResult", "ResourceLimits",
    "SubmissionManifest", "TriState", "ingest_submission", "metrics", "replay_model",
    "task_profile",
]
