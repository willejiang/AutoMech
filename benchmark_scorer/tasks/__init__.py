"""Frozen deterministic benchmark task registries."""

from .comfort_v1 import TASK_REGISTRY, TASKS, ComfortTask, get_task

__all__ = ["ComfortTask", "TASKS", "TASK_REGISTRY", "get_task"]
