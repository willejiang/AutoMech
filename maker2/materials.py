"""Named material classes -> physical properties for MuJoCo (maker2-mujoco-contact).

The manager tags each part with a material class (a word it can reason about — "steel",
"brass", "ruby", "plastic"); this maps it to the density + contact friction MuJoCo needs.
Replaces a single hardcoded mass/friction for every part, so a heavy brass plate and a
light ruby jewel behave differently under gravity and a steel gear grips differently than
a plastic one.

Density is kg/m^3 (part mass = density x mesh volume). Friction is MuJoCo's
"sliding torsional rolling" triple. Unknown/blank material falls back to steel.
"""

from __future__ import annotations

# density kg/m^3, friction "slide torsion roll"
_MATERIALS: dict[str, dict] = {
    "steel":   {"density": 7850.0, "friction": "1.0 0.05 0.005"},
    "brass":   {"density": 8500.0, "friction": "0.9 0.05 0.005"},
    "ruby":    {"density": 4000.0, "friction": "0.3 0.02 0.002"},   # jewel bearing: low
    "plastic": {"density": 1200.0, "friction": "0.6 0.04 0.004"},
    "aluminum": {"density": 2700.0, "friction": "0.8 0.05 0.005"},
    "titanium": {"density": 4500.0, "friction": "0.9 0.05 0.005"},
    "rubber":  {"density": 1100.0, "friction": "1.5 0.10 0.02"},    # high grip
    "wood":    {"density": 700.0,  "friction": "0.6 0.05 0.005"},
    "gold":    {"density": 19300.0, "friction": "0.8 0.05 0.005"},
}

DEFAULT_MATERIAL = "steel"


def material_props(name: str) -> dict:
    """(density, friction) for a material class; falls back to steel for unknown/blank."""
    key = (name or "").strip().lower()
    return _MATERIALS.get(key, _MATERIALS[DEFAULT_MATERIAL])


def density_of(name: str) -> float:
    return material_props(name)["density"]


def friction_of(name: str) -> str:
    return material_props(name)["friction"]


def known_materials() -> list[str]:
    return sorted(_MATERIALS)
