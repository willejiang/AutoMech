"""Canonical dimension vocabulary (C2): fix a size_mm key set per shape_hint and fold the
managers' free-form aliases (gear_dia / wheel_dia / pitch_diameter / ...) onto it, so the
downstream dimension-lock + degeneracy gates stop drowning in the dim-naming chaos.

Design note (ALIAS-TOLERANT on purpose): the manager PROMPT that MANDATES these keys lives
in prompts/schema.py, which Track 2 owns. Until that mandate lands, the manager still emits
aliases — so the gates must ACCEPT today's output. Hence ``canonical_size`` normalizes a raw
size_mm to canonical keys but LEAVES unknown keys in place, and the gates read the normalized
dict. This way strictness improves (a gear_dia is now recognized as pitch_dia) without a hard
break. TODO(Track 2): add the canonical-key MANDATE to prompts/schema.py; then the gates can
optionally reject non-canonical keys instead of folding them.
"""

from __future__ import annotations

# The canonical key set the manager SHOULD emit per shape_hint (what Track 2's prompt will
# mandate). Used for docs + an optional strict check; the gates fold aliases onto these.
CANONICAL_KEYS: dict[str, tuple] = {
    "cylinder": ("radius", "height", "bore_dia"),
    "sphere": ("radius",),
    "box": ("x", "y", "z"),
    "cube": ("x", "y", "z"),
    "gear": ("module", "teeth", "thickness", "bore_dia"),
}

# Free-form alias -> canonical key. Managers name the SAME dimension a dozen ways; fold them
# so a downstream gate sees one key. Only unambiguous synonyms are mapped (a genuinely
# different dimension is left untouched). Diameters are NOT auto-halved here — the key name
# is preserved (pitch_dia stays a diameter); the gates already handle dia/2 matching.
ALIASES: dict[str, str] = {
    # radial
    "outer_diameter": "outer_dia",
    "od": "outer_dia",
    "inner_diameter": "inner_dia",
    "id": "inner_dia",
    "bore": "bore_dia",
    "bore_diameter": "bore_dia",
    "bore_d": "bore_dia",
    "shaft_dia": "bore_dia",
    "hole_dia": "bore_dia",
    "pitch_diameter": "pitch_dia",
    "gear_dia": "pitch_dia",
    "gear_diameter": "pitch_dia",
    "wheel_dia": "pitch_dia",
    "wheel_diameter": "pitch_dia",
    "pinion_dia": "pitch_dia",
    "diameter": "outer_dia",
    "dia": "outer_dia",
    "r": "radius",
    "rad": "radius",
    # axial / extent
    "h": "height",
    "ht": "height",
    "len": "length",
    "l": "length",
    "w": "width",
    "d": "depth",
    "thk": "thickness",
    "thick": "thickness",
    "wheel_thk": "thickness",
    "gear_thickness": "thickness",
    "face_width": "thickness",
    # gear
    "num_teeth": "teeth",
    "tooth_count": "teeth",
    "n_teeth": "teeth",
    "z": "z",                 # box extent — keep (do NOT fold to teeth count)
    "mod": "module",
    "m_module": "module",
}


def canonical_key(key: str) -> str:
    """Fold one raw size_mm key onto its canonical name (lower-cased). Unknown keys pass
    through unchanged so nothing is lost."""
    k = str(key).strip().lower()
    return ALIASES.get(k, k)


def canonical_size(size: dict, hint: str = "") -> dict:
    """Normalize a raw size_mm dict to canonical keys, folding aliases. On a key COLLISION
    (both an alias and its canonical target present, or two aliases of the same target),
    the first-seen positive value wins and the rest are dropped — a manager rarely means
    two different values for one dimension, and keeping both would defeat the point. The
    ``hint`` is accepted for future per-shape rules; unused today."""
    out: dict = {}
    if not isinstance(size, dict):
        return out
    for raw_k, v in size.items():
        ck = canonical_key(raw_k)
        if ck in out:
            # keep the existing value unless it is falsy and the new one is real
            if not _is_positive(out[ck]) and _is_positive(v):
                out[ck] = v
            continue
        out[ck] = v
    return out


def _is_positive(v) -> bool:
    try:
        return float(v) > 0.0
    except (TypeError, ValueError):
        return False
