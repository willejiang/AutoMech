# Multi-bore housing pattern mismatch

## Symptoms
An authoritative assembly solve reports gear center-distance failures or visible gear overlap while the required ratio, tooth counts, modules, and gear geometry are locked design intent.

## Evidence to inspect
Read `assembly_constraint_report.json`, the housing `sub_frames.json`, the housing link's `cq/*.py` or persisted PortSpecs, built gear `size_mm`, local gear poses, and the relevant `run.log` section. Compare the physical bore centers with realized seat-frame centers and required pitch-center distances.

## Characteristic finding
One physical housing link owns multiple bores. Abstract seat frames and real CAD bore centers differ, or the bore pattern spacing disagrees with the locked pitch distances. Moving only an abstract frame cannot move a hole already cut into a monolithic STL.

## Do not
- Do not change locked tooth counts, modules, or ratio merely to fit an accidental housing pattern.
- Do not move only `sub_frames` while leaving the physical bores unchanged.
- Do not regenerate a generic solid envelope box.
- Do not trigger a full Boss iteration before safe localized candidates are exhausted.

## Correct repair
Use a `housing_multibore_pattern` candidate. Pin one datum bore. Python computes every other center from authoritative shaft/gear constraints. Preserve each bore axis, diameter, and depth and preserve all gear semantics. Atomically rewrite the physical CadQuery bore pattern and matching realized frames from the same center list, rebuild only that housing link, verify real holes in the STL, then rerun libslvs and assembly.

## Escalation
Block this candidate when no one-to-one physical bore mapping can be proven, the pattern is not machine-editable, wall/ligament margins fail, or the housing topology is unsupported. Escalate rather than guessing.
