# Minimal external-aero case for simpleFoam (incompressible, steady RAS).
# Theoretical template wired by run_scenario_openfoam.py. Drop body.stl into
# constant/triSurface, then surfaceFeatureExtract -> snappyHexMesh -> simpleFoam.
# Inlet velocity is set from the spec (inlet_velocity_ms). Forces are reported by
# the forces function object (drag/lift). Not yet run — GPU/CFD box is offline.
