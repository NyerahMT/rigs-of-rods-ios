#!/usr/bin/env python3
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_simple2_ground_model_parity.py <AuthoredVehicleRuntime.cpp>")

p = Path(sys.argv[1])
s = p.read_text()

old = '''        road = GroundContactParams();
        road.friction.adhesion_velocity = 0.55f;
        road.friction.static_friction = 1.12f;
        road.friction.sliding_friction = 0.76f;
        road.friction.hydrodynamic_friction = 0.018f;
        road.friction.stribeck_velocity = 0.35f;
        road.friction.stribeck_alpha = 2.0f;
'''
new = '''        road = GroundContactParams();
        // This control build runs the stock Simple2 terrain. Match its authored
        // [asphalt] ground model exactly (RigsOfRods/content,
        // simple2-terrain/simple2_groundmodel.cfg) instead of the temporary
        // hand-tuned contact curve that made the tyres feel disconnected.
        // simple2_landuse.cfg maps the terrain to asphalt and also uses asphalt
        // as defaultuse, so one ground model is correct for the current map.
        road.ground_strength = 1.0f;
        road.friction.adhesion_velocity = 3.0f;
        road.friction.static_friction = 1.2f;
        road.friction.sliding_friction = 0.75f;
        road.friction.hydrodynamic_friction = 0.01f;
        road.friction.stribeck_velocity = 6.0f;
        road.friction.stribeck_alpha = 2.0f;
'''

if new in s:
    print('Simple2 asphalt ground model parity already applied')
    raise SystemExit(0)
if s.count(old) != 1:
    raise SystemExit(f"Simple2 ground-model anchor expected once, found {s.count(old)}")

p.write_text(s.replace(old, new, 1))
print('matched stock Simple2 asphalt adhesion/Stribeck ground model')
