#!/usr/bin/env python3
from pathlib import Path
import subprocess
import sys

here = Path(__file__).resolve().parent
args = sys.argv[1:]
if not args:
    raise SystemExit("usage: apply_runtime_mass_parity.py <AuthoredVehicleRuntime.cpp>")

subprocess.check_call([sys.executable, str(here / "apply_runtime_mass_parity_base.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_wheel_parity.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_wheel_format_parity.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_node_friction_parity.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_shock_parity.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_force_order_parity.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_ground_contact_node_parity.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_simple2_ground_model_parity.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_wheel_brake_parity.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_default_diff_parity.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_meshwheel_build_bridge.py"), *args])
subprocess.check_call([sys.executable, str(here / "apply_upstream_rigdef_build_hook.py"), *args])
