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
