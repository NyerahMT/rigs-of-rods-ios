#!/usr/bin/env python3
"""Compare two RoR physics JSONL traces and report the first divergence.

The left trace is conventionally the upstream desktop oracle and the right trace
is the portable/iOS core. Tolerances are deliberately small but nonzero so ARM64
and x86_64 floating-point noise does not masquerade as a physics difference.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable


def load(path: Path) -> list[dict]:
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise SystemExit(f"{path}:{line_no}: invalid JSON: {exc}")
    if not rows:
        raise SystemExit(f"{path}: empty trace")
    return rows


def close(a: float, b: float, abs_tol: float, rel_tol: float) -> bool:
    return math.isclose(float(a), float(b), abs_tol=abs_tol, rel_tol=rel_tol)


def vector_error(a: Iterable[float], b: Iterable[float]) -> float:
    aa = list(a)
    bb = list(b)
    return max(abs(float(x) - float(y)) for x, y in zip(aa, bb))


def compare_sample(a: dict, b: dict, abs_tol: float, rel_tol: float) -> list[str]:
    errors: list[str] = []
    if a.get("step") != b.get("step"):
        return [f"step mismatch oracle={a.get('step')} candidate={b.get('step')}"]

    ta = a["telemetry"]
    tb = b["telemetry"]
    for key in ("heading", "speed", "forward_speed", "driven_wheel_speed", "engine_rpm", "steering", "throttle"):
        if not close(ta[key], tb[key], abs_tol, rel_tol):
            errors.append(f"telemetry.{key}: oracle={ta[key]:.9g} candidate={tb[key]:.9g} delta={abs(float(ta[key])-float(tb[key])):.9g}")
    if ta["gear"] != tb["gear"]:
        errors.append(f"telemetry.gear: oracle={ta['gear']} candidate={tb['gear']}")
    if vector_error(ta["center"], tb["center"]) > abs_tol:
        errors.append(f"telemetry.center: oracle={ta['center']} candidate={tb['center']} max_delta={vector_error(ta['center'], tb['center']):.9g}")

    na = a["nodes"]
    nb = b["nodes"]
    if len(na) != len(nb):
        errors.append(f"node count: oracle={len(na)} candidate={len(nb)}")
        return errors

    worst = None
    worst_delta = 0.0
    for xa, xb in zip(na, nb):
        if xa["i"] != xb["i"]:
            errors.append(f"node index mismatch oracle={xa['i']} candidate={xb['i']}")
            break
        for field in ("p", "v", "f"):
            delta = vector_error(xa[field], xb[field])
            scale = max(1.0, *(abs(float(v)) for v in xa[field]), *(abs(float(v)) for v in xb[field]))
            allowed = max(abs_tol, rel_tol * scale)
            if delta > allowed and delta > worst_delta:
                worst_delta = delta
                worst = (xa["i"], field, xa[field], xb[field], allowed)
        for field in ("m", "mu"):
            if not close(xa[field], xb[field], abs_tol, rel_tol):
                delta = abs(float(xa[field]) - float(xb[field]))
                if delta > worst_delta:
                    worst_delta = delta
                    worst = (xa["i"], field, xa[field], xb[field], max(abs_tol, rel_tol * max(1.0, abs(float(xa[field])), abs(float(xb[field])))))
    if worst:
        i, field, va, vb, allowed = worst
        errors.append(f"worst node divergence: node={i} field={field} oracle={va} candidate={vb} delta={worst_delta:.9g} allowed={allowed:.9g}")
    return errors


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("oracle", type=Path)
    p.add_argument("candidate", type=Path)
    p.add_argument("--abs", dest="abs_tol", type=float, default=1.0e-5)
    p.add_argument("--rel", dest="rel_tol", type=float, default=1.0e-5)
    args = p.parse_args()

    oracle = load(args.oracle)
    candidate = load(args.candidate)
    if len(oracle) != len(candidate):
        print(f"trace sample count mismatch: oracle={len(oracle)} candidate={len(candidate)}")
        return 1

    for index, (a, b) in enumerate(zip(oracle, candidate)):
        errors = compare_sample(a, b, args.abs_tol, args.rel_tol)
        if errors:
            print(f"first divergence at sample {index}, physics step {a.get('step')}")
            for error in errors[:12]:
                print(f"  {error}")
            return 1

    print(f"physics traces match: {len(oracle)} samples, abs={args.abs_tol:g}, rel={args.rel_tol:g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
