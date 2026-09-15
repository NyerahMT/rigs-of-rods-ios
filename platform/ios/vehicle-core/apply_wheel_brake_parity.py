#!/usr/bin/env python3
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_wheel_brake_parity.py <AuthoredVehicleRuntime.cpp>")

p = Path(sys.argv[1])
s = p.read_text()


def one(old: str, new: str, label: str) -> None:
    global s
    if new in s:
        return
    if s.count(old) != 1:
        raise SystemExit(f"wheel brake parity anchor '{label}' expected once, found {s.count(old)}")
    s = s.replace(old, new, 1)


one(
'''        std::size_t driven_count = 0;
        std::size_t braked_count = 0;
        for (const WheelFixture& fixture : wheels)
        {
            if (fixture.propulsion != 0) ++driven_count;
            if (fixture.braking != 0) ++braked_count;
        }
''',
'''        std::size_t driven_count = 0;
        for (const WheelFixture& fixture : wheels)
        {
            if (fixture.propulsion != 0) ++driven_count;
        }
''',
"remove non-upstream brake-force division")

one(
'''            float available_brake_torque = 0.0f;
            if (fixture.braking != 0 && braked_count > 0)
            {
                available_brake_torque += brake * service_force * fixture.wheel.radius /
                    static_cast<float>(braked_count);
            }
            if (handbrake && fixture.propulsion != 0)
            {
                const float parking_force = (rig.brakes.present && rig.brakes.parking_force > 0.0f)
                    ? rig.brakes.parking_force
                    : service_force * 2.0f;
                available_brake_torque = std::max(
                    available_brake_torque,
                    parking_force * fixture.wheel.radius / std::max<std::size_t>(1, driven_count));
            }
''',
'''            float available_brake_torque = 0.0f;
            // WheelBraking is serialized directly in truck files:
            //   0 NONE, 1 FOOT_HAND, 2/3 FOOT_HAND_SKID_{L,R}, 4 FOOT_ONLY.
            // Actor::CalcWheels applies ar_brake_force independently to every
            // braked wheel; it is already a torque limit, so do not divide it
            // between wheels or multiply it by radius here.
            if (fixture.braking != 0)
                available_brake_torque += brake * service_force;

            // Parking brake applies to every braking mode except FOOT_ONLY,
            // regardless of whether that wheel is driven.
            if (handbrake && fixture.braking != 0 && fixture.braking != 4)
            {
                const float parking_force = (rig.brakes.present && rig.brakes.parking_force > 0.0f)
                    ? rig.brakes.parking_force
                    : service_force * 2.0f;
                available_brake_torque += parking_force;
            }
''',
"foot/parking brake torque semantics")

p.write_text(s)
print('matched RoR per-wheel footbrake and FOOT_ONLY/handbrake semantics')
