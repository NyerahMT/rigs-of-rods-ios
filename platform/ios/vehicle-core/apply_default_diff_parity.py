#!/usr/bin/env python3
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_default_diff_parity.py <AuthoredVehicleRuntime.cpp>")

p = Path(sys.argv[1])
s = p.read_text()


def one(old: str, new: str, label: str) -> None:
    global s
    if new in s:
        return
    if s.count(old) != 1:
        raise SystemExit(f"default diff parity anchor '{label}' expected once, found {s.count(old)}")
    s = s.replace(old, new, 1)


one(
    '#include "GroundContact.h"\n',
    '#include "GroundContact.h"\n#include "Differentials.h"\n',
    "Differentials include")

one(
'''        drivetrain.Step(throttle_state, driven_wheel_rpm, dt);
        const float per_driven_torque = driven_count > 0
            ? drivetrain.OutputTorque() / static_cast<float>(driven_count)
            : 0.0f;
        const float service_force = rig.brakes.present ? std::max(0.0f, rig.brakes.service_force) : 30000.0f;
''',
'''        drivetrain.Step(throttle_state, driven_wheel_rpm, dt);

        // ActorSpawner::FinalizeRig() automatically pairs propelled wheels and
        // gives each pair a VISCOUS_DIFF when the truck has no explicit axle
        // override. Actor::CalcDifferentials() first splits engine torque over
        // all propelled wheels, then lets each wheel differential redistribute
        // its pair's torque from the live wheel speeds.
        std::vector<float> drive_torques(wheels.size(), 0.0f);
        std::vector<std::size_t> driven_indices;
        driven_indices.reserve(driven_count);
        const float base_driven_torque = driven_count > 0
            ? drivetrain.OutputTorque() / static_cast<float>(driven_count)
            : 0.0f;
        for (std::size_t i = 0; i < wheels.size(); ++i)
        {
            if (wheels[i].propulsion != 0)
            {
                drive_torques[i] = base_driven_torque;
                driven_indices.push_back(i);
            }
        }
        for (std::size_t pair = 1; pair < driven_indices.size(); pair += 2)
        {
            const std::size_t i0 = driven_indices[pair - 1];
            const std::size_t i1 = driven_indices[pair];
            DifferentialData diff = {
                {wheels[i0].wheel.speed, wheels[i1].wheel.speed},
                0.0f,
                {0.0f, 0.0f},
                drive_torques[i0] + drive_torques[i1],
                dt
            };
            Differential::CalcViscousDiff(diff);
            drive_torques[i0] = diff.out_torque[0];
            drive_torques[i1] = diff.out_torque[1];
        }

        const float service_force = rig.brakes.present ? std::max(0.0f, rig.brakes.service_force) : 30000.0f;
''',
"default viscous differential torque distribution")

one(
'''        for (WheelFixture& fixture : wheels)
        {
            if (fixture.propulsion != 0)
                fixture.wheel.torque += per_driven_torque;
''',
'''        for (WheelFixture& fixture : wheels)
        {
            const std::size_t fixture_index = static_cast<std::size_t>(&fixture - wheels.data());
            if (fixture.propulsion != 0)
                fixture.wheel.torque += drive_torques[fixture_index];
''',
"apply differential wheel torque")

p.write_text(s)
print('matched RoR default propelled-wheel VISCOUS_DIFF pairing and torque redistribution')
