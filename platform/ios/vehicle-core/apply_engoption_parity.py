#!/usr/bin/env python3
"""Preserve upstream RigDef::Engoption semantics through the portable actor.

RigDef has eleven engoption parameters. The old portable parser kept only the
first six and mapped the two timing names literally. Desktop ActorSpawner has a
historic swap when calling Engine::SetEngineOptions(): RigDef shift_time becomes
Engine clutch-time, while RigDef clutch_time becomes Engine shift-time. This
transform deliberately reproduces that behavior and forwards the remaining RPM,
mixture and braking parameters.
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_engoption_parity.py <AuthoredVehicleRuntime.cpp>")

runtime = Path(sys.argv[1]).resolve()
root = runtime.parents[3]
parser = root / "source/main/resources/rig_def_fileformat/PortableRigDef.cpp"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"engoption parity anchor '{label}' expected once, found {count}")
    return text.replace(old, new, 1)


p = parser.read_text()
old_parse = '''            case Section::EngOption:
                if (!tokens.empty())
                {
                    document.engoption.present = true;
                    document.engoption.inertia = F(tokens[0], 10.0f);
                    if (tokens.size() > 1 && !tokens[1].empty()) document.engoption.type = tokens[1][0];
                    if (tokens.size() > 2) document.engoption.clutch_force = F(tokens[2], -1.0f);
                    if (tokens.size() > 3) document.engoption.shift_time = F(tokens[3], -1.0f);
                    if (tokens.size() > 4) document.engoption.clutch_time = F(tokens[4], -1.0f);
                    if (tokens.size() > 5) document.engoption.post_shift_time = F(tokens[5], -1.0f);
                }
                else Warn(document, line_number, "engoption requires engine inertia");
                section = Section::None;
                break;
'''
new_parse = '''            case Section::EngOption:
                if (!tokens.empty())
                {
                    document.engoption.present = true;
                    document.engoption.inertia = F(tokens[0], 10.0f);
                    if (tokens.size() > 1 && !tokens[1].empty()) document.engoption.type = tokens[1][0];
                    if (tokens.size() > 2) document.engoption.clutch_force = F(tokens[2], -1.0f);
                    if (tokens.size() > 3) document.engoption.shift_time = F(tokens[3], -1.0f);
                    if (tokens.size() > 4) document.engoption.clutch_time = F(tokens[4], -1.0f);
                    if (tokens.size() > 5) document.engoption.post_shift_time = F(tokens[5], -1.0f);
                    if (tokens.size() > 6) document.engoption.stall_rpm = F(tokens[6], -1.0f);
                    if (tokens.size() > 7) document.engoption.idle_rpm = F(tokens[7], -1.0f);
                    if (tokens.size() > 8) document.engoption.max_idle_mixture = F(tokens[8], -1.0f);
                    if (tokens.size() > 9) document.engoption.min_idle_mixture = F(tokens[9], -1.0f);
                    if (tokens.size() > 10) document.engoption.braking_torque = F(tokens[10], -1.0f);
                }
                else Warn(document, line_number, "engoption requires engine inertia");
                section = Section::None;
                break;
'''
p = replace_once(p, old_parse, new_parse, "eleven-value parser")
parser.write_text(p)

r = runtime.read_text()
old_map = '''        if (rig.engoption.present)
        {
            // RigDef::Engoption is not cosmetic. The Bandit, for example,
            // authors 0.075 kg*m^2 inertia rather than Engine's 10.0 default.
            // Ignoring it makes throttle response more than two orders of
            // magnitude too slow before the clutch can transfer torque.
            drivetrain_config.engine_inertia = rig.engoption.inertia;
            if (rig.engoption.clutch_force >= 0.0f)
                drivetrain_config.clutch_force = rig.engoption.clutch_force;
            if (rig.engoption.shift_time > 0.0f)
                drivetrain_config.shift_time = rig.engoption.shift_time;
            if (rig.engoption.clutch_time > 0.0f)
                drivetrain_config.clutch_time = rig.engoption.clutch_time;
            if (rig.engoption.post_shift_time > 0.0f)
                drivetrain_config.post_shift_time = rig.engoption.post_shift_time;
        }
'''
new_map = '''        if (rig.engoption.present)
        {
            drivetrain_config.engine_inertia = rig.engoption.inertia;
            drivetrain_config.engine_type = rig.engoption.type;
            drivetrain_config.clutch_force = rig.engoption.clutch_force;

            // ActorSpawner::ProcessEngoption() passes these two RigDef fields to
            // SetEngineOptions(ctime, stime) in this historical order. Preserve
            // the resulting desktop behavior rather than fixing the names.
            if (rig.engoption.shift_time > 0.0f)
                drivetrain_config.clutch_time = rig.engoption.shift_time;
            if (rig.engoption.clutch_time > 0.0f)
                drivetrain_config.shift_time = rig.engoption.clutch_time;
            if (rig.engoption.post_shift_time > 0.0f)
                drivetrain_config.post_shift_time = rig.engoption.post_shift_time;
            if (rig.engoption.idle_rpm > 0.0f)
                drivetrain_config.idle_rpm = rig.engoption.idle_rpm;
            if (rig.engoption.stall_rpm > 0.0f)
                drivetrain_config.stall_rpm = rig.engoption.stall_rpm;
            if (rig.engoption.max_idle_mixture > 0.0f)
                drivetrain_config.max_idle_mixture = rig.engoption.max_idle_mixture;
            if (rig.engoption.min_idle_mixture > 0.0f)
                drivetrain_config.min_idle_mixture = rig.engoption.min_idle_mixture;
            if (rig.engoption.braking_torque > 0.0f)
                drivetrain_config.engine_braking_torque = rig.engoption.braking_torque;
        }
'''
r = replace_once(r, old_map, new_map, "runtime Engine::SetEngineOptions mapping")
runtime.write_text(r)

print('preserved all eleven engoption parameters and ActorSpawner timing-field swap')
