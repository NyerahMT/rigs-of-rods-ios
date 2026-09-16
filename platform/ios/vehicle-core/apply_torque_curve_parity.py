#!/usr/bin/env python3
"""Port RigDef `torquecurve` into the portable engine path.

The real upstream parser accepts either a single predefined torque-model name or
custom `rpm, torque_multiplier` samples. Custom samples are executed by
RoRDrivetrain using Ogre::SimpleSpline-compatible interpolation. Predefined names
are preserved but remain audit-visible until the pinned torque_models.cfg data is
packaged into the iOS runtime.
"""
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_torque_curve_parity.py <AuthoredVehicleRuntime.cpp>")

runtime = Path(sys.argv[1]).resolve()
root = runtime.parents[3]
parser = root / "source/main/resources/rig_def_fileformat/PortableRigDef.cpp"


def once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"torquecurve parity anchor '{label}' expected once, found {count}")
    return text.replace(old, new, 1)


s = parser.read_text()
s = once(
    s,
'''    Engine,
    EngOption,
    Brakes,
''',
'''    Engine,
    EngOption,
    TorqueCurve,
    Brakes,
''',
    "torquecurve section enum")

s = once(
    s,
'''    if (keyword == "engine") return Section::Engine;
    if (keyword == "engoption") return Section::EngOption;
    if (keyword == "brakes") return Section::Brakes;
''',
'''    if (keyword == "engine") return Section::Engine;
    if (keyword == "engoption") return Section::EngOption;
    if (keyword == "torquecurve") return Section::TorqueCurve;
    if (keyword == "brakes") return Section::Brakes;
''',
    "torquecurve section keyword")

s = once(
    s,
'''        "slidenodes", "soundsources", "soundsources2", "submesh", "texcoords", "ties",
        "torquecurve", "transfercase", "triggers", "turbojets", "turboprops", "turboprops2",
''',
'''        "slidenodes", "soundsources", "soundsources2", "submesh", "texcoords", "ties",
        "transfercase", "triggers", "turbojets", "turboprops", "turboprops2",
''',
    "torquecurve unsupported removal")

case = '''            case Section::TorqueCurve:
                // RigDef::Parser::ParseTorqueCurve(): a one-token line selects
                // a predefined model; two tokens append a custom spline point.
                document.torque_curve.present = true;
                if (tokens.size() == 1)
                {
                    document.torque_curve.predefined_model = tokens[0];
                }
                else if (tokens.size() == 2)
                {
                    TorqueCurveSample sample;
                    sample.rpm = F(tokens[0]);
                    sample.torque_multiplier = F(tokens[1]);
                    document.torque_curve.samples.push_back(sample);
                }
                else
                {
                    Warn(document, line_number, "torquecurve line requires model name or rpm,multiplier");
                }
                break;

'''
s = once(
    s,
'            case Section::Brakes:\n',
    case + '            case Section::Brakes:\n',
    "torquecurve parser case")
parser.write_text(s)

r = runtime.read_text()
insert = '''        if (rig.torque_curve.present)
        {
            drivetrain_config.predefined_torque_model = rig.torque_curve.predefined_model;
            drivetrain_config.torque_curve_samples.clear();
            drivetrain_config.torque_curve_samples.reserve(rig.torque_curve.samples.size());
            for (const PortableRigDef::TorqueCurveSample& source : rig.torque_curve.samples)
            {
                RoRTorqueCurveSample sample;
                sample.rpm = source.rpm;
                sample.torque_multiplier = source.torque_multiplier;
                drivetrain_config.torque_curve_samples.push_back(sample);
            }
            if (!rig.torque_curve.predefined_model.empty())
                Warn("predefined torquecurve model preserved but awaits pinned torque_models.cfg resource parity");
        }

        drivetrain.Configure(drivetrain_config);
'''
r = once(
    r,
'        drivetrain.Configure(drivetrain_config);\n',
    insert,
    "drivetrain torque curve configuration")
runtime.write_text(r)

print('ported custom RigDef torque curves into Ogre-SimpleSpline-compatible drivetrain state')
