#!/usr/bin/env python3
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_wheel_format_parity.py <AuthoredVehicleRuntime.cpp>")

runtime = Path(sys.argv[1]).resolve()
root = runtime.parents[3]


def patch_once(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        return
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected one anchor in {path}, found {count}")
    path.write_text(text.replace(old, new, 1))


header = root / "source/main/resources/rig_def_fileformat/PortableRigDef.h"
patch_once(
    header,
    'struct Wheel { bool wheels2=false; float rim_radius=0,tire_radius=0,width=0; int num_rays=0; std::string axis_node_0,axis_node_1,rigidity_node; int braking=0,propulsion=0; std::string reference_arm_node; float mass=0,rim_spring=0,rim_damping=0,tire_spring=0,tire_damping=0; };',
    'struct Wheel { bool wheels2=false; bool meshwheel=false; bool meshwheels2=false; float rim_radius=0,tire_radius=0,width=0; int num_rays=0; std::string axis_node_0,axis_node_1,rigidity_node; int braking=0,propulsion=0; std::string reference_arm_node; float mass=0,rim_spring=0,rim_damping=0,tire_spring=0,tire_damping=0; char side=\'r\'; std::string mesh_name,material_name; };',
    "PortableRigDef wheel metadata")

cpp = root / "source/main/resources/rig_def_fileformat/PortableRigDef.cpp"
patch_once(
    cpp,
    '''    Wheels,
    Wheels2,
    MeshWheels2,
''',
    '''    Wheels,
    Wheels2,
    MeshWheels,
    MeshWheels2,
''',
    "meshwheels section enum")
patch_once(
    cpp,
    '''    if (keyword == "wheels") return Section::Wheels;
    if (keyword == "wheels2") return Section::Wheels2;
    if (keyword == "meshwheels2") return Section::MeshWheels2;
''',
    '''    if (keyword == "wheels") return Section::Wheels;
    if (keyword == "wheels2") return Section::Wheels2;
    if (keyword == "meshwheels") return Section::MeshWheels;
    if (keyword == "meshwheels2") return Section::MeshWheels2;
''',
    "meshwheels section keyword")
patch_once(
    cpp,
    '        "materialflarebindings", "meshwheels", "particles", "pistonprops", "props", "railgroups",',
    '        "materialflarebindings", "particles", "pistonprops", "props", "railgroups",',
    "meshwheels unsupported removal")

old_case = '''            case Section::MeshWheels2:
                if (tokens.size() >= 16)
                {
                    Wheel wheel;
                    wheel.wheels2 = false;
                    wheel.tire_radius = F(tokens[0]); wheel.rim_radius = F(tokens[1]); wheel.width = F(tokens[2]); wheel.num_rays = I(tokens[3]);
                    wheel.axis_node_0 = tokens[4]; wheel.axis_node_1 = tokens[5]; wheel.rigidity_node = tokens[6];
                    wheel.braking = I(tokens[7]); wheel.propulsion = I(tokens[8]); wheel.reference_arm_node = tokens[9];
                    wheel.mass = F(tokens[10]); wheel.rim_spring = beam_defaults.spring; wheel.rim_damping = beam_defaults.damping;
                    wheel.tire_spring = F(tokens[11]); wheel.tire_damping = F(tokens[12]);
                    document.wheels.push_back(wheel);
                }
                else Warn(document, line_number, "meshwheels2 requires 16 values");
                break;
'''
new_case = '''            case Section::MeshWheels:
            case Section::MeshWheels2:
                if (tokens.size() >= 16)
                {
                    Wheel wheel;
                    wheel.wheels2 = false;
                    wheel.meshwheel = section == Section::MeshWheels;
                    wheel.meshwheels2 = section == Section::MeshWheels2;
                    wheel.tire_radius = F(tokens[0]); wheel.rim_radius = F(tokens[1]); wheel.width = F(tokens[2]); wheel.num_rays = I(tokens[3]);
                    wheel.axis_node_0 = tokens[4]; wheel.axis_node_1 = tokens[5]; wheel.rigidity_node = tokens[6];
                    wheel.braking = I(tokens[7]); wheel.propulsion = I(tokens[8]); wheel.reference_arm_node = tokens[9];
                    wheel.mass = F(tokens[10]); wheel.rim_spring = beam_defaults.spring; wheel.rim_damping = beam_defaults.damping;
                    wheel.tire_spring = F(tokens[11]); wheel.tire_damping = F(tokens[12]);
                    if (!tokens[13].empty()) wheel.side = static_cast<char>(std::tolower(static_cast<unsigned char>(tokens[13][0])));
                    wheel.mesh_name = tokens[14];
                    wheel.material_name = tokens[15];
                    document.wheels.push_back(wheel);
                }
                else Warn(document, line_number, section == Section::MeshWheels2 ? "meshwheels2 requires 16 values" : "meshwheels requires 16 values");
                break;
'''
patch_once(cpp, old_case, new_case, "meshwheel metadata parse")

# Now that the parser preserves the keyword, remove the temporary rim-radius
# inference used by the topology transform and use the actual authored format.
text = runtime.read_text()
old = 'const float spoke_max_extension = (!def.wheels2 && def.rim_radius > 0.0f) ? 0.15f : 0.0f;'
new = 'const float spoke_max_extension = def.meshwheels2 ? 0.15f : 0.0f;'
if new not in text:
    if text.count(old) != 1:
        raise SystemExit("runtime meshwheels2 extension anchor drifted")
    runtime.write_text(text.replace(old, new, 1))

print('preserved meshwheels/meshwheels2 keyword, side, rim mesh and tyre material metadata')
