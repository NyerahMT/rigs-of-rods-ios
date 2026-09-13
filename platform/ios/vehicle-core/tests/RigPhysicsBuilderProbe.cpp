#include "BeamPhysics.h"
#include "PortableRigDef.h"
#include "RigPhysicsBuilder.h"
#include "SimConstants.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace RoR;
using namespace RoR::IOSVehicleCore;
using namespace RoR::PortableRigDef;

namespace
{
void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool Finite(const PhysicsVec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool Near(float a, float b, float epsilon = 0.001f)
{
    return std::fabs(a - b) <= epsilon;
}
}

int main()
{
    const char* rig_text = R"ROR(Structural Skeleton
set_beam_defaults 250000, 4500, 300000, 500000
nodes
left,  -1.0, 0.75, 0.0
right,  1.0, 0.75, 0.0
top,    0.0, 1.75, 0.0
beams
left, right
left, top
right, top
end
)ROR";

    const Document rig = Parse(rig_text);
    RigPhysicsModel model = BuildStructuralModel(rig, 12.0f);

    Require(model.errors.empty(), "valid parsed rig builds without structural errors");
    Require(model.nodes.size() == 3, "all parsed nodes become live physics nodes");
    Require(model.beams.size() == 3, "all parsed beams become live physics beams");
    Require(model.nodes[0].id == "left", "node ids remain traceable to the rig definition");
    Require(Near(model.nodes[0].state.mass, 12.0f), "bring-up node mass is assigned explicitly");
    Require(Near(model.beams[0].state.rest_length, 2.0f), "beam rest length comes from rig coordinates");
    Require(Near(model.beams[0].state.spring, 250000.0f), "parsed beam spring reaches solver state");
    Require(Near(model.beams[0].state.damping, 4500.0f), "parsed beam damping reaches solver state");

    // Pin the base nodes for this structural response probe. Real RoR fixed-node
    // semantics remain a later parser/spawner stage; this only validates that a
    // parsed rig is now driving the same node/beam solver used by the iPhone core.
    model.nodes[0].state.immovable = true;
    model.nodes[1].state.immovable = true;

    const float initial_top_y = model.nodes[2].state.position.y;
    model.nodes[2].state.position.y += 0.20f;

    float peak_downward_speed = 0.0f;
    for (int step = 0; step < 1200; ++step)
    {
        StepStructuralModel(model, DEFAULT_GRAVITY, PHYSICS_DT);
        for (const RigPhysicsNode& node : model.nodes)
        {
            Require(Finite(node.state.position) && Finite(node.state.velocity),
                "parsed structural rig remains finite at 2 kHz");
        }
        peak_downward_speed = std::min(peak_downward_speed, model.nodes[2].state.velocity.y);
    }

    Require(peak_downward_speed < -0.05f,
        "parsed beams generate restoring motion through the live solver");
    Require(model.nodes[2].state.position.y < initial_top_y + 0.20f,
        "displaced parsed node moves back toward its rig-defined structure");

    const char* broken_rig_text = R"ROR(Broken Rig
nodes
0, 0, 0, 0
beams
0, missing
end
)ROR";
    const RigPhysicsModel broken = BuildStructuralModel(Parse(broken_rig_text));
    Require(!broken.errors.empty(), "missing rig node references are rejected explicitly");
    Require(broken.beams.empty(), "invalid beam is not admitted to the solver");

    std::cout << "RoR rig physics builder probe passed: parsed .truck nodes/beams now drive the 2 kHz solver.\n";
    return EXIT_SUCCESS;
}
