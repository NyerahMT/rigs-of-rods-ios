#pragma once
#include <cstddef>
#include <vector>
namespace RoR { namespace IOSVehicleCore {
struct MassNodeInput { bool tyre=false; bool loaded=false; bool override_mass=false; float override_weight=0; float minimass=50; float tyre_mass=0; };
struct MassBeamInput { std::size_t a=0,b=0; float reference_length=0; bool virtual_beam=false; };
struct MassDistributionResult { std::vector<float> masses; float total_mass=0; float participating_beam_length=0; std::size_t shared_load_nodes=0; };
MassDistributionResult CalculateRoRNodeMasses(float dry_mass,float load_mass,const std::vector<MassNodeInput>& nodes,const std::vector<MassBeamInput>& beams,bool minimass_skip_loaded_nodes=false);
} }
