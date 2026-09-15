#include "RoRMassDistribution.h"
#include <algorithm>
namespace RoR { namespace IOSVehicleCore {
MassDistributionResult CalculateRoRNodeMasses(float dry_mass,float load_mass,const std::vector<MassNodeInput>& nodes,const std::vector<MassBeamInput>& beams,bool skip_loaded){
 MassDistributionResult r;r.masses.resize(nodes.size(),0);
 for(std::size_t i=0;i<nodes.size();++i){if(nodes[i].tyre){r.masses[i]=nodes[i].tyre_mass;continue;}if(nodes[i].loaded&&!nodes[i].override_mass)++r.shared_load_nodes;}
 for(std::size_t i=0;i<nodes.size();++i){const auto&n=nodes[i];if(n.tyre)continue;if(!n.loaded)r.masses[i]=0;else if(!n.override_mass)r.masses[i]=r.shared_load_nodes?load_mass/static_cast<float>(r.shared_load_nodes):0;else r.masses[i]=n.override_weight;}
 for(const auto&b:beams){if(b.virtual_beam||b.a>=nodes.size()||b.b>=nodes.size())continue;float half=b.reference_length*.5f;if(!nodes[b.a].tyre)r.participating_beam_length+=half;if(!nodes[b.b].tyre)r.participating_beam_length+=half;}
 if(r.participating_beam_length>0){for(const auto&b:beams){if(b.virtual_beam||b.a>=nodes.size()||b.b>=nodes.size())continue;float half_mass=b.reference_length*dry_mass/r.participating_beam_length*.5f;if(!nodes[b.a].tyre)r.masses[b.a]+=half_mass;if(!nodes[b.b].tyre)r.masses[b.b]+=half_mass;}}
 for(std::size_t i=0;i<nodes.size();++i){if(nodes[i].tyre)continue;if(skip_loaded&&nodes[i].loaded)continue;if(r.masses[i]<nodes[i].minimass)r.masses[i]=nodes[i].minimass;}
 for(float m:r.masses)r.total_mass+=m;return r;
}
} }
