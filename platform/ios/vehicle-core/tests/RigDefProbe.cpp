#include "PortableRigDef.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
using namespace RoR::PortableRigDef;
namespace { void Require(bool c,const char*m){if(!c){std::cerr<<"FAIL: "<<m<<'\n';std::exit(EXIT_FAILURE);}} bool Near(float a,float b,float e=.001f){return std::fabs(a-b)<=e;} }
int main(){const char* rig_text=R"ROR(Portable Drift Coupe
set_beam_defaults_scale 0.5, 1, 1, 1
set_beam_defaults 800000, 9000, 300000, 500000
set_node_defaults 0, 1.25, 0.8, 1.1, c
minimass
42
nodes
0,1.25,0.55,0.78
1,1.25,0.55,-0.78,l
2,-1.25,0.55,0.78,l 120
set_default_minimass 12
set_node_defaults 15, 0.75, 1, 1
3,-1.25,0.55,-0.78
4,1.05,0.85,0.25
5,1.05,0.85,-0.25
beams
0,2
1,3
0,1
2,3
hydros
4,0,0.12,s
5,1,-0.12,s
shocks
0,4,220000,4200,0.70,1.25,1.00
1,5,220000,4200,0.70,1.25,1.00
props
0,1,2,0,0,0,0,0,0,dummy.mesh
wheels2
0.30,0.50,0.26,12,0,4,2,1,0,2,18,350000,4500,180000,2500
0.30,0.50,0.26,12,1,5,3,1,0,3,18,350000,4500,180000,2500
0.30,0.50,0.28,12,2,4,0,2,1,0,20,380000,4800,170000,2400
0.30,0.50,0.28,12,3,5,1,2,1,1,20,380000,4800,170000,2400
engine
1200,6500,410,4.10,-3.20,0,3.05,2.10,1.55,1.18,1.00
brakes
30000,55000
end
)ROR";Document r=Parse(rig_text);
Require(r.name=="Portable Drift Coupe","name");Require(r.nodes.size()==6,"nodes");Require(r.beams.size()==4,"beams");Require(r.wheels.size()==4,"wheels");
Require(Near(r.nodes[0].minimass,42),"minimass section applies");Require(Near(r.nodes[0].friction,1.25),"node friction default");Require(Near(r.nodes[0].volume,.8),"node volume default");Require(r.nodes[0].options.find('c')!=std::string::npos,"default node option");
Require(r.nodes[1].loaded_mass&&!r.nodes[1].override_mass,"plain l node uses shared globals load mass");Require(r.nodes[2].loaded_mass&&r.nodes[2].override_mass&&Near(r.nodes[2].load_weight,120),"explicit l mass overrides load mass");
Require(Near(r.nodes[3].minimass,12),"set_default_minimass applies forward");Require(r.nodes[3].loaded_mass&&r.nodes[3].override_mass&&Near(r.nodes[3].load_weight,15),"default loadweight creates loaded override node");Require(Near(r.nodes[3].friction,.75),"changed friction default");
Require(Near(r.beams[0].spring,400000),"beam-default scale applies");Require(Near(r.beams[0].damping,9000),"beam damping scale");Require(Near(r.hydros[0].spring,400000),"scaled defaults reach hydros");Require(Near(r.hydros[0].lengthening_factor,.12),"hydro factor");
Require(r.engine.present&&Near(r.engine.torque,410),"engine");Require(r.brakes.present&&Near(r.brakes.parking_force,55000),"brakes");Require(r.warnings.empty(),"supported fixture warns cleanly");
std::cout<<"RigDef parity probe passed\n";return EXIT_SUCCESS;}
