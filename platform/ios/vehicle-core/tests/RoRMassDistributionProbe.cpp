#include "RoRMassDistribution.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
using namespace RoR::IOSVehicleCore;
static void R(bool c,const char*m){if(!c){std::cerr<<"FAIL: "<<m<<'\n';std::exit(EXIT_FAILURE);}}
static bool N(float a,float b){return std::fabs(a-b)<.001f;}
int main(){std::vector<MassNodeInput> n(4);n[0].minimass=50;n[1].loaded=true;n[1].minimass=50;n[2].loaded=true;n[2].override_mass=true;n[2].override_weight=120;n[2].minimass=50;n[3].tyre=true;n[3].tyre_mass=20;
std::vector<MassBeamInput>b={{0,1,2,false},{1,2,1,false},{2,3,1,false},{0,2,100,true}};auto r=CalculateRoRNodeMasses(700,300,n,b,false);R(r.shared_load_nodes==1,"shared loaded-node count");R(N(r.participating_beam_length,3.5f),"non-virtual non-tyre half-length sum");R(N(r.masses[0],200),"normal mass from beam reference length");R(N(r.masses[1],600),"shared load plus dry mass");R(N(r.masses[2],320),"override load plus dry mass");R(N(r.masses[3],20),"tyre mass excluded from redistribution");R(N(r.total_mass,1140),"total mass");std::cout<<"RoR mass distribution parity probe passed\n";return EXIT_SUCCESS;}
