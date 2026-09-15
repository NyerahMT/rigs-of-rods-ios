#include "AuthoredVehicleRuntime.h"
#include "SimConstants.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
using namespace RoR::IOSVehicleCore;
namespace{std::string Read(const char*p){std::ifstream f(p,std::ios::binary);if(!f)return {};std::ostringstream s;s<<f.rdbuf();return s.str();}float Dist(const PhysicsVec3&a,const PhysicsVec3&b){float x=a.x-b.x,z=a.z-b.z;return std::sqrt(x*x+z*z);}}
int main(int argc,char**argv){
 if(argc!=2){std::cerr<<"usage: ror_authored_vehicle_smoke <vehicle.truck>\n";return 64;}
 const std::string text=Read(argv[1]);if(text.empty()){std::cerr<<"FAIL: cannot read vehicle\n";return 1;}
 AuthoredVehicleRuntime v(text);
 std::cerr<<"vehicle="<<v.VehicleName()<<" nodes="<<v.NodeCount()<<" tyres="<<v.TireNodeIndices().size()<<" beams="<<v.BeamPairs().size()<<'\n';
 for(const auto&w:v.Warnings())std::cerr<<"warning: "<<w<<'\n';
 for(const auto&e:v.Errors())std::cerr<<"error: "<<e<<'\n';
 if(!v.Ready()){std::cerr<<"FAIL: runtime did not build\n";return 2;}
 v.SetControls(0,0,0,false);for(int i=0;i<3000;++i)v.Step(PHYSICS_DT);
 if(!v.IsFinite()){std::cerr<<"FAIL: vehicle unstable while settling\n";return 3;}
 const auto s=v.Telemetry();
 v.SetControls(0,.60f,0,false);for(int i=0;i<5000;++i)v.Step(PHYSICS_DT);
 if(!v.IsFinite()){std::cerr<<"FAIL: vehicle unstable under power\n";return 4;}
 const auto p=v.Telemetry();
 const float travel=Dist(s.center,p.center);
 std::cerr<<"travel="<<travel<<" speed="<<p.speed_mps<<" forward="<<p.forward_speed_mps<<" tread="<<p.driven_wheel_speed_mps<<'\n';
 if(!(travel>.05f&&p.driven_wheel_speed_mps>.10f)){std::cerr<<"FAIL: reference vehicle does not drive\n";return 5;}
 std::cout<<"authored reference vehicle smoke probe passed\n";return 0;
}
