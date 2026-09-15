#include "AuthoredVehicleRuntime.h"
#include "PortableRigDef.h"
#include "SimConstants.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
using namespace RoR; using namespace RoR::IOSVehicleCore;
#ifndef ROR_DAF_FIXTURE_PATH
#error ROR_DAF_FIXTURE_PATH must point to the pinned authored DAF truck file
#endif
namespace { void Require(bool c,const char*m){if(!c){std::cerr<<"FAIL: "<<m<<'\n';std::exit(EXIT_FAILURE);}} std::string ReadFile(const char*p){std::ifstream i(p,std::ios::binary);Require((bool)i,"fixture opens");std::ostringstream o;o<<i.rdbuf();return o.str();} float HD(const PhysicsVec3&a,const PhysicsVec3&b){float x=a.x-b.x,z=a.z-b.z;return std::sqrt(x*x+z*z);} float AD(float a,float b){float d=a-b;while(d>3.14159265f)d-=6.28318531f;while(d<-3.14159265f)d+=6.28318531f;return d;} }
int main(){static_assert(PHYSICS_DT==0.0005f,"RoR 2 kHz timestep");const std::string text=ReadFile(ROR_DAF_FIXTURE_PATH);const PortableRigDef::Document p=PortableRigDef::Parse(text);
Require(p.name=="Daf Semi truck","title");Require(p.globals.present&&std::fabs(p.globals.dry_mass-10000)<.1f,"dry mass");Require(p.nodes.size()==79,"79 authored nodes");Require(p.beams.size()>200,"authored beams");Require(p.shocks.size()==8,"shocks");Require(p.hydros.size()==4,"hydros");Require(p.wheels.size()==4,"wheels");Require(p.contacters.size()==15,"contacters");Require(p.engine.present&&std::fabs(p.engine.torque-8000)<.1f,"engine");
AuthoredVehicleRuntime t(text);if(!t.Ready())for(const auto&e:t.Errors())std::cerr<<"runtime error: "<<e<<'\n';Require(t.Ready(),"DAF runtime builds");Require(t.NodeCount()==175,"generated node count");Require(t.TireNodeIndices().size()==96,"tire nodes");Require(t.BeamPairs().size()>600,"beam network");
t.SetControls(0,0,0,false);for(int i=0;i<3000;++i)t.Step(PHYSICS_DT);Require(t.IsFinite(),"settles finite");auto settled=t.Telemetry();auto axis0=t.Node(36).position-t.Node(6).position;
t.SetControls(0,.60f,0,false);for(int i=0;i<5000;++i)t.Step(PHYSICS_DT);Require(t.IsFinite(),"powered finite");auto powered=t.Telemetry();float travel=HD(powered.center,settled.center);std::cerr<<"launch travel="<<travel<<" body="<<powered.speed_mps<<" forward="<<powered.forward_speed_mps<<" tread="<<powered.driven_wheel_speed_mps<<'\n';Require(travel>.20f,"powered travel");Require(powered.driven_wheel_speed_mps>.50f,"rear wheels rotate");Require(powered.forward_speed_mps>.05f,"authored forward direction");
float h0=powered.heading_radians;t.SetControls(.72f,.30f,0,false);for(int i=0;i<4200;++i)t.Step(PHYSICS_DT);Require(t.IsFinite(),"steering finite");auto steering=t.Telemetry();auto axis1=t.Node(36).position-t.Node(6).position;float hm=HD(axis1,axis0),hd=std::fabs(AD(steering.heading_radians,h0));Require(std::fabs(steering.steering)>.25f,"hydro state");Require(hm>.01f,"hydro moves axle");Require(hd>.002f,"road forces yaw");
float tread=steering.driven_wheel_speed_mps;t.SetControls(-.20f,0,1,true);for(int i=0;i<3200;++i)t.Step(PHYSICS_DT);Require(t.IsFinite(),"braking finite");auto stopped=t.Telemetry();Require(stopped.driven_wheel_speed_mps<tread*.80f,"brakes reduce tread speed");std::cout<<"RoR authored DAF strict regression passed\n";return EXIT_SUCCESS;}
