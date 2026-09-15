#include "RoRDrivetrain.h"
#include "SimConstants.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
using namespace RoR::IOSVehicleCore;
namespace{void Require(bool c,const char*m){if(!c){std::cerr<<"FAIL: "<<m<<'\n';std::exit(EXIT_FAILURE);}}bool Near(float a,float b,float e=.01f){return std::fabs(a-b)<=e;}}
int main(){
    RoRDrivetrainConfig c;
    c.shift_down_rpm=1000.0f;c.shift_up_rpm=1500.0f;c.engine_torque=8000.0f;c.differential_ratio=2.0f;
    c.reverse_gear_ratio=10.85f;c.neutral_gear_ratio=13.86f;
    const float g[]={9.52f,6.56f,5.48f,4.58f,3.83f,3.02f,2.53f,2.08f,1.74f,1.43f,1.20f,1.00f};
    c.forward_gears.assign(g,g+12);
    RoRDrivetrain d(c);
    Require(d.Gear()==1,"starts in first forward gear");
    Require(Near(d.DriveRatio(),19.04f),"first gear includes global differential ratio");
    Require(Near(d.Telemetry().engine_rpm,800.0f),"RoR default idle RPM");
    for(int i=0;i<1000;++i)d.Step(.60f,0.0f,PHYSICS_DT);
    const auto t=d.Telemetry();
    std::cerr<<"rpm="<<t.engine_rpm<<" clutch="<<t.clutch<<" output="<<t.clutch_torque<<" ratio="<<t.drive_ratio<<" gear="<<t.gear<<'\n';
    Require(std::isfinite(t.engine_rpm)&&std::isfinite(t.clutch_torque),"finite drivetrain state");
    Require(t.engine_rpm>1000.0f,"throttle raises engine above clutch engagement RPM");
    Require(t.clutch>0.0f,"automatic clutch engages");
    Require(t.clutch_torque>10000.0f,"gearbox produces substantial output-shaft torque");
    Require(t.clutch_torque>c.engine_torque,"gear reduction multiplies launch torque");
    d.Step(0.0f,0.0f,PHYSICS_DT);
    Require(std::isfinite(d.OutputTorque()),"coast remains finite");
    std::cout<<"RoR drivetrain parity probe passed\n";
    return EXIT_SUCCESS;
}
