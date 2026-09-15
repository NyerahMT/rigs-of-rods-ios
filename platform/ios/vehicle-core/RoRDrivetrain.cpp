#include "RoRDrivetrain.h"

#include <algorithm>
#include <cmath>

namespace RoR { namespace IOSVehicleCore {
namespace {
float Clamp(float v,float lo,float hi){return std::max(lo,std::min(v,hi));}
}

RoRDrivetrain::RoRDrivetrain(){Configure(RoRDrivetrainConfig());}
RoRDrivetrain::RoRDrivetrain(const RoRDrivetrainConfig& config){Configure(config);}

void RoRDrivetrain::Configure(const RoRDrivetrainConfig& config)
{
    m_config=config;
    m_config.shift_down_rpm=std::max(1.0f,std::fabs(m_config.shift_down_rpm));
    m_config.shift_up_rpm=std::max(m_config.shift_down_rpm+1.0f,std::fabs(m_config.shift_up_rpm));
    m_config.engine_torque=std::fabs(m_config.engine_torque);
    m_config.differential_ratio=std::fabs(m_config.differential_ratio)>1.0e-5f?std::fabs(m_config.differential_ratio):1.0f;
    m_config.engine_inertia=std::max(0.01f,m_config.engine_inertia);
    m_config.clutch_force=std::max(0.0f,m_config.clutch_force);
    m_config.stall_rpm=std::max(0.0f,std::min(m_config.stall_rpm,m_config.shift_down_rpm*0.9f));
    if(m_config.engine_braking_torque>=0.0f)m_config.engine_braking_torque=-m_config.engine_torque/5.0f;
    m_idle_rpm=std::min(m_config.shift_down_rpm,800.0f);
    Reset();
}

void RoRDrivetrain::Reset()
{
    m_state=RoRDrivetrainTelemetry();
    m_state.gear=m_config.forward_gears.empty()?0:1;
    m_state.running=true;
    m_state.engine_rpm=m_idle_rpm;
    m_state.drive_ratio=InternalRatio(m_state.gear);
    m_throttle=0.0f;
    m_shift_cooldown=0.0f;
}
void RoRDrivetrain::Start(){m_state.running=true;if(m_state.engine_rpm<m_idle_rpm)m_state.engine_rpm=m_idle_rpm;if(m_state.gear==0&&!m_config.forward_gears.empty())m_state.gear=1;}
void RoRDrivetrain::Stop(){m_state.running=false;m_state.clutch=0.0f;m_state.clutch_torque=0.0f;}

float RoRDrivetrain::InternalRatio(int gear) const
{
    if(gear<1||static_cast<std::size_t>(gear)>m_config.forward_gears.size())return 0.0f;
    return m_config.forward_gears[static_cast<std::size_t>(gear-1)]*m_config.differential_ratio;
}

float RoRDrivetrain::EnginePower(float rpm) const
{
    // RoR's stock engine is intentionally close to a flat-torque model. The
    // configured shift-up RPM is nominal max RPM; hard redline is 1.25x.
    float tq=1.0f;
    const float maxrpm=m_config.shift_up_rpm;
    const float redline=maxrpm*1.25f;
    if(rpm>maxrpm)tq=Clamp((redline-rpm)/(redline-maxrpm),0.0f,1.0f);
    if(rpm<0.0f)tq=0.0f;
    return m_config.engine_torque*tq;
}

float RoRDrivetrain::IdleMixture() const
{
    return m_state.engine_rpm<=m_idle_rpm?m_config.max_idle_mixture:m_config.min_idle_mixture;
}

void RoRDrivetrain::UpdateAutomaticGear(float dt)
{
    if(m_state.gear<=0||m_config.forward_gears.empty())return;
    if(m_shift_cooldown>0.0f){m_shift_cooldown=std::max(0.0f,m_shift_cooldown-dt);return;}
    const int top=static_cast<int>(m_config.forward_gears.size());
    if(m_state.engine_rpm>m_config.shift_up_rpm&&m_state.gear<top)
    {
        ++m_state.gear;
        m_state.clutch=0.0f;
        m_shift_cooldown=0.20f;
    }
    else if(m_state.engine_rpm<m_config.shift_down_rpm&&m_state.gear>1)
    {
        --m_state.gear;
        m_state.clutch=0.0f;
        m_shift_cooldown=0.20f;
    }
}

void RoRDrivetrain::Step(float throttle,float wheel_spin_rpm,float dt)
{
    if(dt<=0.0f)return;
    m_throttle=Clamp(throttle,0.0f,1.0f);
    m_state.wheel_spin_rpm=wheel_spin_rpm;
    if(!m_state.running){m_state.clutch_torque=0.0f;return;}

    UpdateAutomaticGear(dt);
    m_state.drive_ratio=InternalRatio(m_state.gear);
    const float ratio=m_state.drive_ratio;

    // Engine::UpdateEngine(): idle mixture, engine power, compression braking,
    // previous clutch load, then inertia integration.
    const float acc=std::max(m_throttle,IdleMixture());
    m_state.engine_torque=EnginePower(m_state.engine_rpm)*acc;
    float total_torque=m_state.engine_torque;
    total_torque+=m_config.engine_braking_torque*(m_state.engine_rpm/m_config.shift_up_rpm)*(1.0f-acc);
    if(m_state.gear!=0&&std::fabs(ratio)>1.0e-6f)
        total_torque-=m_state.clutch_torque/ratio;
    m_state.engine_rpm+=dt*total_torque/m_config.engine_inertia;
    m_state.engine_rpm=std::max(0.0f,m_state.engine_rpm);

    if(m_state.gear==0||std::fabs(ratio)<=1.0e-6f)
    {
        m_state.clutch=0.0f;
        m_state.clutch_torque=0.0f;
        return;
    }

    // RoR automatic clutch. The clutch stays open below the configured
    // shift-down RPM, then progressively couples engine and driveshaft.
    const float declutch_rpm=m_config.shift_down_rpm*0.75f+m_config.stall_rpm*0.25f;
    if(m_state.engine_rpm<declutch_rpm)
    {
        m_state.clutch=0.0f;
    }
    else if(m_state.engine_rpm<m_config.shift_down_rpm&&m_config.shift_down_rpm>declutch_rpm)
    {
        const float c=(m_state.engine_rpm-declutch_rpm)/(m_config.shift_down_rpm-declutch_rpm);
        m_state.clutch=std::min(c*c,m_state.clutch);
    }
    else if(m_state.engine_rpm>m_config.shift_down_rpm&&m_state.clutch<1.0f&&m_shift_cooldown<=0.0f)
    {
        const float first_ratio=std::fabs(InternalRatio(1));
        const float threshold=1.5f*std::max(m_config.engine_torque,EnginePower(m_state.engine_rpm))*first_ratio;
        const float gearbox_spinner=m_state.engine_rpm/ratio;
        const float clutch_torque=(gearbox_spinner-wheel_spin_rpm)*m_config.clutch_force;
        const float limited=Clamp(clutch_torque,-threshold,threshold);
        const float reaction=limited/ratio;
        const float full_range=std::max(1.0f,m_config.shift_up_rpm-m_config.shift_down_rpm);
        const float range=full_range*0.4f*std::sqrt(std::max(0.2f,acc));
        const float power_ratio=Clamp((m_state.engine_rpm-m_config.shift_down_rpm)/std::max(1.0f,range),0.0f,1.0f);
        const float engine_torque=EnginePower(m_state.engine_rpm)*std::min(acc,0.9f)*power_ratio;
        if(std::fabs(reaction)>1.0e-6f)
        {
            const float torque_diff=std::min(engine_torque,std::fabs(reaction));
            m_state.clutch=std::max(m_state.clutch,Clamp(torque_diff/std::fabs(reaction),0.0f,1.0f));
        }
    }
    m_state.clutch=Clamp(m_state.clutch,0.0f,1.0f);

    // Engine::UpdateEngine() clutch output. This is the torque returned by
    // Engine::getTorque() and consumed by Actor::CalcDifferentials().
    const float first_ratio=std::fabs(InternalRatio(1));
    const float force_threshold=1.5f*std::max(m_config.engine_torque,EnginePower(m_state.engine_rpm))*first_ratio;
    const float gearbox_spinner=m_state.engine_rpm/ratio;
    float clutch_torque=(gearbox_spinner-wheel_spin_rpm)*m_state.clutch*m_config.clutch_force;
    clutch_torque=Clamp(clutch_torque,-force_threshold,force_threshold);
    clutch_torque*=1.0f-std::exp(-std::fabs(gearbox_spinner-wheel_spin_rpm));
    m_state.clutch_torque=clutch_torque;
}

float RoRDrivetrain::OutputTorque() const{return m_state.clutch_torque;}
float RoRDrivetrain::DriveRatio() const{return m_state.drive_ratio;}
int RoRDrivetrain::Gear() const{return m_state.gear;}
const RoRDrivetrainTelemetry& RoRDrivetrain::Telemetry() const{return m_state;}

} }
