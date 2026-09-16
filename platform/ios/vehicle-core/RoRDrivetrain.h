#pragma once

#include <vector>

namespace RoR { namespace IOSVehicleCore {

struct RoRDrivetrainConfig
{
    float shift_down_rpm = 1000.0f; // Engine::m_engine_min_rpm
    float shift_up_rpm = 2500.0f;   // Engine::m_engine_max_rpm
    float engine_torque = 1000.0f;
    float differential_ratio = 1.0f;
    float reverse_gear_ratio = 1.0f;
    float neutral_gear_ratio = 0.0f;
    std::vector<float> forward_gears;

    float engine_inertia = 10.0f;
    char engine_type = 't';
    // Negative means let Engine::SetEngineOptions() select the type default:
    // 5000 for car/electric, 10000 for truck.
    float clutch_force = -1.0f;
    float shift_time = 0.5f;
    float clutch_time = 0.2f;
    float post_shift_time = 0.2f;
    float idle_rpm = -1.0f; // <=0 => min(engine_min_rpm, 800)
    float stall_rpm = 300.0f;
    float max_idle_mixture = 0.1f;
    float min_idle_mixture = 0.0f;
    // Positive authored magnitude; <=0 => engine_torque/5 like Engine ctor.
    float engine_braking_torque = -1.0f;
};

struct RoRDrivetrainTelemetry
{
    float engine_rpm = 0.0f;
    float engine_torque = 0.0f;
    float clutch = 0.0f;
    float clutch_torque = 0.0f;
    float wheel_spin_rpm = 0.0f;
    float drive_ratio = 0.0f;
    float current_acc = 0.0f;
    int gear = 0;
    bool running = false;
    bool shifting = false;
    bool post_shifting = false;
};

/// Renderer-independent mechanical land-vehicle engine path extracted from
/// RoR::Engine. The runtime starts it through the same effective state as
/// Engine::startEngine() in automatic mode: running, DRIVE selected, first gear.
class RoRDrivetrain
{
public:
    RoRDrivetrain();
    explicit RoRDrivetrain(const RoRDrivetrainConfig& config);

    void Configure(const RoRDrivetrainConfig& config);
    void Reset();
    void Start();
    void Stop();
    void Step(float throttle, float wheel_spin_rpm, float dt);

    float OutputTorque() const;
    float DriveRatio() const;
    int Gear() const;
    const RoRDrivetrainTelemetry& Telemetry() const;

private:
    float EnginePower(float rpm) const;
    float IdleMixture() const;
    float InternalRatio(int gear) const;
    void RequestAutomaticShift();
    void BeginShift(int delta);
    void UpdateShiftState(float dt);
    void UpdateAutoClutch(float acc);

    RoRDrivetrainConfig m_config;
    RoRDrivetrainTelemetry m_state;
    float m_idle_rpm = 800.0f;
    float m_braking_torque = -200.0f;
    float m_auto_acc = 0.0f;
    float m_cur_acc = 0.0f;
    float m_shift_clock = 0.0f;
    float m_post_shift_clock = 0.0f;
    int m_shift_value = 0;
};

} }
