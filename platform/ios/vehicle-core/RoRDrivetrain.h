#pragma once

#include <vector>

namespace RoR { namespace IOSVehicleCore {

struct RoRDrivetrainConfig
{
    float shift_down_rpm = 1000.0f;
    float shift_up_rpm = 2500.0f;
    float engine_torque = 1000.0f;
    float differential_ratio = 1.0f;
    float reverse_gear_ratio = 1.0f;
    float neutral_gear_ratio = 0.0f;
    std::vector<float> forward_gears;

    // Defaults from RoR::Engine when no engoption overrides are authored.
    float engine_inertia = 10.0f;
    float clutch_force = 10000.0f;
    float stall_rpm = 300.0f;
    float max_idle_mixture = 0.1f;
    float min_idle_mixture = 0.0f;
    float engine_braking_torque = 0.0f; // <= 0 means use -engine_torque / 5
};

struct RoRDrivetrainTelemetry
{
    float engine_rpm = 0.0f;
    float engine_torque = 0.0f;
    float clutch = 0.0f;
    float clutch_torque = 0.0f;
    float wheel_spin_rpm = 0.0f;
    float drive_ratio = 0.0f;
    int gear = 0;
    bool running = false;
};

/// Renderer-independent extraction of the mechanical land-vehicle engine path
/// used by RoR::Engine + Actor::CalcDifferentials(). It deliberately keeps
/// transmission torque in the same units as Engine::getTorque(): clutch/output
/// shaft Nm, which Actor then divides among propulsed wheels.
class RoRDrivetrain
{
public:
    RoRDrivetrain();
    explicit RoRDrivetrain(const RoRDrivetrainConfig& config);

    void Configure(const RoRDrivetrainConfig& config);
    void Reset();
    void Start();
    void Stop();

    /// wheel_spin_rpm is the average propulsed-wheel angular speed, matching
    /// Actor::CalcWheels() -> Engine::setWheelSpin().
    void Step(float throttle, float wheel_spin_rpm, float dt);

    float OutputTorque() const;
    float DriveRatio() const;
    int Gear() const;
    const RoRDrivetrainTelemetry& Telemetry() const;

private:
    float EnginePower(float rpm) const;
    float IdleMixture() const;
    float InternalRatio(int gear) const;
    void UpdateAutomaticGear(float dt);

    RoRDrivetrainConfig m_config;
    RoRDrivetrainTelemetry m_state;
    float m_idle_rpm = 800.0f;
    float m_throttle = 0.0f;
    float m_shift_cooldown = 0.0f;
};

} }
