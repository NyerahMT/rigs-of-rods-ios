#include "RoRDrivetrain.h"

#include "ApproxMath.h"

#include <algorithm>
#include <cmath>

namespace RoR { namespace IOSVehicleCore {
namespace {
float Clamp(float v, float lo, float hi) { return std::max(lo, std::min(v, hi)); }
}

RoRDrivetrain::RoRDrivetrain() { Configure(RoRDrivetrainConfig()); }
RoRDrivetrain::RoRDrivetrain(const RoRDrivetrainConfig& config) { Configure(config); }

void RoRDrivetrain::Configure(const RoRDrivetrainConfig& config)
{
    m_config = config;
    m_config.shift_down_rpm = std::max(1.0f, std::fabs(m_config.shift_down_rpm));
    m_config.shift_up_rpm = std::max(m_config.shift_down_rpm + 1.0f, std::fabs(m_config.shift_up_rpm));
    m_config.engine_torque = std::fabs(m_config.engine_torque);
    m_config.differential_ratio = std::fabs(m_config.differential_ratio) > 1.0e-5f
        ? std::fabs(m_config.differential_ratio) : 1.0f;
    m_config.engine_inertia = std::max(1.0e-4f, m_config.engine_inertia);

    if (m_config.clutch_force < 0.0f)
        m_config.clutch_force = (m_config.engine_type == 'c' || m_config.engine_type == 'e') ? 5000.0f : 10000.0f;

    m_config.shift_time = std::max(0.0f, m_config.shift_time);
    m_config.post_shift_time = std::max(0.0f, m_config.post_shift_time);
    m_config.clutch_time = Clamp(m_config.clutch_time, 0.0f, 0.9f * m_config.shift_time);

    m_idle_rpm = m_config.idle_rpm > 0.0f
        ? m_config.idle_rpm : std::min(m_config.shift_down_rpm, 800.0f);
    m_config.stall_rpm = Clamp(m_config.stall_rpm, 0.0f, 0.9f * m_idle_rpm);

    if (m_config.max_idle_mixture <= 0.0f) m_config.max_idle_mixture = 0.1f;
    if (m_config.min_idle_mixture < 0.0f) m_config.min_idle_mixture = 0.0f;
    m_braking_torque = m_config.engine_braking_torque > 0.0f
        ? -m_config.engine_braking_torque
        : -m_config.engine_torque / 5.0f;

    Reset();
}

void RoRDrivetrain::Reset()
{
    m_state = RoRDrivetrainTelemetry();
    m_state.gear = m_config.forward_gears.empty() ? 0 : 1;
    m_state.running = true;
    m_state.engine_rpm = m_idle_rpm;
    m_state.drive_ratio = InternalRatio(m_state.gear);
    m_state.clutch = 0.0f;
    m_auto_acc = 0.0f;
    m_cur_acc = 0.0f;
    m_shift_clock = 0.0f;
    m_post_shift_clock = 0.0f;
    m_shift_value = 0;
}

void RoRDrivetrain::Start()
{
    m_state.running = true;
    m_state.engine_rpm = m_idle_rpm;
    if (m_state.gear == 0 && !m_config.forward_gears.empty()) m_state.gear = 1;
    m_state.drive_ratio = InternalRatio(m_state.gear);
}

void RoRDrivetrain::Stop()
{
    m_state.running = false;
    m_state.clutch = 0.0f;
    m_state.clutch_torque = 0.0f;
    m_state.shifting = false;
    m_state.post_shifting = false;
    m_shift_value = 0;
}

float RoRDrivetrain::InternalRatio(int gear) const
{
    if (gear < 1 || static_cast<std::size_t>(gear) > m_config.forward_gears.size()) return 0.0f;
    return m_config.forward_gears[static_cast<std::size_t>(gear - 1)] * m_config.differential_ratio;
}

float RoRDrivetrain::TorqueMultiplier(float rpm) const
{
    const std::vector<RoRTorqueCurveSample>& points = m_config.torque_curve_samples;
    if (points.empty())
        return 1.0f;
    if (points.size() == 1)
        return points.front().torque_multiplier;

    const float min_rpm = points.front().rpm;
    const float max_rpm = points.back().rpm;
    if (min_rpm == max_rpm)
        return points.front().torque_multiplier;

    // RoR::TorqueCurve first converts RPM to a global [0,1] spline parameter,
    // then Ogre::SimpleSpline::interpolate(t) assumes the control points are
    // evenly spaced. Preserve that exact behavior rather than doing RPM-local
    // linear interpolation between samples.
    const float global_t = Clamp((rpm - min_rpm) / (max_rpm - min_rpm), 0.0f, 1.0f);
    const float fseg = global_t * static_cast<float>(points.size() - 1u);
    std::size_t segment = static_cast<std::size_t>(fseg);
    if (segment >= points.size() - 1u)
        return points.back().torque_multiplier;
    const float t = fseg - static_cast<float>(segment);
    if (t == 0.0f) return points[segment].torque_multiplier;
    if (t == 1.0f) return points[segment + 1u].torque_multiplier;

    const auto tangent = [&points](std::size_t i)
    {
        if (i == 0u)
            return 0.5f * (points[1u].torque_multiplier - points[0u].torque_multiplier);
        if (i + 1u == points.size())
            return 0.5f * (points[i].torque_multiplier - points[i - 1u].torque_multiplier);
        return 0.5f * (points[i + 1u].torque_multiplier - points[i - 1u].torque_multiplier);
    };

    // Y component of Ogre's Hermite matrix used by SimpleSpline.
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    const float h01 = -2.0f * t3 + 3.0f * t2;
    const float h10 = t3 - 2.0f * t2 + t;
    const float h11 = t3 - t2;
    return h00 * points[segment].torque_multiplier +
           h01 * points[segment + 1u].torque_multiplier +
           h10 * tangent(segment) +
           h11 * tangent(segment + 1u);
}

float RoRDrivetrain::EnginePower(float rpm) const
{
    return m_config.engine_torque * TorqueMultiplier(rpm);
}

float RoRDrivetrain::IdleMixture() const
{
    return m_state.engine_rpm <= m_idle_rpm
        ? m_config.max_idle_mixture : m_config.min_idle_mixture;
}

void RoRDrivetrain::BeginShift(int delta)
{
    const int top = static_cast<int>(m_config.forward_gears.size());
    if (delta == 0 || m_state.shifting || m_state.post_shifting) return;
    if (m_state.gear + delta < 1 || m_state.gear + delta > top) return;
    m_shift_value = delta;
    m_state.shifting = true;
    m_shift_clock = 0.0f;
}

void RoRDrivetrain::RequestAutomaticShift()
{
    if (m_state.shifting || m_state.post_shifting || m_state.gear <= 0) return;
    const int top = static_cast<int>(m_config.forward_gears.size());

    if (m_state.engine_rpm > m_config.shift_up_rpm - 100.0f &&
        m_state.gear < top && m_state.clutch > 0.99f)
    {
        BeginShift(1);
    }
    else if (m_state.gear > 1 && m_state.engine_rpm < m_config.shift_down_rpm)
    {
        BeginShift(-1);
    }
}

void RoRDrivetrain::UpdateShiftState(float dt)
{
    if (m_state.shifting)
    {
        m_shift_clock += dt;

        if (m_config.shift_time <= 0.0f)
        {
            if (m_shift_value != 0)
            {
                m_state.gear = std::max(1, std::min(
                    static_cast<int>(m_config.forward_gears.size()),
                    m_state.gear + m_shift_value));
                m_shift_value = 0;
            }
            m_cur_acc = m_auto_acc;
            m_state.shifting = false;
            m_state.post_shifting = m_config.post_shift_time > 0.0f;
            m_post_shift_clock = 0.0f;
            return;
        }

        if (m_shift_value != 0)
        {
            const float declutch_time = std::min(
                std::max(0.0f, m_config.shift_time - m_config.clutch_time),
                m_config.clutch_time);
            if (declutch_time > 0.0f && m_shift_clock <= declutch_time)
            {
                const float ratio = std::pow(1.0f - (m_shift_clock / declutch_time), 2.0f);
                m_state.clutch = std::min(ratio, m_state.clutch);
                m_cur_acc = std::min(ratio, m_auto_acc);
            }
            else
            {
                m_state.gear = std::max(1, std::min(
                    static_cast<int>(m_config.forward_gears.size()),
                    m_state.gear + m_shift_value));
                m_shift_value = 0;
            }
        }

        if (m_shift_clock > m_config.shift_time)
        {
            m_cur_acc = m_auto_acc;
            m_state.shifting = false;
            m_state.post_shifting = true;
            m_post_shift_clock = 0.0f;
        }
        else if (m_shift_value == 0 && m_state.gear != 0 &&
                 m_shift_clock >= (m_config.shift_time - m_config.clutch_time))
        {
            if (m_config.clutch_time > 0.0f)
            {
                const float timer = m_shift_clock - (m_config.shift_time - m_config.clutch_time);
                const float ratio = std::sqrt(std::max(0.0f, timer / m_config.clutch_time));
                m_cur_acc = (m_auto_acc / 2.0f) * ratio;
            }
        }
    }

    if (m_state.post_shifting)
    {
        m_post_shift_clock += dt;
        if (m_config.post_shift_time <= 0.0f || m_post_shift_clock > m_config.post_shift_time)
        {
            m_state.post_shifting = false;
            m_cur_acc = m_auto_acc;
        }
        else if (m_auto_acc > 0.0f)
        {
            const float ratio = m_post_shift_clock / m_config.post_shift_time;
            m_cur_acc = (m_auto_acc / 2.0f) + (m_auto_acc / 2.0f) * ratio;
        }
        else if (m_state.gear != 0)
        {
            const float drive_ratio = InternalRatio(m_state.gear);
            if (std::fabs(drive_ratio) > 1.0e-6f)
            {
                const float gearbox_spinner = m_state.engine_rpm / drive_ratio;
                if (m_state.wheel_spin_rpm > gearbox_spinner)
                {
                    const float ratio = std::sqrt(m_post_shift_clock / m_config.post_shift_time);
                    m_state.clutch = std::max(m_state.clutch, ratio);
                }
            }
        }
    }
}

void RoRDrivetrain::UpdateAutoClutch(float acc)
{
    const float ratio = InternalRatio(m_state.gear);
    const float declutch_rpm = m_config.shift_down_rpm * 0.75f + m_config.stall_rpm * 0.25f;

    if (m_state.gear == 0 || m_state.engine_rpm < declutch_rpm)
    {
        m_state.clutch = 0.0f;
    }
    else if (m_state.engine_rpm < m_config.shift_down_rpm && m_config.shift_down_rpm > declutch_rpm)
    {
        const float clutch = (m_state.engine_rpm - declutch_rpm) /
            (m_config.shift_down_rpm - declutch_rpm);
        m_state.clutch = std::min(clutch * clutch, m_state.clutch);
    }
    else if (m_shift_value == 0 && m_state.engine_rpm > m_config.shift_down_rpm &&
             m_state.clutch < 1.0f && std::fabs(ratio) > 1.0e-6f)
    {
        const float first_ratio = std::fabs(InternalRatio(1));
        const float threshold = 1.5f * EnginePower(m_state.engine_rpm) * first_ratio;
        const float gearbox_spinner = m_state.engine_rpm / ratio;
        const float clutch_torque = (gearbox_spinner - m_state.wheel_spin_rpm) * m_config.clutch_force;
        const float reaction_torque = Clamp(clutch_torque, -threshold, threshold) / ratio;

        const float range = (m_config.shift_up_rpm - m_config.shift_down_rpm) *
            0.4f * std::sqrt(std::max(0.2f, acc));
        const float power_ratio = range > 1.0e-6f
            ? std::min((m_state.engine_rpm - m_config.shift_down_rpm) / range, 1.0f)
            : 1.0f;
        const float engine_torque = EnginePower(m_state.engine_rpm) *
            std::min(m_cur_acc, 0.9f) * power_ratio;
        const float torque_diff = std::min(engine_torque, std::fabs(reaction_torque));

        if (std::fabs(reaction_torque) > 1.0e-8f)
        {
            const float clutch = torque_diff / reaction_torque;
            m_state.clutch = std::max(m_state.clutch, clutch);
        }
    }

    m_state.clutch = Clamp(m_state.clutch, 0.0f, 1.0f);
}

void RoRDrivetrain::Step(float throttle, float wheel_spin_rpm, float dt)
{
    if (dt <= 0.0f) return;

    m_auto_acc = Clamp(throttle, 0.0f, 1.0f);
    if (!m_state.shifting && !m_state.post_shifting)
        m_cur_acc = m_auto_acc;
    m_state.wheel_spin_rpm = wheel_spin_rpm;

    if (!m_state.running)
    {
        m_state.clutch_torque = 0.0f;
        m_state.current_acc = m_cur_acc;
        return;
    }

    m_state.drive_ratio = InternalRatio(m_state.gear);
    const float drive_ratio = m_state.drive_ratio;
    const float acc = std::max(IdleMixture(), m_cur_acc);

    float total_torque = m_braking_torque *
        (m_state.engine_rpm / m_config.shift_up_rpm) * (1.0f - m_cur_acc);

    m_state.engine_torque = 0.0f;
    if (m_state.engine_rpm < m_config.shift_up_rpm * 1.25f)
    {
        m_state.engine_torque = EnginePower(m_state.engine_rpm) * acc;
        total_torque += m_state.engine_torque;
    }

    if (m_config.engine_type != 'e' && m_state.engine_rpm < m_config.stall_rpm)
        m_state.running = false;

    if (m_state.gear != 0 && std::fabs(drive_ratio) > 1.0e-6f)
        total_torque -= m_state.clutch_torque / drive_ratio;

    m_state.engine_rpm += dt * total_torque / m_config.engine_inertia;

    if (m_state.gear != 0 && std::fabs(drive_ratio) > 1.0e-6f)
    {
        const float first_ratio = std::fabs(InternalRatio(1));
        const float force_threshold = 1.5f *
            std::max(m_config.engine_torque, EnginePower(m_state.engine_rpm)) * first_ratio;
        const float gearbox_spinner = m_state.engine_rpm / drive_ratio;
        float clutch_torque = (gearbox_spinner - m_state.wheel_spin_rpm) *
            m_state.clutch * m_config.clutch_force;
        clutch_torque = Clamp(clutch_torque, -force_threshold, force_threshold);
        clutch_torque *= 1.0f - approx_exp(-std::fabs(gearbox_spinner - m_state.wheel_spin_rpm));
        m_state.clutch_torque = clutch_torque;
    }
    else
    {
        m_state.clutch_torque = 0.0f;
    }

    m_state.engine_rpm = std::max(0.0f, m_state.engine_rpm);

    UpdateShiftState(dt);
    UpdateAutoClutch(acc);
    RequestAutomaticShift();

    m_state.drive_ratio = InternalRatio(m_state.gear);
    m_state.current_acc = m_cur_acc;
}

float RoRDrivetrain::OutputTorque() const { return m_state.clutch_torque; }
float RoRDrivetrain::DriveRatio() const { return m_state.drive_ratio; }
int RoRDrivetrain::Gear() const { return m_state.gear; }
const RoRDrivetrainTelemetry& RoRDrivetrain::Telemetry() const { return m_state; }

} }
