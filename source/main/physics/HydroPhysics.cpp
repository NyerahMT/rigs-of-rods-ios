/*
    This source file is part of Rigs of Rods.

    Portable steering-hydro helpers extracted from Actor::CalcHydros().
*/

#include "HydroPhysics.h"

#include <algorithm>
#include <cmath>

namespace RoR {
namespace
{
float Clamp(float value, float low, float high)
{
    return std::max(low, std::min(value, high));
}
}

float StepHydroSteeringState(
    float state,
    float command,
    float wheel_speed,
    bool speed_coupling,
    float analog_smoothing,
    float analog_sensitivity,
    float dt)
{
    command = Clamp(command, -1.0f, 1.0f);
    state = Clamp(state, -1.0f, 1.0f);
    if (dt <= 0.0f)
    {
        return state;
    }

    if (!speed_coupling)
    {
        // Same analog steering smoothing used by Actor::CalcHydros().
        const float smoothing = Clamp(analog_smoothing, 0.5f, 2.0f);
        const float sensitivity = Clamp(analog_sensitivity, 0.5f, 2.0f);
        const float diff = command - state;
        const float rate = std::exp(-std::min(std::fabs(diff), 1.0f) / sensitivity) * diff;
        state += (10.0f / smoothing) * dt * rate;
    }
    else
    {
        // RoR's speed-coupled steering rate: steering slows down as road speed rises,
        // but never below the 20% minimum represented by 1.2.
        const float rate = std::max(1.2f, 30.0f / (10.0f + std::fabs(wheel_speed / 2.0f)));
        const float step = dt * rate;
        if (state > command)
        {
            state = std::max(command, state - step);
        }
        else if (state < command)
        {
            state = std::min(command, state + step);
        }
    }

    return Clamp(state, -1.0f, 1.0f);
}

float CalcHydroTargetLength(
    float reference_length,
    float steering_state,
    float lengthening_factor,
    float short_bound,
    float long_bound)
{
    if (reference_length <= 0.0f)
    {
        return 0.0f;
    }

    float factor = 1.0f - steering_state * lengthening_factor;
    factor = std::max(1.0f - std::max(short_bound, 0.0f), factor);
    factor = std::min(1.0f + std::max(long_bound, 0.0f), factor);
    return reference_length * factor;
}

} // namespace RoR
