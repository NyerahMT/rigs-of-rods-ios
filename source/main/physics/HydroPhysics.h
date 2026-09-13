/*
    This source file is part of Rigs of Rods.

    Portable steering-hydro helpers extracted from Actor::CalcHydros().
*/

#pragma once

namespace RoR {

/// Advances RoR's steering hydro state toward the requested command.
/// command/state are normalized to [-1, 1].
float StepHydroSteeringState(
    float state,
    float command,
    float wheel_speed,
    bool speed_coupling,
    float analog_smoothing,
    float analog_sensitivity,
    float dt);

/// Returns the target beam length used by a steering hydro.
/// This follows CalcHydros(): ref_length * (1 - state * lengthening_factor),
/// with the beam short/long limits applied as multiplicative bounds.
float CalcHydroTargetLength(
    float reference_length,
    float steering_state,
    float lengthening_factor,
    float short_bound,
    float long_bound);

} // namespace RoR
