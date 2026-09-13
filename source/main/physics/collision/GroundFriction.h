/*
    This source file is part of Rigs of Rods

    For more information, see http://www.rigsofrods.org/

    Rigs of Rods is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3, as
    published by the Free Software Foundation.

    Rigs of Rods is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Rigs of Rods. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

namespace RoR {

/// Scalar subset of ground_model_t needed by the tire/ground friction equations.
/// Keeping this free of renderer types lets the existing RoR friction model be
/// compiled and tested as part of the portable vehicle core.
struct GroundFrictionParams
{
    float adhesion_velocity;
    float static_friction;
    float sliding_friction;
    float hydrodynamic_friction;
    float stribeck_velocity;
    float stribeck_alpha;
};

/// Returns the tangential force scale applied to the moderated ground reaction
/// while the contact is in the static-friction regime.
float CalcStaticFrictionScale(float slip_velocity, const GroundFrictionParams& params);

/// Returns the tangential force scale applied to the moderated ground reaction
/// while the contact is in the sliding/Stribeck-friction regime.
float CalcStribeckFrictionScale(float slip_velocity, const GroundFrictionParams& params);

} // namespace RoR
