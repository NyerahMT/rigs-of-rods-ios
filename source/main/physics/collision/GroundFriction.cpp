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

#include "GroundFriction.h"
#include "ApproxMath.h"

#include <algorithm>

using namespace RoR;

float RoR::CalcStaticFrictionScale(float slip_velocity, const GroundFrictionParams& params)
{
    return -params.static_friction *
        (1.0f - approx_exp(-slip_velocity / params.adhesion_velocity));
}

float RoR::CalcStribeckFrictionScale(float slip_velocity, const GroundFrictionParams& params)
{
    const float g = params.sliding_friction +
        (params.static_friction - params.sliding_friction) *
        approx_exp(-approx_pow(slip_velocity / params.stribeck_velocity, params.stribeck_alpha));

    return -(g + std::min(params.hydrodynamic_friction * slip_velocity, 5.0f));
}
