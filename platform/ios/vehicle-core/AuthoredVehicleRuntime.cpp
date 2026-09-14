/*
    This source file is part of Rigs of Rods.
*/

#include "AuthoredVehicleRuntime.h"

#include "GroundContact.h"
#include "HydroPhysics.h"
#include "PortableRigDef.h"
#include "SimConstants.h"
#include "WheelPhysics.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace RoR {
namespace IOSVehicleCore {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr std::size_t kInvalidIndex = static_cast<std::size_t>(-1);

float Clamp(float value, float low, float high)
{
    return std::max(low, std::min(value, high));
}

float Length(const PhysicsVec3& value)
{
    return std::sqrt(value.squaredLength());
}

float Distance(const PhysicsVec3& a, const PhysicsVec3& b)
{
    return Length(a - b);
}

PhysicsVec3 Normalized(const PhysicsVec3& value)
{
    const float len = Length(value);
    return (len > 1.0e-7f) ? value * (1.0f / len) : PhysicsVec3();
}

bool Finite(const PhysicsVec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

std::string NormalizeLegacyNodeRef(const std::string& reference)
{
    if (reference.size() > 1 && reference[0] == '-')
    {
        bool numeric = true;
        for (std::size_t i = 1; i < reference.size(); ++i)
        {
            if (reference[i] < '0' || reference[i] > '9')
            {
                numeric = false;
                break;
            }
        }
        if (numeric)
            return reference.substr(1);
    }
    return reference;
}

} // namespace

struct AuthoredVehicleRuntime::Impl
{
    enum class BeamKind
    {
        Normal,
        Shock,
        Hydro,
        Wheel
    };

    struct BeamLink
    {
        std::size_t a = 0;
        std::size_t b = 0;
        BeamCoreState beam;
        BeamKind kind = BeamKind::Normal;
    };

    struct HydroLink
    {
        std::size_t beam_index = 0;
        float reference_length = 0.0f;
        float lengthening_factor = 0.0f;
    };

    struct WheelFixture
    {
        std::size_t axis0 = 0;
        std::size_t axis1 = 0;
        std::size_t reference_arm = kInvalidIndex;
        std::size_t near_attach = kInvalidIndex;
        int braking = 0;
        int propulsion = 0;
        std::vector<std::size_t> tire_nodes;
        std::vector<WheelNodeBinding> bindings;
        WheelCoreState wheel;
    };

    std::string source_text;
    PortableRigDef::Document rig;
    std::string vehicle_name;
    std::deque<NodeCoreState> nodes;
    std::vector<BeamLink> beams;
    std::vector<HydroLink> hydros;
    std::vector<WheelFixture> wheels;
    std::vector<std::pair<std::size_t, std::size_t>> beam_pairs;
    std::vector<std::size_t> tire_nodes;
    std::vector<std::size_t> contact_nodes;
    std::unordered_map<std::string, std::size_t> node_indices;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    GroundContactParams road;

    float steering = 0.0f;
    float steering_state = 0.0f;
    float throttle = 0.0f;
    float throttle_state = 0.0f;
    float brake = 0.0f;
    bool handbrake = false;
    bool finite = true;
    std::uint64_t physics_steps = 0;

    explicit Impl(const std::string& text): source_text(text)
    {
        Build();
    }

    void Error(const std::string& text)
    {
        errors.push_back(text);
    }

    void Warn(const std::string& text)
    {
        warnings.push_back(text);
    }

    std::size_t ResolveNode(const std::string& reference, bool allow_legacy_signed = true)
    {
        if (reference.empty() || reference == "9999" || reference == "-1")
            return kInvalidIndex;

        std::string key = reference;
        if (allow_legacy_signed)
            key = NormalizeLegacyNodeRef(key);

        const auto found = node_indices.find(key);
        return found == node_indices.end() ? kInvalidIndex : found->second;
    }

    std::size_t AddNode(const PhysicsVec3& position, float mass)
    {
        nodes.emplace_back();
        NodeCoreState& node = nodes.back();
        node.position = position;
        node.mass = std::max(mass, 0.25f);
        node.force = PhysicsVec3(0.0f, node.mass * DEFAULT_GRAVITY, 0.0f);
        return nodes.size() - 1;
    }

    std::size_t AddBeam(
        std::size_t a,
        std::size_t b,
        float spring,
        float damping,
        BeamKind kind,
        float rest_length = -1.0f)
    {
        if (a == b || a >= nodes.size() || b >= nodes.size())
            return kInvalidIndex;

        const float natural_length = Distance(nodes[a].position, nodes[b].position);
        if (natural_length <= 1.0e-6f)
            return kInvalidIndex;

        BeamLink link;
        link.a = a;
        link.b = b;
        link.beam.rest_length = rest_length > 1.0e-6f ? rest_length : natural_length;
        link.beam.spring = std::max(0.0f, spring);
        link.beam.damping = std::max(0.0f, damping);
        link.kind = kind;
        beams.push_back(link);
        beam_pairs.emplace_back(a, b);
        return beams.size() - 1;
    }

    PhysicsVec3 WheelCenter(const WheelFixture& wheel) const
    {
        return (nodes[wheel.axis0].position + nodes[wheel.axis1].position) * 0.5f;
    }

    PhysicsVec3 Forward() const
    {
        if (wheels.size() >= 4)
        {
            const PhysicsVec3 front = (WheelCenter(wheels[0]) + WheelCenter(wheels[1])) * 0.5f;
            const std::size_t rear0 = wheels.size() - 2;
            const std::size_t rear1 = wheels.size() - 1;
            const PhysicsVec3 rear = (WheelCenter(wheels[rear0]) + WheelCenter(wheels[rear1])) * 0.5f;
            PhysicsVec3 direction = front - rear;
            direction.y = 0.0f;
            const PhysicsVec3 normalized = Normalized(direction);
            if (normalized.squaredLength() > 0.5f)
                return normalized;
        }

        if (nodes.size() >= 2)
        {
            PhysicsVec3 direction = nodes[0].position - nodes.back().position;
            direction.y = 0.0f;
            return Normalized(direction);
        }
        return PhysicsVec3(1.0f, 0.0f, 0.0f);
    }

    PhysicsVec3 Center() const
    {
        PhysicsVec3 weighted;
        float mass = 0.0f;
        for (const NodeCoreState& node : nodes)
        {
            weighted += node.position * node.mass;
            mass += node.mass;
        }
        return mass > 0.0f ? weighted * (1.0f / mass) : PhysicsVec3();
    }

    PhysicsVec3 CenterVelocity() const
    {
        PhysicsVec3 weighted;
        float mass = 0.0f;
        for (const NodeCoreState& node : nodes)
        {
            weighted += node.velocity * node.mass;
            mass += node.mass;
        }
        return mass > 0.0f ? weighted * (1.0f / mass) : PhysicsVec3();
    }

    float Heading() const
    {
        const PhysicsVec3 forward = Forward();
        return std::atan2(forward.z, forward.x);
    }

    void BuildPlainWheel(const PortableRigDef::Wheel& def, WheelFixture& fixture)
    {
        const int rays = std::max(3, def.num_rays);
        const PhysicsVec3 axis = Normalized(nodes[fixture.axis1].position - nodes[fixture.axis0].position);
        if (axis.squaredLength() < 0.5f)
        {
            Error("wheel has coincident axis nodes");
            return;
        }

        PhysicsVec3 radial_up = PhysicsVec3(0.0f, 1.0f, 0.0f) - axis * axis.y;
        if (radial_up.squaredLength() < 1.0e-5f)
            radial_up = PhysicsVec3(1.0f, 0.0f, 0.0f) - axis * axis.x;
        radial_up = Normalized(radial_up);
        const PhysicsVec3 radial_forward = Normalized(axis.cross(radial_up));
        const float node_mass = std::max(0.5f, def.mass / static_cast<float>(rays * 2));
        const float spring = def.tire_spring > 0.0f ? def.tire_spring : 200000.0f;
        const float damping = def.tire_damping > 0.0f ? def.tire_damping : 2500.0f;

        fixture.tire_nodes.reserve(static_cast<std::size_t>(rays * 2));
        fixture.bindings.reserve(static_cast<std::size_t>(rays * 2));

        for (int ray = 0; ray < rays; ++ray)
        {
            const float angle = 2.0f * kPi * static_cast<float>(ray) / static_cast<float>(rays);
            const PhysicsVec3 radial =
                radial_up * (def.tire_radius * std::cos(angle)) +
                radial_forward * (def.tire_radius * std::sin(angle));
            const std::size_t sides[2] = {fixture.axis0, fixture.axis1};
            for (int side = 0; side < 2; ++side)
            {
                const std::size_t outer = AddNode(nodes[sides[side]].position + radial, node_mass);
                fixture.tire_nodes.push_back(outer);
                tire_nodes.push_back(outer);

                WheelNodeBinding binding;
                binding.outer = &nodes[outer];
                binding.inner = &nodes[sides[side]];
                fixture.bindings.push_back(binding);

                // RoR's generated plain wheel is located from both axle nodes.
                AddBeam(outer, fixture.axis0, spring, damping, BeamKind::Wheel);
                AddBeam(outer, fixture.axis1, spring, damping, BeamKind::Wheel);
            }
        }

        for (int ray = 0; ray < rays; ++ray)
        {
            const int next = (ray + 1) % rays;
            const std::size_t side0 = fixture.tire_nodes[static_cast<std::size_t>(ray * 2)];
            const std::size_t side1 = fixture.tire_nodes[static_cast<std::size_t>(ray * 2 + 1)];
            const std::size_t next0 = fixture.tire_nodes[static_cast<std::size_t>(next * 2)];
            const std::size_t next1 = fixture.tire_nodes[static_cast<std::size_t>(next * 2 + 1)];
            AddBeam(side0, next0, spring, damping, BeamKind::Wheel);
            AddBeam(side1, next1, spring, damping, BeamKind::Wheel);
            AddBeam(side0, side1, spring, damping, BeamKind::Wheel);
            AddBeam(side0, next1, spring, damping, BeamKind::Wheel);
        }

        const std::string normalized_rigidity = NormalizeLegacyNodeRef(def.rigidity_node);
        if (normalized_rigidity != def.rigidity_node && def.rigidity_node != "-1")
        {
            Warn("normalized legacy signed wheel rigidity-node reference " +
                 def.rigidity_node + " -> " + normalized_rigidity);
        }
        const std::size_t rigidity = ResolveNode(def.rigidity_node);
        if (rigidity != kInvalidIndex)
        {
            const float d0 = Distance(nodes[rigidity].position, nodes[fixture.axis0].position);
            const float d1 = Distance(nodes[rigidity].position, nodes[fixture.axis1].position);
            const int side = d0 < d1 ? 0 : 1;
            for (int ray = 0; ray < rays; ++ray)
            {
                AddBeam(
                    rigidity,
                    fixture.tire_nodes[static_cast<std::size_t>(ray * 2 + side)],
                    spring,
                    damping,
                    BeamKind::Wheel);
            }
        }
    }

    void BuildWheel2(const PortableRigDef::Wheel& def, WheelFixture& fixture)
    {
        const int rays = std::max(3, def.num_rays);
        const PhysicsVec3 axis = Normalized(nodes[fixture.axis1].position - nodes[fixture.axis0].position);
        if (axis.squaredLength() < 0.5f)
        {
            Error("wheels2 has coincident axis nodes");
            return;
        }

        PhysicsVec3 radial_up = PhysicsVec3(0.0f, 1.0f, 0.0f) - axis * axis.y;
        if (radial_up.squaredLength() < 1.0e-5f)
            radial_up = PhysicsVec3(1.0f, 0.0f, 0.0f) - axis * axis.x;
        radial_up = Normalized(radial_up);
        const PhysicsVec3 radial_forward = Normalized(axis.cross(radial_up));
        const float node_mass = std::max(0.35f, def.mass / static_cast<float>(rays * 4));
        const float rim_spring = def.rim_spring > 0.0f ? def.rim_spring : 300000.0f;
        const float rim_damping = def.rim_damping > 0.0f ? def.rim_damping : 1800.0f;
        const float tire_spring = def.tire_spring > 0.0f ? def.tire_spring : 180000.0f;
        const float tire_damping = def.tire_damping > 0.0f ? def.tire_damping : 1800.0f;

        std::vector<std::size_t> rim_nodes;
        rim_nodes.reserve(static_cast<std::size_t>(rays * 2));
        fixture.tire_nodes.reserve(static_cast<std::size_t>(rays * 2));
        fixture.bindings.reserve(static_cast<std::size_t>(rays * 2));

        for (int ray = 0; ray < rays; ++ray)
        {
            const float angle = 2.0f * kPi * static_cast<float>(ray) / static_cast<float>(rays);
            const PhysicsVec3 unit = radial_up * std::cos(angle) + radial_forward * std::sin(angle);
            const std::size_t sides[2] = {fixture.axis0, fixture.axis1};
            for (int side = 0; side < 2; ++side)
            {
                const std::size_t rim = AddNode(nodes[sides[side]].position + unit * def.rim_radius, node_mass);
                const std::size_t tire = AddNode(nodes[sides[side]].position + unit * def.tire_radius, node_mass);
                rim_nodes.push_back(rim);
                fixture.tire_nodes.push_back(tire);
                tire_nodes.push_back(tire);

                WheelNodeBinding binding;
                binding.outer = &nodes[tire];
                binding.inner = &nodes[sides[side]];
                fixture.bindings.push_back(binding);

                AddBeam(rim, fixture.axis0, rim_spring, rim_damping, BeamKind::Wheel);
                AddBeam(rim, fixture.axis1, rim_spring, rim_damping, BeamKind::Wheel);
                AddBeam(tire, rim, tire_spring, tire_damping, BeamKind::Wheel);
                AddBeam(tire, fixture.axis0, tire_spring, tire_damping, BeamKind::Wheel);
                AddBeam(tire, fixture.axis1, tire_spring, tire_damping, BeamKind::Wheel);
            }
        }

        for (int ray = 0; ray < rays; ++ray)
        {
            const int next = (ray + 1) % rays;
            const std::size_t r0 = rim_nodes[static_cast<std::size_t>(ray * 2)];
            const std::size_t r1 = rim_nodes[static_cast<std::size_t>(ray * 2 + 1)];
            const std::size_t rn0 = rim_nodes[static_cast<std::size_t>(next * 2)];
            const std::size_t rn1 = rim_nodes[static_cast<std::size_t>(next * 2 + 1)];
            const std::size_t t0 = fixture.tire_nodes[static_cast<std::size_t>(ray * 2)];
            const std::size_t t1 = fixture.tire_nodes[static_cast<std::size_t>(ray * 2 + 1)];
            const std::size_t tn0 = fixture.tire_nodes[static_cast<std::size_t>(next * 2)];
            const std::size_t tn1 = fixture.tire_nodes[static_cast<std::size_t>(next * 2 + 1)];

            AddBeam(r0, rn0, rim_spring, rim_damping, BeamKind::Wheel);
            AddBeam(r1, rn1, rim_spring, rim_damping, BeamKind::Wheel);
            AddBeam(r0, r1, rim_spring, rim_damping, BeamKind::Wheel);
            AddBeam(r0, rn1, rim_spring, rim_damping, BeamKind::Wheel);
            AddBeam(r1, rn0, rim_spring, rim_damping, BeamKind::Wheel);

            AddBeam(t0, tn0, tire_spring, tire_damping, BeamKind::Wheel);
            AddBeam(t1, tn1, tire_spring, tire_damping, BeamKind::Wheel);
            AddBeam(t0, t1, tire_spring, tire_damping, BeamKind::Wheel);
            AddBeam(t0, tn1, tire_spring, tire_damping, BeamKind::Wheel);
            AddBeam(t1, tn0, tire_spring, tire_damping, BeamKind::Wheel);
            AddBeam(t0, rn0, tire_spring, tire_damping, BeamKind::Wheel);
            AddBeam(t1, rn1, tire_spring, tire_damping, BeamKind::Wheel);
        }

        const std::size_t rigidity = ResolveNode(def.rigidity_node);
        if (rigidity != kInvalidIndex)
        {
            const float d0 = Distance(nodes[rigidity].position, nodes[fixture.axis0].position);
            const float d1 = Distance(nodes[rigidity].position, nodes[fixture.axis1].position);
            const int side = d0 < d1 ? 0 : 1;
            for (int ray = 0; ray < rays; ++ray)
            {
                AddBeam(
                    rigidity,
                    fixture.tire_nodes[static_cast<std::size_t>(ray * 2 + side)],
                    tire_spring,
                    tire_damping,
                    BeamKind::Wheel);
            }
        }
    }

    void BuildWheel(const PortableRigDef::Wheel& def)
    {
        const std::size_t axis0 = ResolveNode(def.axis_node_0, false);
        const std::size_t axis1 = ResolveNode(def.axis_node_1, false);
        if (axis0 == kInvalidIndex || axis1 == kInvalidIndex)
        {
            Error("wheel references missing axle nodes " + def.axis_node_0 + "/" + def.axis_node_1);
            return;
        }
        if (def.tire_radius <= 0.0f || def.num_rays < 3)
        {
            Error("wheel has invalid radius or ray count");
            return;
        }

        WheelFixture fixture;
        fixture.axis0 = axis0;
        fixture.axis1 = axis1;
        fixture.braking = def.braking;
        fixture.propulsion = def.propulsion;
        fixture.reference_arm = ResolveNode(def.reference_arm_node);
        fixture.wheel.radius = def.tire_radius;
        fixture.wheel.rotational_mass = std::max(1.0f, def.mass);

        if (def.wheels2)
            BuildWheel2(def, fixture);
        else
            BuildPlainWheel(def, fixture);

        if (fixture.reference_arm != kInvalidIndex)
        {
            const float d0 = Distance(nodes[fixture.reference_arm].position, nodes[axis0].position);
            const float d1 = Distance(nodes[fixture.reference_arm].position, nodes[axis1].position);
            fixture.near_attach = d0 < d1 ? axis0 : axis1;
        }
        wheels.push_back(fixture);
    }

    void AlignTiresToGround()
    {
        if (tire_nodes.empty())
            return;

        float minimum_y = std::numeric_limits<float>::max();
        for (std::size_t index : tire_nodes)
            minimum_y = std::min(minimum_y, nodes[index].position.y);

        if (!std::isfinite(minimum_y))
            return;
        const float offset = -minimum_y + 0.002f;
        for (NodeCoreState& node : nodes)
            node.position.y += offset;
    }

    void ConfigureWheelDirections()
    {
        const PhysicsVec3 forward = Forward();
        for (WheelFixture& fixture : wheels)
        {
            const PhysicsVec3 axis = Normalized(nodes[fixture.axis1].position - nodes[fixture.axis0].position);
            PhysicsVec3 down(0.0f, -1.0f, 0.0f);
            down -= axis * down.dot(axis);
            down = Normalized(down);
            const PhysicsVec3 positive_tread = axis.cross(down);
            fixture.wheel.reverse_rotation = positive_tread.dot(forward) < 0.0f;
        }
    }

    void Build()
    {
        rig = PortableRigDef::Parse(source_text);
        vehicle_name = rig.name.empty() ? "Unnamed RoR vehicle" : rig.name;
        nodes.clear();
        beams.clear();
        hydros.clear();
        wheels.clear();
        beam_pairs.clear();
        tire_nodes.clear();
        contact_nodes.clear();
        node_indices.clear();
        errors.clear();
        warnings = rig.warnings;
        steering = steering_state = 0.0f;
        throttle = throttle_state = brake = 0.0f;
        handbrake = false;
        finite = true;
        physics_steps = 0;

        if (rig.nodes.empty())
        {
            Error("authored vehicle contains no parsed nodes");
            return;
        }

        float dry_mass = rig.globals.present ? rig.globals.dry_mass : 0.0f;
        if (dry_mass <= 0.0f)
        {
            dry_mass = static_cast<float>(rig.nodes.size()) * 50.0f;
            Warn("vehicle has no usable globals dry mass; using portable fallback mass");
        }
        const float authored_node_mass = std::max(1.0f, dry_mass / static_cast<float>(rig.nodes.size()));

        for (const PortableRigDef::Node& source : rig.nodes)
        {
            if (source.id.empty() || node_indices.find(source.id) != node_indices.end())
            {
                Error("duplicate or empty authored node id: " + source.id);
                continue;
            }
            const std::size_t index = AddNode(PhysicsVec3(source.x, source.y, source.z), authored_node_mass);
            node_indices[source.id] = index;
        }

        for (const PortableRigDef::Beam& source : rig.beams)
        {
            const std::size_t a = ResolveNode(source.node_a, false);
            const std::size_t b = ResolveNode(source.node_b, false);
            if (a == kInvalidIndex || b == kInvalidIndex)
            {
                Error("authored beam references missing node " + source.node_a + " -> " + source.node_b);
                continue;
            }
            AddBeam(a, b, source.spring, source.damping, BeamKind::Normal);
        }

        for (const PortableRigDef::Shock& source : rig.shocks)
        {
            const std::size_t a = ResolveNode(source.node_a, false);
            const std::size_t b = ResolveNode(source.node_b, false);
            if (a == kInvalidIndex || b == kInvalidIndex)
            {
                Error("shock references missing node");
                continue;
            }
            const float base = Distance(nodes[a].position, nodes[b].position);
            const float rest = base * (source.precompression > 0.0f ? source.precompression : 1.0f);
            AddBeam(a, b, source.spring, source.damping, BeamKind::Shock, rest);
        }

        for (const PortableRigDef::Hydro& source : rig.hydros)
        {
            const std::size_t a = ResolveNode(source.node_a, false);
            const std::size_t b = ResolveNode(source.node_b, false);
            if (a == kInvalidIndex || b == kInvalidIndex)
            {
                Error("hydro references missing node");
                continue;
            }
            const float reference = Distance(nodes[a].position, nodes[b].position);
            const std::size_t beam_index = AddBeam(a, b, source.spring, source.damping, BeamKind::Hydro, reference);
            if (beam_index != kInvalidIndex)
            {
                HydroLink hydro;
                hydro.beam_index = beam_index;
                hydro.reference_length = reference;
                hydro.lengthening_factor = source.lengthening_factor;
                hydros.push_back(hydro);
            }
        }

        for (const PortableRigDef::Wheel& source : rig.wheels)
            BuildWheel(source);

        if (wheels.empty())
            Error("authored vehicle has no supported wheel section");

        for (const std::string& reference : rig.contacters)
        {
            const std::size_t index = ResolveNode(reference, false);
            if (index != kInvalidIndex)
                contact_nodes.push_back(index);
        }

        AlignTiresToGround();
        ConfigureWheelDirections();

        road = GroundContactParams();
        road.friction.adhesion_velocity = 0.55f;
        road.friction.static_friction = 1.12f;
        road.friction.sliding_friction = 0.76f;
        road.friction.hydrodynamic_friction = 0.018f;
        road.friction.stribeck_velocity = 0.35f;
        road.friction.stribeck_alpha = 2.0f;
        road.ground_strength = 1.0f;
        road.node_friction = 1.0f;
        road.solid_ground_level = 0.0f;

        if (!rig.globals.present)
            Warn("portable authored runtime is using fallback mass distribution");
        else
            Warn("portable authored runtime currently distributes globals dry mass uniformly across authored nodes");
    }

    void StepInternal(float dt)
    {
        if (!finite || !errors.empty() || dt <= 0.0f)
            return;

        // Ground contact for generated tire nodes plus explicitly authored contacters.
        for (std::size_t index : tire_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);
        for (std::size_t index : contact_nodes)
            ApplyFlatGroundContact(nodes[index], 0.0f, road, dt);

        const PhysicsVec3 center_velocity = CenterVelocity();
        steering_state = StepHydroSteeringState(
            steering_state,
            steering,
            Length(center_velocity),
            false,
            1.0f,
            1.0f,
            dt);
        for (HydroLink& hydro : hydros)
        {
            if (hydro.beam_index >= beams.size())
                continue;
            beams[hydro.beam_index].beam.rest_length = CalcHydroTargetLength(
                hydro.reference_length,
                steering_state,
                hydro.lengthening_factor,
                0.30f,
                0.30f);
        }

        const float throttle_rate = 1.8f * dt;
        if (throttle_state < throttle)
            throttle_state = std::min(throttle, throttle_state + throttle_rate);
        else
            throttle_state = std::max(throttle, throttle_state - throttle_rate * 2.2f);

        std::size_t driven_count = 0;
        std::size_t braked_count = 0;
        for (const WheelFixture& fixture : wheels)
        {
            if (fixture.propulsion != 0) ++driven_count;
            if (fixture.braking != 0) ++braked_count;
        }

        const float authored_engine_torque = rig.engine.present ? std::fabs(rig.engine.torque) : 1200.0f;
        const float total_drive_torque = std::min(authored_engine_torque, 8000.0f) * throttle_state * 0.55f;
        const float per_driven_torque = driven_count > 0
            ? total_drive_torque / static_cast<float>(driven_count)
            : 0.0f;
        const float service_force = rig.brakes.present ? std::max(0.0f, rig.brakes.service_force) : 30000.0f;

        for (WheelFixture& fixture : wheels)
        {
            if (fixture.propulsion != 0)
                fixture.wheel.torque += per_driven_torque;

            float available_brake_torque = 0.0f;
            if (fixture.braking != 0 && braked_count > 0)
            {
                available_brake_torque += brake * service_force * fixture.wheel.radius /
                    static_cast<float>(braked_count);
            }
            if (handbrake && fixture.propulsion != 0)
            {
                const float parking_force = (rig.brakes.present && rig.brakes.parking_force > 0.0f)
                    ? rig.brakes.parking_force
                    : service_force * 2.0f;
                available_brake_torque = std::max(
                    available_brake_torque,
                    parking_force * fixture.wheel.radius / std::max<std::size_t>(1, driven_count));
            }
            if (available_brake_torque > 0.0f)
            {
                fixture.wheel.torque += CalcWheelBrakeTorque(
                    fixture.wheel.speed,
                    fixture.wheel.average_speed,
                    fixture.wheel.radius,
                    fixture.wheel.rotational_mass,
                    fixture.wheel.last_reaction_torque,
                    available_brake_torque,
                    dt);
            }

            const PhysicsVec3 axis = nodes[fixture.axis1].position - nodes[fixture.axis0].position;
            const WheelStepResult result = StepWheelNodes(
                fixture.wheel,
                nodes[fixture.axis0],
                nodes[fixture.axis1],
                fixture.bindings,
                dt);

            if (fixture.propulsion != 0 && fixture.reference_arm != kInvalidIndex && fixture.near_attach != kInvalidIndex)
            {
                ApplyWheelReactionTorque(
                    result.applied_torque,
                    axis,
                    nodes[fixture.reference_arm],
                    nodes[fixture.near_attach]);
            }
        }

        for (BeamLink& link : beams)
        {
            ApplyBeamForce(nodes[link.a], nodes[link.b], link.beam);
            if (!std::isfinite(link.beam.stress))
            {
                finite = false;
                return;
            }
        }

        for (NodeCoreState& node : nodes)
        {
            IntegrateNode(node, DEFAULT_GRAVITY, dt);
            if (!Finite(node.position) || !Finite(node.velocity) || node.velocity.squaredLength() > 1.0e12f)
            {
                finite = false;
                return;
            }
        }
        ++physics_steps;
    }
};

AuthoredVehicleRuntime::AuthoredVehicleRuntime(const std::string& truck_text):
    m_impl(new Impl(truck_text))
{
}

AuthoredVehicleRuntime::~AuthoredVehicleRuntime() = default;

void AuthoredVehicleRuntime::Reset()
{
    m_impl->Build();
}

void AuthoredVehicleRuntime::SetControls(float steering, float throttle, float brake, bool handbrake)
{
    m_impl->steering = Clamp(steering, -1.0f, 1.0f);
    m_impl->throttle = Clamp(throttle, 0.0f, 1.0f);
    m_impl->brake = Clamp(brake, 0.0f, 1.0f);
    m_impl->handbrake = handbrake;
}

void AuthoredVehicleRuntime::Step(float dt)
{
    m_impl->StepInternal(dt);
}

bool AuthoredVehicleRuntime::Ready() const
{
    return m_impl->errors.empty() && !m_impl->nodes.empty() && !m_impl->wheels.empty();
}

bool AuthoredVehicleRuntime::IsFinite() const
{
    return m_impl->finite;
}

const std::string& AuthoredVehicleRuntime::VehicleName() const
{
    return m_impl->vehicle_name;
}

const std::vector<std::string>& AuthoredVehicleRuntime::Errors() const
{
    return m_impl->errors;
}

const std::vector<std::string>& AuthoredVehicleRuntime::Warnings() const
{
    return m_impl->warnings;
}

std::size_t AuthoredVehicleRuntime::NodeCount() const
{
    return m_impl->nodes.size();
}

const NodeCoreState& AuthoredVehicleRuntime::Node(std::size_t index) const
{
    static const NodeCoreState empty;
    return index < m_impl->nodes.size() ? m_impl->nodes[index] : empty;
}

const std::vector<std::pair<std::size_t, std::size_t>>& AuthoredVehicleRuntime::BeamPairs() const
{
    return m_impl->beam_pairs;
}

const std::vector<std::size_t>& AuthoredVehicleRuntime::TireNodeIndices() const
{
    return m_impl->tire_nodes;
}

AuthoredVehicleTelemetry AuthoredVehicleRuntime::Telemetry() const
{
    AuthoredVehicleTelemetry telemetry;
    telemetry.center = m_impl->Center();
    telemetry.heading_radians = m_impl->Heading();
    const PhysicsVec3 velocity = m_impl->CenterVelocity();
    telemetry.speed_mps = Length(velocity);
    telemetry.forward_speed_mps = velocity.dot(m_impl->Forward());
    telemetry.steering = m_impl->steering_state;
    telemetry.throttle = m_impl->throttle_state;
    telemetry.handbrake = m_impl->handbrake;
    telemetry.physics_steps = m_impl->physics_steps;

    float driven_speed = 0.0f;
    std::size_t driven_count = 0;
    for (const Impl::WheelFixture& fixture : m_impl->wheels)
    {
        if (fixture.propulsion != 0)
        {
            driven_speed += std::fabs(fixture.wheel.speed);
            ++driven_count;
        }
    }
    telemetry.driven_wheel_speed_mps = driven_count > 0
        ? driven_speed / static_cast<float>(driven_count)
        : 0.0f;
    return telemetry;
}

} // namespace IOSVehicleCore
} // namespace RoR
