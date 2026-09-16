#!/usr/bin/env python3
"""Add a coherent 60 FPS render bridge to the generated iOS game shell.

Physics remains authoritative at 2 kHz.  The renderer consumes a deliberately
smaller 120 Hz snapshot stream, renders one display interval behind it, and
interpolates every vehicle visual from the same pair of states.  This removes
thousands of redundant full-node copies per second while preventing body,
wheel, prop and camera phase mismatch.
"""

from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_render_interpolation.py <generated-OgreGameApp.mm>")

p = Path(sys.argv[1])
s = p.read_text()


def one(old: str, new: str, label: str) -> None:
    global s
    if new in s:
        return
    count = s.count(old)
    if count != 1:
        raise SystemExit(f"render optimization anchor '{label}' expected once, found {count}")
    s = s.replace(old, new, 1)


one(
    '#include <chrono>\n',
    '#include <chrono>\n#include <deque>\n',
    'deque include')

one(
    'constexpr double kMaxSimCatchup = 0.05;\n',
    'constexpr double kMaxSimCatchup = 0.05;\n'
    'constexpr double kSnapshotPublishHz = 120.0;\n'
    'constexpr double kSnapshotPublishInterval = 1.0 / kSnapshotPublishHz;\n'
    'constexpr double kRenderInterpolationDelay = 1.0 / static_cast<double>(kRenderFps);\n'
    'constexpr double kRenderSnapshotHistory = 0.05;\n',
    'render timing constants')

one(
    '    RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;\n};\n\nclass SimulationHost\n',
    '''    RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry;
    double published_at = 0.0;
};

double SnapshotClockSeconds()
{
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

RoR::PhysicsVec3 InterpolatePhysicsVec3(
    const RoR::PhysicsVec3& a, const RoR::PhysicsVec3& b, float alpha)
{
    return a * (1.0f - alpha) + b * alpha;
}

Snapshot InterpolateSnapshot(const Snapshot& a, const Snapshot& b, double alpha_double)
{
    if (a.nodes.size() != b.nodes.size() || a.nodes.empty()) return b;
    const float alpha = static_cast<float>(std::max(0.0, std::min(1.0, alpha_double)));
    Snapshot out = b; // discrete state (gear/errors/step number) comes from the newer sample.
    out.nodes.resize(b.nodes.size());
    for (std::size_t i = 0; i < b.nodes.size(); ++i)
        out.nodes[i] = InterpolatePhysicsVec3(a.nodes[i], b.nodes[i], alpha);

    auto& t = out.telemetry;
    const auto& ta = a.telemetry;
    const auto& tb = b.telemetry;
    t.center = InterpolatePhysicsVec3(ta.center, tb.center, alpha);
    t.heading_radians = WrapAngle(
        ta.heading_radians + WrapAngle(tb.heading_radians - ta.heading_radians) * alpha);
    t.speed_mps = ta.speed_mps + (tb.speed_mps - ta.speed_mps) * alpha;
    t.forward_speed_mps = ta.forward_speed_mps + (tb.forward_speed_mps - ta.forward_speed_mps) * alpha;
    t.driven_wheel_speed_mps = ta.driven_wheel_speed_mps +
        (tb.driven_wheel_speed_mps - ta.driven_wheel_speed_mps) * alpha;
    t.engine_rpm = ta.engine_rpm + (tb.engine_rpm - ta.engine_rpm) * alpha;
    t.steering = ta.steering + (tb.steering - ta.steering) * alpha;
    t.throttle = ta.throttle + (tb.throttle - ta.throttle) * alpha;
    out.published_at = a.published_at + (b.published_at - a.published_at) * alpha;
    return out;
}

class SimulationHost
''',
    'snapshot interpolation helpers')

one(
'''    Snapshot GetSnapshot() const
    {
        std::lock_guard<std::mutex> guard(snapshot_mutex);
        return snapshot;
    }
''',
'''    Snapshot GetSnapshot() const
    {
        std::lock_guard<std::mutex> guard(snapshot_mutex);
        if (snapshot_history.size() < 2) return snapshot;

        // Rendering one display interval behind the simulation gives the 60 Hz
        // display two real 120 Hz physics snapshots to blend between.  No
        // extrapolation and no independent body/wheel smoothing are required.
        const double target = SnapshotClockSeconds() - kRenderInterpolationDelay;
        if (target <= snapshot_history.front().published_at)
            return snapshot_history.front();

        for (std::size_t i = 1; i < snapshot_history.size(); ++i)
        {
            const Snapshot& newer = snapshot_history[i];
            if (newer.published_at < target) continue;
            const Snapshot& older = snapshot_history[i - 1];
            const double span = newer.published_at - older.published_at;
            if (span <= 1.0e-9) return newer;
            return InterpolateSnapshot(older, newer, (target - older.published_at) / span);
        }
        return snapshot;
    }
''',
    'render snapshot getter')

one(
'''        s.nodes.reserve(runtime.NodeCount());
        for (std::size_t i = 0; i < runtime.NodeCount(); ++i) s.nodes.push_back(runtime.Node(i).position);
        std::lock_guard<std::mutex> guard(snapshot_mutex);
        snapshot = std::move(s);
''',
'''        s.nodes.reserve(runtime.NodeCount());
        for (std::size_t i = 0; i < runtime.NodeCount(); ++i) s.nodes.push_back(runtime.Node(i).position);
        s.published_at = SnapshotClockSeconds();
        std::lock_guard<std::mutex> guard(snapshot_mutex);

        // Reset() restarts the physics step counter. Never interpolate across
        // that discontinuity or a respawn would smear the whole vehicle.
        if (!snapshot_history.empty() &&
            s.telemetry.physics_steps <= snapshot_history.back().telemetry.physics_steps)
            snapshot_history.clear();

        snapshot_history.push_back(s);
        const double cutoff = s.published_at - kRenderSnapshotHistory;
        while (snapshot_history.size() > 2 && snapshot_history[1].published_at < cutoff)
            snapshot_history.pop_front();
        snapshot = std::move(s);
''',
    'snapshot history publication')

one(
'''        auto previous = Clock::now();
        double accumulator = 0.0;
        while (running.load())
''',
'''        auto previous = Clock::now();
        double accumulator = 0.0;
        double last_publish_at = SnapshotClockSeconds();
        while (running.load())
''',
    'snapshot rate limiter state')

one(
'''            if (steps == kMaxSimStepsPerBatch) accumulator = 0.0;
            if (steps > 0) Publish();
            std::this_thread::sleep_for(std::chrono::microseconds(500));
''',
'''            if (steps == kMaxSimStepsPerBatch) accumulator = 0.0;
            if (steps > 0)
            {
                const double publish_now = SnapshotClockSeconds();
                if (publish_now - last_publish_at >= kSnapshotPublishInterval)
                {
                    Publish();
                    last_publish_at = publish_now;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
''',
    'snapshot publication cap')

one(
'''    mutable std::mutex snapshot_mutex;
    Snapshot snapshot;
};
''',
'''    mutable std::mutex snapshot_mutex;
    Snapshot snapshot;
    std::deque<Snapshot> snapshot_history;
};
''',
    'snapshot history storage')

# prepare_audio_game_source.py adds the production flexbody bridge.  Reuse one
# conversion buffer instead of allocating/freeing a node vector every frame.
one(
'''    void UpdateFlexBodies(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        EnsureFlexBodies(s, visual);
        if (flexbody_instances.empty()) return;

        std::vector<Ogre::Vector3> nodes;
        nodes.reserve(s.nodes.size());
        for (const RoR::PhysicsVec3& node : s.nodes) nodes.push_back(OgreVec(node));
        for (auto& instance : flexbody_instances) instance->Update(nodes, vehicle_world_offset);
    }
''',
'''    void UpdateFlexBodies(const Snapshot& s, const RoR::IOSOgre::AuthoredVisualGeometry& visual)
    {
        EnsureFlexBodies(s, visual);
        if (flexbody_instances.empty()) return;

        flexbody_node_cache.resize(s.nodes.size());
        for (std::size_t i = 0; i < s.nodes.size(); ++i)
            flexbody_node_cache[i] = OgreVec(s.nodes[i]);
        for (auto& instance : flexbody_instances)
            instance->Update(flexbody_node_cache, vehicle_world_offset);
    }
''',
    'reuse flexbody node conversion buffer')

one(
'''    std::vector<std::unique_ptr<RoR::IOSOgre::IOSFlexBody>> flexbody_instances;
''',
'''    std::vector<std::unique_ptr<RoR::IOSOgre::IOSFlexBody>> flexbody_instances;
    std::vector<Ogre::Vector3> flexbody_node_cache;
''',
    'flexbody node conversion cache')

# The old camera constants were per-frame, so a drop from 60 to 35 FPS changed
# the apparent damping and amplified visible stepping.  Preserve roughly the
# same 60 Hz feel while making the filter time-based.
one(
'''        const Ogre::Vector3 target_center = OgreVec(t.center) + vehicle_world_offset;
        if (!camera_started) { camera_center=target_center; camera_heading=t.heading_radians; camera_started=true; }
        camera_center += (target_center-camera_center)*0.14f;
        camera_heading = WrapAngle(camera_heading + WrapAngle(t.heading_radians-camera_heading)*0.12f);
''',
'''        const Ogre::Vector3 target_center = OgreVec(t.center) + vehicle_world_offset;
        const double camera_now = SnapshotClockSeconds();
        const float camera_dt = camera_last_update > 0.0
            ? static_cast<float>(std::max(0.001, std::min(0.050, camera_now - camera_last_update)))
            : (1.0f / static_cast<float>(kRenderFps));
        camera_last_update = camera_now;
        if (!camera_started) { camera_center=target_center; camera_heading=t.heading_radians; camera_started=true; }
        const float center_alpha = 1.0f - std::exp(-9.05f * camera_dt);
        const float heading_alpha = 1.0f - std::exp(-7.67f * camera_dt);
        camera_center += (target_center-camera_center)*center_alpha;
        camera_heading = WrapAngle(camera_heading + WrapAngle(t.heading_radians-camera_heading)*heading_alpha);
''',
    'time based camera smoothing')

one(
'''    float look_yaw=0.0f;
    float look_pitch=0.0f;
''',
'''    float look_yaw=0.0f;
    float look_pitch=0.0f;
    double camera_last_update=0.0;
''',
    'camera timing state')

p.write_text(s)
print('optimized iOS render bridge: 120 Hz snapshots, 60 Hz interpolation, cached flexbody conversion, time-based camera')
