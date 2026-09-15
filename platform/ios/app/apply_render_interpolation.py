#!/usr/bin/env python3
"""Add one coherent render-snapshot interpolation layer to the generated iOS shell.

All vehicle visuals (flexbody, tyre surface, static rim, props and camera) must consume
one interpolated node state.  Smoothing those systems independently would introduce
exactly the body/wheel phase error this patch is intended to remove.
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
        raise SystemExit(f"render interpolation anchor '{label}' expected once, found {count}")
    s = s.replace(old, new, 1)


one(
    '#include <chrono>\n',
    '#include <chrono>\n#include <deque>\n',
    'deque include')

one(
    'constexpr double kMaxSimCatchup = 0.05;\n',
    'constexpr double kMaxSimCatchup = 0.05;\n'
    'constexpr double kRenderInterpolationDelay = 1.0 / static_cast<double>(kRenderFps);\n'
    'constexpr double kRenderSnapshotHistory = 0.10;\n',
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

        // Render one display interval behind the simulation.  The physics thread
        // publishes dense 2 kHz states, so this gives us two real simulated
        // states to interpolate between instead of extrapolating or low-pass
        // filtering the body/rims/tyres independently.
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

p.write_text(s)
print('added one-frame buffered interpolation shared by body, rims, tyres, props and camera')
