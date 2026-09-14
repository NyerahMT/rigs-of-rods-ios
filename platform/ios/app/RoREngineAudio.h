#pragma once

#include <memory>
#include <string>

namespace RoR {
namespace IOSAudio {

class EngineAudio
{
public:
    explicit EngineAudio(const std::string& resource_directory);
    ~EngineAudio();

    EngineAudio(const EngineAudio&) = delete;
    EngineAudio& operator=(const EngineAudio&) = delete;

    bool Ready() const;
    const std::string& Error() const;
    void Start();
    void Stop();
    void PlayStarter();

    // The portable vehicle core does not yet carry desktop RoR's clutch/gearbox
    // state. For the sound-system bring-up, infer a smooth engine RPM from the
    // driven-wheel speed plus throttle, while preserving the actual soundscript
    // RPM anchors. This function can later accept drivetrain RPM directly.
    void UpdateFromVehicle(float driven_wheel_speed_mps, float throttle);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace IOSAudio
} // namespace RoR
