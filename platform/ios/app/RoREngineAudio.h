#pragma once

#include <memory>
#include <string>

namespace RoR {
namespace IOSAudio {

class EngineAudio
{
public:
    // `truck_text` is used to select only soundscript blocks actually referenced
    // by the authored vehicle's soundsources section. This keeps one generic iOS
    // audio path usable by stock/community RoR vehicles instead of hardcoding a car.
    EngineAudio(const std::string& resource_directory,
                const std::string& soundscript_filename,
                const std::string& truck_text);
    ~EngineAudio();

    EngineAudio(const EngineAudio&) = delete;
    EngineAudio& operator=(const EngineAudio&) = delete;

    bool Ready() const;
    const std::string& Error() const;
    void Start();
    void Stop();
    void PlayStarter();

    // RPM comes from the same drivetrain state that produces wheel torque.
    void UpdateFromEngine(float engine_rpm, float throttle);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace IOSAudio
} // namespace RoR
