#import <AVFoundation/AVFoundation.h>

#include "RoREngineAudio.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace RoR {
namespace IOSAudio {
namespace {

std::string JoinPath(const std::string& a, const std::string& b)
{
    if (a.empty()) return b;
    return a.back() == '/' ? a + b : a + "/" + b;
}

std::string Trim(const std::string& s)
{
    const std::size_t begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return {};
    const std::size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

float Clamp(float value, float lo, float hi)
{
    return std::max(lo, std::min(value, hi));
}

struct LayerDefinition
{
    float reference_rpm = 0.0f;
    std::string filename;
};

struct ScriptDefinition
{
    std::vector<LayerDefinition> layers;
    std::string starter;
};

ScriptDefinition ParseSoundScript(const std::string& path)
{
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open RoR soundscript: " + path);

    ScriptDefinition result;
    std::string line;
    while (std::getline(input, line))
    {
        const std::size_t comment = line.find_first_of(";#");
        if (comment != std::string::npos) line.resize(comment);
        line = Trim(line);
        if (line.empty()) continue;

        std::istringstream stream(line);
        std::string keyword;
        stream >> keyword;
        if (keyword == "sound")
        {
            LayerDefinition layer;
            if (stream >> layer.reference_rpm >> layer.filename && layer.reference_rpm > 0.0f)
                result.layers.push_back(std::move(layer));
        }
        else if (keyword == "start_sound")
        {
            std::string mode;
            stream >> mode >> result.starter;
        }
    }

    std::sort(result.layers.begin(), result.layers.end(), [](const LayerDefinition& a, const LayerDefinition& b) {
        return a.reference_rpm < b.reference_rpm;
    });
    if (result.layers.empty())
        throw std::runtime_error("RoR soundscript has no engine RPM layers");
    return result;
}

} // namespace

struct EngineAudio::Impl
{
    struct Layer
    {
        float reference_rpm = 0.0f;
        __strong AVAudioPlayerNode* player = nil;
        __strong AVAudioUnitVarispeed* varispeed = nil;
        __strong AVAudioPCMBuffer* buffer = nil;
    };

    std::string directory;
    std::string error;
    __strong AVAudioEngine* engine = nil;
    __strong AVAudioPlayerNode* starter_player = nil;
    __strong AVAudioPCMBuffer* starter_buffer = nil;
    std::vector<Layer> layers;
    bool running = false;
    float smoothed_rpm = 700.0f;

    explicit Impl(const std::string& resource_directory): directory(resource_directory)
    {
        @autoreleasepool
        {
            try
            {
                const ScriptDefinition script = ParseSoundScript(JoinPath(directory, "351Wmustang.soundscript"));
                engine = [[AVAudioEngine alloc] init];

                NSError* session_error = nil;
                AVAudioSession* session = AVAudioSession.sharedInstance;
                [session setCategory:AVAudioSessionCategoryPlayback
                         withOptions:AVAudioSessionCategoryOptionMixWithOthers
                               error:&session_error];
                if (session_error) throw std::runtime_error(session_error.localizedDescription.UTF8String);
                [session setActive:YES error:&session_error];
                if (session_error) throw std::runtime_error(session_error.localizedDescription.UTF8String);

                for (const LayerDefinition& def : script.layers)
                {
                    NSError* file_error = nil;
                    NSString* ns_path = [NSString stringWithUTF8String:JoinPath(directory, def.filename).c_str()];
                    AVAudioFile* file = [[AVAudioFile alloc] initForReading:[NSURL fileURLWithPath:ns_path] error:&file_error];
                    if (!file || file_error)
                        throw std::runtime_error("cannot load engine sample: " + def.filename);

                    const AVAudioFramePosition bounded = std::min<AVAudioFramePosition>(
                        file.length, static_cast<AVAudioFramePosition>(std::numeric_limits<AVAudioFrameCount>::max()));
                    AVAudioPCMBuffer* buffer = [[AVAudioPCMBuffer alloc] initWithPCMFormat:file.processingFormat
                                                                             frameCapacity:static_cast<AVAudioFrameCount>(bounded)];
                    if (![file readIntoBuffer:buffer error:&file_error] || file_error)
                        throw std::runtime_error("cannot decode engine sample: " + def.filename);

                    Layer layer;
                    layer.reference_rpm = def.reference_rpm;
                    layer.player = [[AVAudioPlayerNode alloc] init];
                    layer.varispeed = [[AVAudioUnitVarispeed alloc] init];
                    layer.buffer = buffer;
                    [engine attachNode:layer.player];
                    [engine attachNode:layer.varispeed];
                    [engine connect:layer.player to:layer.varispeed format:buffer.format];
                    [engine connect:layer.varispeed to:engine.mainMixerNode format:buffer.format];
                    layer.player.volume = 0.0f;
                    layers.push_back(layer);
                }

                if (!script.starter.empty())
                {
                    NSError* file_error = nil;
                    NSString* ns_path = [NSString stringWithUTF8String:JoinPath(directory, script.starter).c_str()];
                    AVAudioFile* file = [[AVAudioFile alloc] initForReading:[NSURL fileURLWithPath:ns_path] error:&file_error];
                    if (file && !file_error)
                    {
                        const AVAudioFramePosition bounded = std::min<AVAudioFramePosition>(
                            file.length, static_cast<AVAudioFramePosition>(std::numeric_limits<AVAudioFrameCount>::max()));
                        starter_buffer = [[AVAudioPCMBuffer alloc] initWithPCMFormat:file.processingFormat
                                                                       frameCapacity:static_cast<AVAudioFrameCount>(bounded)];
                        if ([file readIntoBuffer:starter_buffer error:&file_error] && !file_error)
                        {
                            starter_player = [[AVAudioPlayerNode alloc] init];
                            [engine attachNode:starter_player];
                            [engine connect:starter_player to:engine.mainMixerNode format:starter_buffer.format];
                        }
                    }
                }

                if (!layers.empty()) smoothed_rpm = layers.front().reference_rpm;
            }
            catch (const std::exception& e)
            {
                error = e.what();
                layers.clear();
                engine = nil;
            }
        }
    }

    void Start()
    {
        if (!engine || layers.empty() || running) return;
        NSError* start_error = nil;
        if (![engine startAndReturnError:&start_error])
        {
            error = start_error ? start_error.localizedDescription.UTF8String : "AVAudioEngine failed to start";
            return;
        }

        for (Layer& layer : layers)
        {
            [layer.player scheduleBuffer:layer.buffer atTime:nil options:AVAudioPlayerNodeBufferLoops completionHandler:nil];
            [layer.player play];
        }
        running = true;
    }

    void Stop()
    {
        for (Layer& layer : layers) [layer.player stop];
        if (starter_player) [starter_player stop];
        if (engine) [engine stop];
        running = false;
    }

    void PlayStarter()
    {
        if (!starter_player || !starter_buffer) return;
        [starter_player stop];
        [starter_player scheduleBuffer:starter_buffer atTime:nil options:0 completionHandler:nil];
        [starter_player play];
    }

    void UpdateFromVehicle(float driven_speed_mps, float throttle)
    {
        if (!running || layers.empty()) return;
        throttle = Clamp(throttle, 0.0f, 1.0f);

        const float idle = layers.front().reference_rpm;
        const float redline = layers.back().reference_rpm;

        // Temporary clutch/gearbox approximation for the portable core. This is
        // deliberately on the audio side only: it never changes vehicle forces.
        // Five broad speed bands keep RPM climbing through the source recordings
        // while throttle still allows a convincing free-rev at parking speeds.
        const float speed = std::fabs(driven_speed_mps);
        const float band_width = 11.0f;
        const int gear = std::max(1, std::min(5, static_cast<int>(speed / band_width) + 1));
        const float speed_in_band = speed - static_cast<float>(gear - 1) * band_width;
        float target = idle + Clamp(speed_in_band / band_width, 0.0f, 1.0f) * (redline - idle) * 0.92f;
        if (speed < 2.0f)
            target = std::max(target, idle + throttle * (redline - idle) * 0.68f);
        target = Clamp(target, idle, redline * 1.03f);
        smoothed_rpm += (target - smoothed_rpm) * 0.16f;

        std::vector<float> weights(layers.size(), 0.0f);
        if (layers.size() == 1)
            weights[0] = 1.0f;
        else if (smoothed_rpm <= layers.front().reference_rpm)
            weights.front() = 1.0f;
        else if (smoothed_rpm >= layers.back().reference_rpm)
            weights.back() = 1.0f;
        else
        {
            for (std::size_t i = 0; i + 1 < layers.size(); ++i)
            {
                const float lo = layers[i].reference_rpm;
                const float hi = layers[i + 1].reference_rpm;
                if (smoothed_rpm >= lo && smoothed_rpm <= hi)
                {
                    const float t = Clamp((smoothed_rpm - lo) / std::max(1.0f, hi - lo), 0.0f, 1.0f);
                    weights[i] = std::cos(t * 0.5f * 3.14159265358979323846f);
                    weights[i + 1] = std::sin(t * 0.5f * 3.14159265358979323846f);
                    break;
                }
            }
        }

        const float load_gain = 0.72f + throttle * 0.28f;
        for (std::size_t i = 0; i < layers.size(); ++i)
        {
            Layer& layer = layers[i];
            layer.varispeed.rate = Clamp(smoothed_rpm / std::max(1.0f, layer.reference_rpm), 0.55f, 1.75f);
            layer.player.volume = Clamp(weights[i] * load_gain, 0.0f, 1.0f);
        }
    }
};

EngineAudio::EngineAudio(const std::string& resource_directory): m_impl(new Impl(resource_directory)) {}
EngineAudio::~EngineAudio() { if (m_impl) m_impl->Stop(); }
bool EngineAudio::Ready() const { return m_impl && m_impl->engine && !m_impl->layers.empty() && m_impl->error.empty(); }
const std::string& EngineAudio::Error() const { return m_impl->error; }
void EngineAudio::Start() { if (m_impl) m_impl->Start(); }
void EngineAudio::Stop() { if (m_impl) m_impl->Stop(); }
void EngineAudio::PlayStarter() { if (m_impl) m_impl->PlayStarter(); }
void EngineAudio::UpdateFromVehicle(float driven_wheel_speed_mps, float throttle)
{
    if (m_impl) m_impl->UpdateFromVehicle(driven_wheel_speed_mps, throttle);
}

} // namespace IOSAudio
} // namespace RoR
