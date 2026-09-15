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

struct SourceDefinition
{
    std::string name;
    std::string trigger_source;
    std::vector<LayerDefinition> layers;
    std::string start_sound;
};

struct ScriptDefinition
{
    std::vector<LayerDefinition> layers;
    std::string starter;
    std::string engine_source;
    std::string starter_source;
};

bool ReferencedByVehicle(const std::string& truck_text, const std::string& source_name)
{
    if (source_name.empty()) return false;
    // RoR soundsources entries contain the soundscript source name literally.
    // Matching the complete identifier is sufficient here and avoids inventing a
    // second incomplete RigDef parser in the audio layer.
    return truck_text.find(source_name) != std::string::npos;
}

ScriptDefinition ParseSoundScript(const std::string& path, const std::string& truck_text)
{
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open RoR soundscript: " + path);

    std::vector<SourceDefinition> sources;
    SourceDefinition current;
    std::string pending_name;
    bool in_block = false;
    std::string line;

    while (std::getline(input, line))
    {
        const std::size_t comment = line.find_first_of(";#");
        if (comment != std::string::npos) line.resize(comment);
        line = Trim(line);
        if (line.empty()) continue;

        if (!in_block)
        {
            if (line == "{")
            {
                if (!pending_name.empty())
                {
                    current = SourceDefinition();
                    current.name = pending_name;
                    pending_name.clear();
                    in_block = true;
                }
                continue;
            }
            const std::size_t brace = line.find('{');
            if (brace != std::string::npos)
            {
                current = SourceDefinition();
                current.name = Trim(line.substr(0, brace));
                in_block = !current.name.empty();
                continue;
            }
            pending_name = line;
            continue;
        }

        if (line == "}")
        {
            if (!current.name.empty()) sources.push_back(current);
            current = SourceDefinition();
            in_block = false;
            continue;
        }

        std::istringstream stream(line);
        std::string keyword;
        stream >> keyword;
        if (keyword == "trigger_source")
        {
            stream >> current.trigger_source;
        }
        else if (keyword == "sound")
        {
            LayerDefinition layer;
            if (stream >> layer.reference_rpm >> layer.filename && layer.reference_rpm > 0.0f)
                current.layers.push_back(std::move(layer));
        }
        else if (keyword == "start_sound")
        {
            std::string mode;
            stream >> mode >> current.start_sound;
        }
    }
    if (in_block && !current.name.empty()) sources.push_back(current);

    const SourceDefinition* engine_source = nullptr;
    const SourceDefinition* starter_source = nullptr;
    for (const SourceDefinition& source : sources)
    {
        if (!ReferencedByVehicle(truck_text, source.name)) continue;
        if (!engine_source && source.trigger_source == "engine" && !source.layers.empty())
            engine_source = &source;
        if (!starter_source && source.trigger_source == "starter" && !source.start_sound.empty())
            starter_source = &source;
    }
    // Old/simple vehicles sometimes omit soundsources but ship only one engine
    // block. Retain a safe compatibility fallback while preferring authored refs.
    if (!engine_source)
    {
        for (const SourceDefinition& source : sources)
        {
            if (source.trigger_source == "engine" && !source.layers.empty())
            {
                engine_source = &source;
                break;
            }
        }
    }
    if (!starter_source)
    {
        for (const SourceDefinition& source : sources)
        {
            if (source.trigger_source == "starter" && !source.start_sound.empty())
            {
                starter_source = &source;
                break;
            }
        }
    }
    if (!engine_source)
        throw std::runtime_error("RoR soundscript has no referenced engine RPM source");

    ScriptDefinition result;
    result.layers = engine_source->layers;
    result.engine_source = engine_source->name;
    if (starter_source)
    {
        result.starter = starter_source->start_sound;
        result.starter_source = starter_source->name;
    }
    std::sort(result.layers.begin(), result.layers.end(), [](const LayerDefinition& a, const LayerDefinition& b) {
        return a.reference_rpm < b.reference_rpm;
    });
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

    Impl(const std::string& resource_directory,
         const std::string& soundscript_filename,
         const std::string& truck_text): directory(resource_directory)
    {
        @autoreleasepool
        {
            try
            {
                const ScriptDefinition script = ParseSoundScript(
                    JoinPath(directory, soundscript_filename), truck_text);
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

    void UpdateFromEngine(float engine_rpm, float throttle)
    {
        if (!running || layers.empty()) return;
        throttle = Clamp(throttle, 0.0f, 1.0f);
        const float idle = layers.front().reference_rpm;
        const float highest_anchor = layers.back().reference_rpm;
        const float target = std::max(idle, engine_rpm);
        smoothed_rpm += (target - smoothed_rpm) * 0.22f;

        std::vector<float> weights(layers.size(), 0.0f);
        if (layers.size() == 1)
            weights[0] = 1.0f;
        else if (smoothed_rpm <= layers.front().reference_rpm)
            weights.front() = 1.0f;
        else if (smoothed_rpm >= highest_anchor)
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

        const float load_gain = 0.70f + throttle * 0.30f;
        for (std::size_t i = 0; i < layers.size(); ++i)
        {
            Layer& layer = layers[i];
            layer.varispeed.rate = Clamp(smoothed_rpm / std::max(1.0f, layer.reference_rpm), 0.50f, 1.90f);
            layer.player.volume = Clamp(weights[i] * load_gain, 0.0f, 1.0f);
        }
    }
};

EngineAudio::EngineAudio(const std::string& resource_directory,
                         const std::string& soundscript_filename,
                         const std::string& truck_text)
    : m_impl(new Impl(resource_directory, soundscript_filename, truck_text)) {}
EngineAudio::~EngineAudio() { if (m_impl) m_impl->Stop(); }
bool EngineAudio::Ready() const { return m_impl && m_impl->engine && !m_impl->layers.empty() && m_impl->error.empty(); }
const std::string& EngineAudio::Error() const { return m_impl->error; }
void EngineAudio::Start() { if (m_impl) m_impl->Start(); }
void EngineAudio::Stop() { if (m_impl) m_impl->Stop(); }
void EngineAudio::PlayStarter() { if (m_impl) m_impl->PlayStarter(); }
void EngineAudio::UpdateFromEngine(float engine_rpm, float throttle)
{
    if (m_impl) m_impl->UpdateFromEngine(engine_rpm, throttle);
}

} // namespace IOSAudio
} // namespace RoR
