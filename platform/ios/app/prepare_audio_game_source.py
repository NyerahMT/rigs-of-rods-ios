#!/usr/bin/env python3
"""Generate the iOS game-view source with the RoR engine-audio bridge wired in.

Kept as a guarded source transform during bring-up so the renderer/controller
stays readable while the audio bridge is still experimental. Every replacement
has a strict anchor and fails the build if the game source changes underneath it.
"""

from pathlib import Path
import sys

if len(sys.argv) != 3:
    raise SystemExit("usage: prepare_audio_game_source.py <OgreGameApp.mm> <output.mm>")

src = Path(sys.argv[1])
out = Path(sys.argv[2])
text = src.read_text()


def replace_once(old: str, new: str, label: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"audio source patch anchor '{label}' expected once, found {count}")
    text = text.replace(old, new, 1)


replace_once(
    '#include "AuthoredVehicleRuntime.h"\n',
    '#include "AuthoredVehicleRuntime.h"\n#include "RoREngineAudio.h"\n',
    "audio include",
)

replace_once(
    '    SimulationHost* _simulation; OgreRenderer* _renderer; UIView* _ogreView; CADisplayLink* _displayLink; UILabel* _speed; UILabel* _status;\n',
    '    SimulationHost* _simulation; OgreRenderer* _renderer; UIView* _ogreView; CADisplayLink* _displayLink; UILabel* _speed; UILabel* _status;\n'
    '    std::unique_ptr<RoR::IOSAudio::EngineAudio> _engineAudio;\n',
    "controller audio member",
)

replace_once(
    '    _simulation=new SimulationHost(std::string(text?text.UTF8String:""));\n',
    '    _simulation=new SimulationHost(std::string(text?text.UTF8String:""));\n'
    '    NSString* audioPath=[[[NSBundle mainBundle] resourcePath] stringByAppendingPathComponent:@"Content/foxbody-audio"];\n'
    '    _engineAudio.reset(new RoR::IOSAudio::EngineAudio(std::string(audioPath.UTF8String)));\n'
    '    if(_engineAudio->Ready()){_engineAudio->Start();_engineAudio->PlayStarter();}\n',
    "audio construction",
)

replace_once(
    '- (void)dealloc{[_displayLink invalidate];delete _renderer;delete _simulation;}\n',
    '- (void)dealloc{[_displayLink invalidate];_engineAudio.reset();delete _renderer;delete _simulation;}\n',
    "audio teardown",
)

replace_once(
    '    Snapshot s=_simulation->GetSnapshot(); if(_renderer)_renderer->Draw(s,_simulation->Visual()); _speed.text=[NSString stringWithFormat:@"%3.0f MPH",s.telemetry.speed_mps*kMetersToMph];\n',
    '    Snapshot s=_simulation->GetSnapshot(); if(_engineAudio&&_engineAudio->Ready())_engineAudio->UpdateFromVehicle(s.telemetry.driven_wheel_speed_mps,s.telemetry.throttle); if(_renderer)_renderer->Draw(s,_simulation->Visual()); _speed.text=[NSString stringWithFormat:@"%3.0f MPH",s.telemetry.speed_mps*kMetersToMph];\n',
    "audio frame update",
)

replace_once(
    '- (void)reset:(id)x{(void)x;_left=_right=_gas=_brake=_handbrake=NO;[self push];_simulation->Reset();if(_renderer)_renderer->ResetLook();}\n',
    '- (void)reset:(id)x{(void)x;_left=_right=_gas=_brake=_handbrake=NO;[self push];_simulation->Reset();if(_engineAudio&&_engineAudio->Ready())_engineAudio->PlayStarter();if(_renderer)_renderer->ResetLook();}\n',
    "starter on reset",
)

out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(text)
print(f"Generated audio-enabled game source: {out}")
