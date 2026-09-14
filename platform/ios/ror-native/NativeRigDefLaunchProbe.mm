#import <Foundation/Foundation.h>

#include "NativeRigDefBridge.h"

// Transitional proof while the simulation still runs AuthoredVehicleRuntime:
// execute the real desktop RoR parser against the exact bundled vehicle on
// every launch. Once ActorSpawner owns vehicle creation this bootstrap goes
// away because native RigDef will be the normal loading path.
__attribute__((constructor))
static void RoRNativeRigDefLaunchProbe()
{
    @autoreleasepool
    {
        NSString* path = [[NSBundle mainBundle] pathForResource:@"b6b0UID-semi"
                                                         ofType:@"truck"
                                                    inDirectory:@"Content/dafsemi"];
        if (!path)
        {
            NSLog(@"[RoR native] RigDef probe: bundled DAF fixture not found");
            return;
        }

        NSError* error = nil;
        NSString* text = [NSString stringWithContentsOfFile:path
                                                   encoding:NSUTF8StringEncoding
                                                      error:&error];
        if (!text)
        {
            NSLog(@"[RoR native] RigDef probe: failed reading fixture: %@", error);
            return;
        }

        const RoR::IOSNative::RigDefSummary summary =
            RoR::IOSNative::ParseRigDef(std::string(text.UTF8String));

        NSLog(@"[RoR native] RigDef=%s nodes=%zu beams=%zu wheels=%zu engines=%zu axles=%zu shocks=%zu commands=%zu props=%zu flexbodies=%zu wings=%zu",
              summary.ready ? "YES" : "NO",
              summary.nodes,
              summary.beams,
              summary.wheels,
              summary.engines,
              summary.axles,
              summary.shocks,
              summary.commands,
              summary.props,
              summary.flexbodies,
              summary.wings);
    }
}
