#import <UIKit/UIKit.h>

#include "BeamPhysics.h"
#include "PortableRigDef.h"

@interface RoRProbeViewController : UIViewController
@end

@implementation RoRProbeViewController
- (void)viewDidLoad
{
    [super viewDidLoad];
    self.view.backgroundColor = UIColor.systemBackgroundColor;

    const std::string rig_text =
        "RoR iOS Probe\n"
        "nodes\n"
        "0, 0, 0, 0\n"
        "1, 1, 0, 0\n"
        "beams\n"
        "0, 1\n"
        "end\n";

    const RoR::PortableRigDef::Document rig = RoR::PortableRigDef::Parse(rig_text);
    const float stress = RoR::CalcBeamStress(0.01f, 0.0f, 100000.0f, 1000.0f);

    NSString *status = [NSString stringWithFormat:
        @"Rigs of Rods iOS bring-up\n\nVehicle core linked: YES\nParsed nodes: %lu\nParsed beams: %lu\nBeam probe stress: %.1f N\n\nARM64 iPhone build",
        (unsigned long)rig.nodes.size(),
        (unsigned long)rig.beams.size(),
        stress];

    UILabel *label = [[UILabel alloc] init];
    label.translatesAutoresizingMaskIntoConstraints = NO;
    label.numberOfLines = 0;
    label.textAlignment = NSTextAlignmentCenter;
    label.font = [UIFont monospacedSystemFontOfSize:17.0 weight:UIFontWeightMedium];
    label.text = status;
    [self.view addSubview:label];

    [NSLayoutConstraint activateConstraints:@[
        [label.leadingAnchor constraintGreaterThanOrEqualToAnchor:self.view.leadingAnchor constant:24.0],
        [label.trailingAnchor constraintLessThanOrEqualToAnchor:self.view.trailingAnchor constant:-24.0],
        [label.centerXAnchor constraintEqualToAnchor:self.view.centerXAnchor],
        [label.centerYAnchor constraintEqualToAnchor:self.view.centerYAnchor]
    ]];
}
@end

@interface RoRProbeAppDelegate : UIResponder <UIApplicationDelegate>
@property(nonatomic, strong) UIWindow *window;
@end

@implementation RoRProbeAppDelegate
- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions
{
    (void)application;
    (void)launchOptions;

    self.window = [[UIWindow alloc] initWithFrame:UIScreen.mainScreen.bounds];
    self.window.rootViewController = [[RoRProbeViewController alloc] init];
    [self.window makeKeyAndVisible];
    return YES;
}
@end

int main(int argc, char *argv[])
{
    @autoreleasepool
    {
        return UIApplicationMain(argc, argv, nil, NSStringFromClass([RoRProbeAppDelegate class]));
    }
}
