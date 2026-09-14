#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>

#include "RigPhysicsBuilder.h"
#include "SimConstants.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace {

constexpr double kMaxFrameCatchup = 0.05;
constexpr int kMaxPhysicsStepsPerFrame = 120;

const char* kLiveRig = R"ROR(RoR iOS Live Beam Probe
set_beam_defaults 180000, 4500, 300000, 500000

nodes
0, -1.00,  1.00, 0
1,  1.00,  1.00, 0
2, -1.00, -1.00, 0
3,  1.00, -1.00, 0

beams
0, 1
0, 2
1, 3
2, 3
0, 3
1, 2

end
)ROR";

} // namespace

@interface RoRPhysicsView : UIView
{
@private
    RoR::IOSVehicleCore::RigPhysicsModel* _model;
    CADisplayLink* _displayLink;
    CFTimeInterval _lastTimestamp;
    double _accumulator;
    uint64_t _physicsSteps;
}
@property(nonatomic, readonly) NSUInteger nodeCount;
@property(nonatomic, readonly) NSUInteger beamCount;
@property(nonatomic, readonly) uint64_t physicsSteps;
- (void)kickTowardPoint:(CGPoint)point;
@end

@implementation RoRPhysicsView

- (instancetype)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    if (self)
    {
        self.backgroundColor = [UIColor colorWithWhite:0.035 alpha:1.0];
        self.multipleTouchEnabled = NO;

        const RoR::PortableRigDef::Document rig = RoR::PortableRigDef::Parse(kLiveRig);
        _model = new RoR::IOSVehicleCore::RigPhysicsModel(
            RoR::IOSVehicleCore::BuildStructuralModel(rig, 25.0f));

        if (_model->nodes.size() >= 4)
        {
            // Two fixed chassis points make this a hanging deformable truss.
            _model->nodes[0].state.immovable = true;
            _model->nodes[1].state.immovable = true;

            // Start slightly displaced so the first frame visibly proves that
            // the solver, not an animation transform, is moving the structure.
            _model->nodes[3].state.position.x += 0.28f;
        }

        _lastTimestamp = 0.0;
        _accumulator = 0.0;
        _physicsSteps = 0;

        _displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(stepFrame:)];
        if (@available(iOS 15.0, *))
        {
            _displayLink.preferredFrameRateRange = CAFrameRateRangeMake(60.0, 120.0, 120.0);
        }
        [_displayLink addToRunLoop:NSRunLoop.mainRunLoop forMode:NSRunLoopCommonModes];
    }
    return self;
}

- (void)dealloc
{
    [_displayLink invalidate];
    delete _model;
    _model = nullptr;
}

- (NSUInteger)nodeCount
{
    return _model ? _model->nodes.size() : 0;
}

- (NSUInteger)beamCount
{
    return _model ? _model->beams.size() : 0;
}

- (uint64_t)physicsSteps
{
    return _physicsSteps;
}

- (void)stepFrame:(CADisplayLink*)link
{
    if (!_model || !_model->errors.empty())
    {
        [self setNeedsDisplay];
        return;
    }

    if (_lastTimestamp <= 0.0)
    {
        _lastTimestamp = link.timestamp;
        [self setNeedsDisplay];
        return;
    }

    const double elapsed = std::min(link.timestamp - _lastTimestamp, kMaxFrameCatchup);
    _lastTimestamp = link.timestamp;
    _accumulator += elapsed;

    int stepsThisFrame = 0;
    while (_accumulator >= PHYSICS_DT && stepsThisFrame < kMaxPhysicsStepsPerFrame)
    {
        RoR::IOSVehicleCore::StepStructuralModel(*_model, DEFAULT_GRAVITY, PHYSICS_DT);
        _accumulator -= PHYSICS_DT;
        ++stepsThisFrame;
        ++_physicsSteps;
    }

    // Never let a long background/resume hitch create an unbounded catch-up spiral.
    if (stepsThisFrame == kMaxPhysicsStepsPerFrame)
    {
        _accumulator = 0.0;
    }

    [self setNeedsDisplay];
}

- (CGPoint)screenPointForNode:(const RoR::IOSVehicleCore::RigPhysicsNode&)node
{
    const CGFloat scale = MIN(self.bounds.size.width, self.bounds.size.height) * 0.18;
    const CGFloat centerX = CGRectGetMidX(self.bounds);
    const CGFloat centerY = CGRectGetMidY(self.bounds) + 30.0;
    return CGPointMake(
        centerX + node.state.position.x * scale,
        centerY - node.state.position.y * scale);
}

- (void)drawRect:(CGRect)rect
{
    [super drawRect:rect];
    (void)rect;

    CGContextRef context = UIGraphicsGetCurrentContext();
    if (!context || !_model)
    {
        return;
    }

    if (!_model->errors.empty())
    {
        NSString* error = [NSString stringWithUTF8String:_model->errors.front().c_str()];
        NSDictionary* attrs = @{
            NSFontAttributeName: [UIFont monospacedSystemFontOfSize:14.0 weight:UIFontWeightMedium],
            NSForegroundColorAttributeName: UIColor.systemRedColor
        };
        [error drawInRect:CGRectInset(self.bounds, 24.0, 80.0) withAttributes:attrs];
        return;
    }

    CGContextSetLineCap(context, kCGLineCapRound);
    CGContextSetLineWidth(context, 3.0);
    CGContextSetStrokeColorWithColor(context, UIColor.systemTealColor.CGColor);

    for (const RoR::IOSVehicleCore::RigPhysicsBeam& beam : _model->beams)
    {
        if (beam.node_a >= _model->nodes.size() || beam.node_b >= _model->nodes.size())
        {
            continue;
        }

        const CGPoint a = [self screenPointForNode:_model->nodes[beam.node_a]];
        const CGPoint b = [self screenPointForNode:_model->nodes[beam.node_b]];
        CGContextMoveToPoint(context, a.x, a.y);
        CGContextAddLineToPoint(context, b.x, b.y);
    }
    CGContextStrokePath(context);

    for (const RoR::IOSVehicleCore::RigPhysicsNode& node : _model->nodes)
    {
        const CGPoint p = [self screenPointForNode:node];
        const CGFloat radius = node.state.immovable ? 8.0 : 7.0;
        CGRect circle = CGRectMake(p.x - radius, p.y - radius, radius * 2.0, radius * 2.0);
        UIColor* color = node.state.immovable ? UIColor.systemOrangeColor : UIColor.systemGreenColor;
        CGContextSetFillColorWithColor(context, color.CGColor);
        CGContextFillEllipseInRect(context, circle);
    }
}

- (void)kickTowardPoint:(CGPoint)point
{
    if (!_model || _model->nodes.size() < 4)
    {
        return;
    }

    RoR::NodeCoreState& node = _model->nodes[3].state;
    const float direction = (point.x < CGRectGetMidX(self.bounds)) ? -1.0f : 1.0f;
    node.velocity.x += direction * 4.5f;
    node.velocity.y += 1.5f;
}

- (void)touchesBegan:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event
{
    [super touchesBegan:touches withEvent:event];
    UITouch* touch = touches.anyObject;
    if (touch)
    {
        [self kickTowardPoint:[touch locationInView:self]];
    }
}

@end

@interface RoRProbeViewController : UIViewController
{
@private
    RoRPhysicsView* _physicsView;
    UILabel* _statusLabel;
    CADisplayLink* _hudLink;
}
@end

@implementation RoRProbeViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    self.view.backgroundColor = UIColor.blackColor;

    _physicsView = [[RoRPhysicsView alloc] initWithFrame:CGRectZero];
    _physicsView.translatesAutoresizingMaskIntoConstraints = NO;
    [self.view addSubview:_physicsView];

    _statusLabel = [[UILabel alloc] init];
    _statusLabel.translatesAutoresizingMaskIntoConstraints = NO;
    _statusLabel.numberOfLines = 0;
    _statusLabel.textAlignment = NSTextAlignmentCenter;
    _statusLabel.font = [UIFont monospacedSystemFontOfSize:14.0 weight:UIFontWeightSemibold];
    _statusLabel.textColor = UIColor.whiteColor;
    [self.view addSubview:_statusLabel];

    UILabel* instruction = [[UILabel alloc] init];
    instruction.translatesAutoresizingMaskIntoConstraints = NO;
    instruction.text = @"Tap left or right to kick the live beam structure";
    instruction.textAlignment = NSTextAlignmentCenter;
    instruction.font = [UIFont systemFontOfSize:13.0 weight:UIFontWeightMedium];
    instruction.textColor = UIColor.secondaryLabelColor;
    instruction.numberOfLines = 0;
    [self.view addSubview:instruction];

    [NSLayoutConstraint activateConstraints:@[
        [_physicsView.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor],
        [_physicsView.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor],
        [_physicsView.topAnchor constraintEqualToAnchor:self.view.topAnchor],
        [_physicsView.bottomAnchor constraintEqualToAnchor:self.view.bottomAnchor],

        [_statusLabel.topAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.topAnchor constant:18.0],
        [_statusLabel.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor constant:20.0],
        [_statusLabel.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor constant:-20.0],

        [instruction.bottomAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.bottomAnchor constant:-18.0],
        [instruction.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor constant:24.0],
        [instruction.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor constant:-24.0]
    ]];

    _hudLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(updateHUD:)];
    _hudLink.preferredFramesPerSecond = 10;
    [_hudLink addToRunLoop:NSRunLoop.mainRunLoop forMode:NSRunLoopCommonModes];
    [self updateHUD:nil];
}

- (void)dealloc
{
    [_hudLink invalidate];
}

- (void)updateHUD:(CADisplayLink*)link
{
    (void)link;
    _statusLabel.text = [NSString stringWithFormat:
        @"RIGS OF RODS — iOS LIVE PHYSICS\n2,000 Hz fixed solver  •  %lu nodes  •  %lu beams\n%llu physics steps",
        (unsigned long)_physicsView.nodeCount,
        (unsigned long)_physicsView.beamCount,
        _physicsView.physicsSteps];
}

@end

@interface RoRProbeAppDelegate : UIResponder <UIApplicationDelegate>
@property(nonatomic, strong) UIWindow *window;
@end

@implementation RoRProbeAppDelegate
- (BOOL)application:(UIApplication*)application didFinishLaunchingWithOptions:(NSDictionary*)launchOptions
{
    (void)application;
    (void)launchOptions;

    self.window = [[UIWindow alloc] initWithFrame:UIScreen.mainScreen.bounds];
    self.window.rootViewController = [[RoRProbeViewController alloc] init];
    [self.window makeKeyAndVisible];
    return YES;
}
@end

int main(int argc, char* argv[])
{
    @autoreleasepool
    {
        return UIApplicationMain(argc, argv, nil, NSStringFromClass([RoRProbeAppDelegate class]));
    }
}
