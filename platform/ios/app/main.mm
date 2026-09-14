#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>

#include "AuthoredVehicleRuntime.h"
#include "SimConstants.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace {
constexpr double kMaxFrameCatchup = 0.05;
constexpr int kMaxPhysicsStepsPerFrame = 120;
constexpr float kMetersToMph = 2.23693629f;
}

@interface RoRDriveView : UIView
{
@private
    RoR::IOSVehicleCore::AuthoredVehicleRuntime* _car;
    CADisplayLink* _displayLink;
    CFTimeInterval _lastTimestamp;
    double _accumulator;
    float _steeringInput;
    float _throttleInput;
    float _brakeInput;
    BOOL _handbrakeInput;
    NSString* _loadError;
}
- (void)setSteeringInput:(float)value;
- (void)setThrottleInput:(float)value;
- (void)setBrakeInput:(float)value;
- (void)setHandbrakeInput:(BOOL)value;
- (void)resetCar;
- (NSString*)hudText;
@end

@implementation RoRDriveView

- (instancetype)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    if (self)
    {
        self.backgroundColor = [UIColor colorWithRed:0.025 green:0.030 blue:0.034 alpha:1.0];
        self.userInteractionEnabled = NO;

        NSString* path = [[NSBundle mainBundle] pathForResource:@"b6b0UID-semi"
                                                        ofType:@"truck"
                                                   inDirectory:@"Content/dafsemi"];
        NSError* readError = nil;
        NSString* authoredText = path
            ? [NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:&readError]
            : nil;
        if (!authoredText)
        {
            _loadError = readError.localizedDescription ?: @"Bundled authored DAF .truck file was not found";
        }
        const char* utf8 = authoredText ? authoredText.UTF8String : "";
        _car = new RoR::IOSVehicleCore::AuthoredVehicleRuntime(std::string(utf8 ? utf8 : ""));

        _lastTimestamp = 0.0;
        _accumulator = 0.0;
        _steeringInput = 0.0f;
        _throttleInput = 0.0f;
        _brakeInput = 0.0f;
        _handbrakeInput = NO;

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
    delete _car;
    _car = nullptr;
}

- (void)setSteeringInput:(float)value
{
    _steeringInput = std::max(-1.0f, std::min(value, 1.0f));
}

- (void)setThrottleInput:(float)value
{
    _throttleInput = std::max(0.0f, std::min(value, 1.0f));
}

- (void)setBrakeInput:(float)value
{
    _brakeInput = std::max(0.0f, std::min(value, 1.0f));
}

- (void)setHandbrakeInput:(BOOL)value
{
    _handbrakeInput = value;
}

- (void)resetCar
{
    if (_car)
    {
        _car->Reset();
    }
    _lastTimestamp = 0.0;
    _accumulator = 0.0;
    _steeringInput = 0.0f;
    _throttleInput = 0.0f;
    _brakeInput = 0.0f;
    _handbrakeInput = NO;
    [self setNeedsDisplay];
}

- (void)stepFrame:(CADisplayLink*)link
{
    if (!_car || !_car->Ready())
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

    _car->SetControls(_steeringInput, _throttleInput, _brakeInput, _handbrakeInput);

    int stepsThisFrame = 0;
    while (_accumulator >= PHYSICS_DT && stepsThisFrame < kMaxPhysicsStepsPerFrame)
    {
        _car->Step(PHYSICS_DT);
        _accumulator -= PHYSICS_DT;
        ++stepsThisFrame;
    }

    if (stepsThisFrame == kMaxPhysicsStepsPerFrame)
        _accumulator = 0.0;

    [self setNeedsDisplay];
}

- (CGPoint)projectWorldPoint:(const RoR::PhysicsVec3&)point
                  telemetry:(const RoR::IOSVehicleCore::AuthoredVehicleTelemetry&)telemetry
                      scale:(CGFloat)scale
{
    const float dx = point.x - telemetry.center.x;
    const float dz = point.z - telemetry.center.z;
    const float c = std::cos(telemetry.heading_radians);
    const float s = std::sin(telemetry.heading_radians);
    const float localForward = dx * c + dz * s;
    const float localRight = -dx * s + dz * c;

    const CGFloat centerX = CGRectGetMidX(self.bounds);
    const CGFloat centerY = CGRectGetMidY(self.bounds) - 4.0;
    return CGPointMake(centerX + localRight * scale, centerY - localForward * scale);
}

- (void)drawGrid:(CGContextRef)context
       telemetry:(const RoR::IOSVehicleCore::AuthoredVehicleTelemetry&)telemetry
           scale:(CGFloat)scale
{
    CGContextSaveGState(context);
    CGContextSetLineWidth(context, 1.0);
    CGContextSetStrokeColorWithColor(context, [UIColor colorWithWhite:1.0 alpha:0.055].CGColor);

    const float span = 28.0f;
    const int minX = static_cast<int>(std::floor((telemetry.center.x - span) / 2.0f));
    const int maxX = static_cast<int>(std::ceil((telemetry.center.x + span) / 2.0f));
    const int minZ = static_cast<int>(std::floor((telemetry.center.z - span) / 2.0f));
    const int maxZ = static_cast<int>(std::ceil((telemetry.center.z + span) / 2.0f));

    for (int gx = minX; gx <= maxX; ++gx)
    {
        const float worldX = static_cast<float>(gx) * 2.0f;
        const RoR::PhysicsVec3 a(worldX, 0.0f, telemetry.center.z - span);
        const RoR::PhysicsVec3 b(worldX, 0.0f, telemetry.center.z + span);
        const CGPoint p0 = [self projectWorldPoint:a telemetry:telemetry scale:scale];
        const CGPoint p1 = [self projectWorldPoint:b telemetry:telemetry scale:scale];
        CGContextMoveToPoint(context, p0.x, p0.y);
        CGContextAddLineToPoint(context, p1.x, p1.y);
    }

    for (int gz = minZ; gz <= maxZ; ++gz)
    {
        const float worldZ = static_cast<float>(gz) * 2.0f;
        const RoR::PhysicsVec3 a(telemetry.center.x - span, 0.0f, worldZ);
        const RoR::PhysicsVec3 b(telemetry.center.x + span, 0.0f, worldZ);
        const CGPoint p0 = [self projectWorldPoint:a telemetry:telemetry scale:scale];
        const CGPoint p1 = [self projectWorldPoint:b telemetry:telemetry scale:scale];
        CGContextMoveToPoint(context, p0.x, p0.y);
        CGContextAddLineToPoint(context, p1.x, p1.y);
    }
    CGContextStrokePath(context);
    CGContextRestoreGState(context);
}

- (BOOL)isTireNode:(std::size_t)index
{
    if (!_car)
        return NO;
    const std::vector<std::size_t>& tireNodes = _car->TireNodeIndices();
    return std::find(tireNodes.begin(), tireNodes.end(), index) != tireNodes.end();
}

- (void)drawLoadFailure:(CGContextRef)context
{
    CGContextSetFillColorWithColor(context, [UIColor colorWithRed:0.28 green:0.02 blue:0.02 alpha:0.90].CGColor);
    CGContextFillRect(context, self.bounds);

    NSString* detail = _loadError;
    if (!detail && _car && !_car->Errors().empty())
        detail = [NSString stringWithUTF8String:_car->Errors().front().c_str()];
    if (!detail)
        detail = @"Unknown authored vehicle load error";

    NSString* warning = [NSString stringWithFormat:@"AUTHORED VEHICLE LOAD FAILED\n%@", detail];
    NSMutableParagraphStyle* paragraph = [[NSMutableParagraphStyle alloc] init];
    paragraph.alignment = NSTextAlignmentCenter;
    NSDictionary* attrs = @{
        NSFontAttributeName: [UIFont monospacedSystemFontOfSize:18.0 weight:UIFontWeightBold],
        NSForegroundColorAttributeName: UIColor.whiteColor,
        NSParagraphStyleAttributeName: paragraph
    };
    [warning drawInRect:CGRectInset(self.bounds, 28.0, self.bounds.size.height * 0.30) withAttributes:attrs];
}

- (void)drawRect:(CGRect)rect
{
    [super drawRect:rect];
    (void)rect;

    CGContextRef context = UIGraphicsGetCurrentContext();
    if (!context || !_car)
        return;

    if (!_car->Ready())
    {
        [self drawLoadFailure:context];
        return;
    }

    const RoR::IOSVehicleCore::AuthoredVehicleTelemetry telemetry = _car->Telemetry();
    const CGFloat scale = std::max<CGFloat>(46.0, std::min(self.bounds.size.width, self.bounds.size.height) * 0.125);

    [self drawGrid:context telemetry:telemetry scale:scale];

    // Every visible line is a live authored/generated beam from the solver.
    CGContextSetLineCap(context, kCGLineCapRound);
    CGContextSetLineWidth(context, 1.0);
    CGContextSetStrokeColorWithColor(context, [UIColor colorWithRed:0.18 green:0.72 blue:0.96 alpha:0.42].CGColor);
    for (const auto& beam : _car->BeamPairs())
    {
        if (beam.first >= _car->NodeCount() || beam.second >= _car->NodeCount())
            continue;
        const CGPoint a = [self projectWorldPoint:_car->Node(beam.first).position telemetry:telemetry scale:scale];
        const CGPoint b = [self projectWorldPoint:_car->Node(beam.second).position telemetry:telemetry scale:scale];
        CGContextMoveToPoint(context, a.x, a.y);
        CGContextAddLineToPoint(context, b.x, b.y);
    }
    CGContextStrokePath(context);

    for (std::size_t i = 0; i < _car->NodeCount(); ++i)
    {
        const BOOL tire = [self isTireNode:i];
        const CGPoint p = [self projectWorldPoint:_car->Node(i).position telemetry:telemetry scale:scale];
        const CGFloat radius = tire ? 2.25 : 1.75;
        const CGRect dot = CGRectMake(p.x - radius, p.y - radius, radius * 2.0, radius * 2.0);
        UIColor* color = tire
            ? [UIColor colorWithRed:0.96 green:0.97 blue:0.99 alpha:0.96]
            : [UIColor colorWithRed:0.20 green:0.95 blue:0.58 alpha:0.84];
        CGContextSetFillColorWithColor(context, color.CGColor);
        CGContextFillEllipseInRect(context, dot);
    }

    CGContextSetStrokeColorWithColor(context, UIColor.systemOrangeColor.CGColor);
    CGContextSetLineWidth(context, 3.0);
    const CGPoint markerBase = CGPointMake(CGRectGetMidX(self.bounds), CGRectGetMidY(self.bounds) - 4.0);
    CGContextMoveToPoint(context, markerBase.x, markerBase.y - 86.0);
    CGContextAddLineToPoint(context, markerBase.x, markerBase.y - 108.0);
    CGContextStrokePath(context);

    if (!_car->IsFinite())
    {
        CGContextSetFillColorWithColor(context, [UIColor colorWithRed:0.35 green:0.02 blue:0.02 alpha:0.82].CGColor);
        CGContextFillRect(context, self.bounds);
        NSString* warning = @"PHYSICS STOPPED\nTap RESET";
        NSMutableParagraphStyle* paragraph = [[NSMutableParagraphStyle alloc] init];
        paragraph.alignment = NSTextAlignmentCenter;
        NSDictionary* attrs = @{
            NSFontAttributeName: [UIFont monospacedSystemFontOfSize:24.0 weight:UIFontWeightBold],
            NSForegroundColorAttributeName: UIColor.whiteColor,
            NSParagraphStyleAttributeName: paragraph
        };
        [warning drawInRect:CGRectInset(self.bounds, 24.0, self.bounds.size.height * 0.38) withAttributes:attrs];
    }
}

- (NSString*)hudText
{
    if (!_car)
        return @"AUTHORED ROR VEHICLE — OFFLINE";
    if (!_car->Ready())
        return @"AUTHORED ROR VEHICLE — LOAD FAILED";

    const RoR::IOSVehicleCore::AuthoredVehicleTelemetry t = _car->Telemetry();
    const float mph = t.speed_mps * kMetersToMph;
    const float treadMph = t.driven_wheel_speed_mps * kMetersToMph;
    NSString* name = [NSString stringWithUTF8String:_car->VehicleName().c_str()];
    return [NSString stringWithFormat:
        @"%@ — REAL ROR IMPORT\n%.1f mph  •  driven tread %.1f mph  •  2,000 Hz\n%lu nodes  •  %lu beams  •  %llu steps%@",
        name ?: @"AUTHORED VEHICLE",
        mph,
        treadMph,
        (unsigned long)_car->NodeCount(),
        (unsigned long)_car->BeamPairs().size(),
        t.physics_steps,
        _car->IsFinite() ? @"" : @"  •  PHYSICS STOPPED"];
}

@end

@interface RoRDriveViewController : UIViewController
{
@private
    RoRDriveView* _driveView;
    UILabel* _hud;
    CADisplayLink* _hudLink;
    BOOL _leftHeld;
    BOOL _rightHeld;
    BOOL _gasHeld;
    BOOL _brakeHeld;
    BOOL _handbrakeHeld;
}
@end

@implementation RoRDriveViewController

- (UIButton*)controlButton:(NSString*)title
{
    UIButton* button = [UIButton buttonWithType:UIButtonTypeSystem];
    button.translatesAutoresizingMaskIntoConstraints = NO;
    [button setTitle:title forState:UIControlStateNormal];
    button.titleLabel.font = [UIFont systemFontOfSize:15.0 weight:UIFontWeightBold];
    button.tintColor = UIColor.whiteColor;
    button.backgroundColor = [UIColor colorWithWhite:0.12 alpha:0.88];
    button.layer.cornerRadius = 14.0;
    button.layer.borderWidth = 1.0;
    button.layer.borderColor = [UIColor colorWithWhite:1.0 alpha:0.10].CGColor;
    return button;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    self.view.backgroundColor = UIColor.blackColor;

    _driveView = [[RoRDriveView alloc] initWithFrame:CGRectZero];
    _driveView.translatesAutoresizingMaskIntoConstraints = NO;
    [self.view addSubview:_driveView];

    _hud = [[UILabel alloc] init];
    _hud.translatesAutoresizingMaskIntoConstraints = NO;
    _hud.numberOfLines = 0;
    _hud.textAlignment = NSTextAlignmentCenter;
    _hud.font = [UIFont monospacedSystemFontOfSize:13.0 weight:UIFontWeightSemibold];
    _hud.textColor = UIColor.whiteColor;
    _hud.backgroundColor = [UIColor colorWithWhite:0.02 alpha:0.58];
    _hud.layer.cornerRadius = 11.0;
    _hud.layer.masksToBounds = YES;
    [self.view addSubview:_hud];

    UIButton* left = [self controlButton:@"◀︎"];
    UIButton* right = [self controlButton:@"▶︎"];
    UIButton* brake = [self controlButton:@"BRAKE"];
    UIButton* handbrake = [self controlButton:@"HB"];
    UIButton* gas = [self controlButton:@"GAS"];
    gas.backgroundColor = [UIColor colorWithRed:0.05 green:0.30 blue:0.15 alpha:0.92];
    handbrake.backgroundColor = [UIColor colorWithRed:0.34 green:0.16 blue:0.03 alpha:0.92];
    brake.backgroundColor = [UIColor colorWithRed:0.34 green:0.05 blue:0.05 alpha:0.92];

    [left addTarget:self action:@selector(leftDown:) forControlEvents:UIControlEventTouchDown];
    [left addTarget:self action:@selector(leftUp:) forControlEvents:(UIControlEventTouchUpInside | UIControlEventTouchUpOutside | UIControlEventTouchCancel)];
    [right addTarget:self action:@selector(rightDown:) forControlEvents:UIControlEventTouchDown];
    [right addTarget:self action:@selector(rightUp:) forControlEvents:(UIControlEventTouchUpInside | UIControlEventTouchUpOutside | UIControlEventTouchCancel)];
    [gas addTarget:self action:@selector(gasDown:) forControlEvents:UIControlEventTouchDown];
    [gas addTarget:self action:@selector(gasUp:) forControlEvents:(UIControlEventTouchUpInside | UIControlEventTouchUpOutside | UIControlEventTouchCancel)];
    [brake addTarget:self action:@selector(brakeDown:) forControlEvents:UIControlEventTouchDown];
    [brake addTarget:self action:@selector(brakeUp:) forControlEvents:(UIControlEventTouchUpInside | UIControlEventTouchUpOutside | UIControlEventTouchCancel)];
    [handbrake addTarget:self action:@selector(handbrakeDown:) forControlEvents:UIControlEventTouchDown];
    [handbrake addTarget:self action:@selector(handbrakeUp:) forControlEvents:(UIControlEventTouchUpInside | UIControlEventTouchUpOutside | UIControlEventTouchCancel)];

    UIStackView* controls = [[UIStackView alloc] initWithArrangedSubviews:@[left, right, brake, handbrake, gas]];
    controls.translatesAutoresizingMaskIntoConstraints = NO;
    controls.axis = UILayoutConstraintAxisHorizontal;
    controls.distribution = UIStackViewDistributionFillEqually;
    controls.spacing = 8.0;
    [self.view addSubview:controls];

    UIButton* reset = [self controlButton:@"RESET"];
    [reset addTarget:self action:@selector(resetPressed:) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:reset];

    UILabel* note = [[UILabel alloc] init];
    note.translatesAutoresizingMaskIntoConstraints = NO;
    note.text = @"complete upstream DAF .truck • authored chassis, shocks, hydros + generated RoR wheels";
    note.textColor = [UIColor colorWithWhite:1.0 alpha:0.48];
    note.font = [UIFont systemFontOfSize:10.5 weight:UIFontWeightMedium];
    note.textAlignment = NSTextAlignmentCenter;
    note.numberOfLines = 1;
    [self.view addSubview:note];

    [NSLayoutConstraint activateConstraints:@[
        [_driveView.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor],
        [_driveView.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor],
        [_driveView.topAnchor constraintEqualToAnchor:self.view.topAnchor],
        [_driveView.bottomAnchor constraintEqualToAnchor:self.view.bottomAnchor],

        [_hud.topAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.topAnchor constant:12.0],
        [_hud.centerXAnchor constraintEqualToAnchor:self.view.centerXAnchor],
        [_hud.widthAnchor constraintLessThanOrEqualToAnchor:self.view.widthAnchor constant:-28.0],

        [reset.topAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.topAnchor constant:14.0],
        [reset.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor constant:-14.0],
        [reset.widthAnchor constraintEqualToConstant:62.0],
        [reset.heightAnchor constraintEqualToConstant:34.0],

        [controls.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor constant:12.0],
        [controls.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor constant:-12.0],
        [controls.bottomAnchor constraintEqualToAnchor:self.view.safeAreaLayoutGuide.bottomAnchor constant:-10.0],
        [controls.heightAnchor constraintEqualToConstant:64.0],

        [note.bottomAnchor constraintEqualToAnchor:controls.topAnchor constant:-7.0],
        [note.centerXAnchor constraintEqualToAnchor:self.view.centerXAnchor]
    ]];

    _hudLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(updateHUD:)];
    _hudLink.preferredFramesPerSecond = 10;
    [_hudLink addToRunLoop:NSRunLoop.mainRunLoop forMode:NSRunLoopCommonModes];
    [self updateControls];
    [self updateHUD:nil];
}

- (void)dealloc
{
    [_hudLink invalidate];
}

- (void)updateControls
{
    const float steering = (_rightHeld ? 1.0f : 0.0f) - (_leftHeld ? 1.0f : 0.0f);
    [_driveView setSteeringInput:steering];
    [_driveView setThrottleInput:_gasHeld ? 1.0f : 0.0f];
    [_driveView setBrakeInput:_brakeHeld ? 1.0f : 0.0f];
    [_driveView setHandbrakeInput:_handbrakeHeld];
}

- (void)updateHUD:(CADisplayLink*)link
{
    (void)link;
    _hud.text = [_driveView hudText];
}

- (void)leftDown:(UIButton*)sender { (void)sender; _leftHeld = YES; [self updateControls]; }
- (void)leftUp:(UIButton*)sender { (void)sender; _leftHeld = NO; [self updateControls]; }
- (void)rightDown:(UIButton*)sender { (void)sender; _rightHeld = YES; [self updateControls]; }
- (void)rightUp:(UIButton*)sender { (void)sender; _rightHeld = NO; [self updateControls]; }
- (void)gasDown:(UIButton*)sender { (void)sender; _gasHeld = YES; [self updateControls]; }
- (void)gasUp:(UIButton*)sender { (void)sender; _gasHeld = NO; [self updateControls]; }
- (void)brakeDown:(UIButton*)sender { (void)sender; _brakeHeld = YES; [self updateControls]; }
- (void)brakeUp:(UIButton*)sender { (void)sender; _brakeHeld = NO; [self updateControls]; }
- (void)handbrakeDown:(UIButton*)sender { (void)sender; _handbrakeHeld = YES; [self updateControls]; }
- (void)handbrakeUp:(UIButton*)sender { (void)sender; _handbrakeHeld = NO; [self updateControls]; }

- (void)resetPressed:(UIButton*)sender
{
    (void)sender;
    _leftHeld = _rightHeld = _gasHeld = _brakeHeld = _handbrakeHeld = NO;
    [_driveView resetCar];
    [self updateControls];
}

@end

@interface RoRProbeAppDelegate : UIResponder <UIApplicationDelegate>
@property(nonatomic, strong) UIWindow* window;
@end

@implementation RoRProbeAppDelegate
- (BOOL)application:(UIApplication*)application didFinishLaunchingWithOptions:(NSDictionary*)launchOptions
{
    (void)application;
    (void)launchOptions;
    self.window = [[UIWindow alloc] initWithFrame:UIScreen.mainScreen.bounds];
    self.window.rootViewController = [[RoRDriveViewController alloc] init];
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
