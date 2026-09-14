#import <UIKit/UIKit.h>
#import <QuartzCore/CAMetalLayer.h>
#import <objc/runtime.h>

static NSString *gCapturedRendererFailure = nil;

@interface UILabel (RoROgreFailureCapture)
@end

@implementation UILabel (RoROgreFailureCapture)

+ (void)load
{
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        Method original = class_getInstanceMethod(self, @selector(setText:));
        Method replacement = class_getInstanceMethod(self, @selector(ror_diag_setText:));
        method_exchangeImplementations(original, replacement);
    });
}

- (void)ror_diag_setText:(NSString *)text
{
    if (text.length)
    {
        NSString *upper = text.uppercaseString;
        if (([upper containsString:@"OGRE"] || [upper containsString:@"RENDER"]) &&
            ([upper containsString:@"FAILED"] || [upper containsString:@"EXCEPTION"] || [upper containsString:@"ERROR"]))
        {
            @synchronized(UILabel.class)
            {
                gCapturedRendererFailure = [text copy];
            }
        }
    }
    [self ror_diag_setText:text];
}

@end

@interface RoROgreRuntimeDiagnostics : NSObject
@end

@implementation RoROgreRuntimeDiagnostics
{
    UILabel *_label;
    NSTimer *_timer;
}

+ (void)load
{
    dispatch_async(dispatch_get_main_queue(), ^{
        RoROgreRuntimeDiagnostics *diag = [RoROgreRuntimeDiagnostics new];
        objc_setAssociatedObject(UIApplication.sharedApplication, @selector(load), diag, OBJC_ASSOCIATION_RETAIN_NONATOMIC);
        [diag start];
    });
}

- (UIWindow *)activeWindow
{
    for (UIScene *scene in UIApplication.sharedApplication.connectedScenes)
    {
        if (![scene isKindOfClass:UIWindowScene.class]) continue;
        UIWindowScene *windowScene = (UIWindowScene *)scene;
        for (UIWindow *window in windowScene.windows)
            if (window.isKeyWindow) return window;
    }
    return UIApplication.sharedApplication.windows.firstObject;
}

- (UIView *)findOgreView:(UIView *)root
{
    if (!root) return nil;
    if ([NSStringFromClass(root.class) isEqualToString:@"OgreMetalView"]) return root;
    for (UIView *child in root.subviews)
    {
        UIView *found = [self findOgreView:child];
        if (found) return found;
    }
    return nil;
}

- (NSString *)capturedFailure
{
    @synchronized(UILabel.class)
    {
        return gCapturedRendererFailure ?: @"";
    }
}

- (NSString *)logTail
{
    NSString *path = [NSTemporaryDirectory() stringByAppendingPathComponent:@"RoROgre.log"];
    NSString *text = [NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil];
    if (!text.length) return @"OGRE log: <empty>";

    NSArray<NSString *> *lines = [text componentsSeparatedByCharactersInSet:NSCharacterSet.newlineCharacterSet];
    NSUInteger start = lines.count > 7 ? lines.count - 7 : 0;
    NSMutableArray<NSString *> *tail = [NSMutableArray array];
    for (NSUInteger i = start; i < lines.count; ++i)
    {
        NSString *line = lines[i];
        if (line.length > 150) line = [line substringToIndex:150];
        if (line.length) [tail addObject:line];
    }
    return tail.count ? [tail componentsJoinedByString:@"\n"] : @"OGRE log: <no lines>";
}

- (void)start
{
    _label = [UILabel new];
    _label.translatesAutoresizingMaskIntoConstraints = NO;
    _label.numberOfLines = 14;
    _label.lineBreakMode = NSLineBreakByWordWrapping;
    _label.textColor = UIColor.whiteColor;
    _label.backgroundColor = [UIColor colorWithRed:0.20 green:0.00 blue:0.25 alpha:0.91];
    _label.font = [UIFont monospacedSystemFontOfSize:8.5 weight:UIFontWeightSemibold];
    _label.layer.cornerRadius = 7.0;
    _label.layer.masksToBounds = YES;

    _timer = [NSTimer scheduledTimerWithTimeInterval:0.35 target:self selector:@selector(update) userInfo:nil repeats:YES];
    [_timer fire];
}

- (void)update
{
    UIWindow *window = [self activeWindow];
    if (!window) return;

    if (!_label.superview)
    {
        [window addSubview:_label];
        [NSLayoutConstraint activateConstraints:@[
            [_label.centerXAnchor constraintEqualToAnchor:window.centerXAnchor],
            [_label.topAnchor constraintEqualToAnchor:window.safeAreaLayoutGuide.topAnchor constant:8],
            [_label.widthAnchor constraintLessThanOrEqualToAnchor:window.widthAnchor multiplier:0.80]
        ]];
    }
    [window bringSubviewToFront:_label];

    NSString *failure = [self capturedFailure];
    UIView *ogre = [self findOgreView:window];
    if (!ogre)
    {
        if (failure.length)
            _label.text = [NSString stringWithFormat:@"DIAG: OGRE VIEW = NO\nCAPTURED INIT FAILURE:\n%@\n--- LOG TAIL ---\n%@", failure, [self logTail]];
        else
            _label.text = [NSString stringWithFormat:@"DIAG: OGRE VIEW = NO\nCAPTURED INIT FAILURE: <none yet>\n--- LOG TAIL ---\n%@", [self logTail]];
        return;
    }

    ogre.layer.borderWidth = 3.0;
    ogre.layer.borderColor = UIColor.magentaColor.CGColor;

    BOOL isMetal = [ogre.layer isKindOfClass:CAMetalLayer.class];
    CAMetalLayer *metal = isMetal ? (CAMetalLayer *)ogre.layer : nil;
    CGSize drawable = metal ? metal.drawableSize : CGSizeZero;
    id device = metal ? metal.device : nil;

    _label.text = [NSString stringWithFormat:
        @"DIAG: OGRE VIEW = YES  window=%@  metal=%@  device=%@\n"
         "bounds %.0fx%.0f  scale %.2f  drawable %.0fx%.0f  hidden=%@ alpha=%.2f\n%@%@",
        ogre.window ? @"YES" : @"NO",
        isMetal ? @"YES" : @"NO",
        device ? @"YES" : @"NO",
        ogre.bounds.size.width, ogre.bounds.size.height,
        ogre.contentScaleFactor,
        drawable.width, drawable.height,
        ogre.hidden ? @"YES" : @"NO", ogre.alpha,
        failure.length ? [NSString stringWithFormat:@"CAPTURED FAILURE: %@\n", failure] : @"",
        [self logTail]];
}

@end
