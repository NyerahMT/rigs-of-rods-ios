#import <UIKit/UIKit.h>
#import <QuartzCore/CAMetalLayer.h>
#import <objc/runtime.h>

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

- (NSString *)logTail
{
    NSString *path = [NSTemporaryDirectory() stringByAppendingPathComponent:@"RoROgre.log"];
    NSString *text = [NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil];
    if (!text.length) return @"OGRE log: <empty>";

    NSArray<NSString *> *lines = [text componentsSeparatedByCharactersInSet:NSCharacterSet.newlineCharacterSet];
    NSMutableArray<NSString *> *useful = [NSMutableArray array];
    for (NSString *line in lines)
    {
        NSString *lower = line.lowercaseString;
        if ([lower containsString:@"error"] || [lower containsString:@"exception"] ||
            [lower containsString:@"metal"] || [lower containsString:@"shader"] ||
            [lower containsString:@"render"])
        {
            if (line.length) [useful addObject:line];
        }
    }
    NSArray<NSString *> *source = useful.count ? useful : lines;
    NSUInteger start = source.count > 4 ? source.count - 4 : 0;
    NSMutableArray<NSString *> *tail = [NSMutableArray array];
    for (NSUInteger i = start; i < source.count; ++i)
    {
        NSString *line = source[i];
        if (line.length > 115) line = [line substringToIndex:115];
        if (line.length) [tail addObject:line];
    }
    return tail.count ? [tail componentsJoinedByString:@"\n"] : @"OGRE log: <no useful lines>";
}

- (void)start
{
    _label = [UILabel new];
    _label.translatesAutoresizingMaskIntoConstraints = NO;
    _label.numberOfLines = 8;
    _label.textColor = UIColor.whiteColor;
    _label.backgroundColor = [UIColor colorWithRed:0.20 green:0.00 blue:0.25 alpha:0.88];
    _label.font = [UIFont monospacedSystemFontOfSize:9.0 weight:UIFontWeightSemibold];
    _label.layer.cornerRadius = 7.0;
    _label.layer.masksToBounds = YES;

    _timer = [NSTimer scheduledTimerWithTimeInterval:0.5 target:self selector:@selector(update) userInfo:nil repeats:YES];
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
            [_label.widthAnchor constraintLessThanOrEqualToAnchor:window.widthAnchor multiplier:0.62]
        ]];
    }
    [window bringSubviewToFront:_label];

    UIView *ogre = [self findOgreView:window];
    if (!ogre)
    {
        _label.text = [NSString stringWithFormat:@"DIAG: OGRE VIEW = NO\n%@", [self logTail]];
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
         "bounds %.0fx%.0f  scale %.2f  drawable %.0fx%.0f  hidden=%@ alpha=%.2f\n%@",
        ogre.window ? @"YES" : @"NO",
        isMetal ? @"YES" : @"NO",
        device ? @"YES" : @"NO",
        ogre.bounds.size.width, ogre.bounds.size.height,
        ogre.contentScaleFactor,
        drawable.width, drawable.height,
        ogre.hidden ? @"YES" : @"NO", ogre.alpha,
        [self logTail]];
}

@end
