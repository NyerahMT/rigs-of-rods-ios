#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

static NSString* ReadUTF8(NSString* path)
{
    NSError* error = nil;
    NSString* text = [NSString stringWithContentsOfFile:path
                                               encoding:NSUTF8StringEncoding
                                                  error:&error];
    if (!text)
    {
        fprintf(stderr, "failed to read %s: %s\n",
                path.UTF8String,
                error.localizedDescription.UTF8String);
        exit(2);
    }
    return text;
}

static NSString* ResolveUnifiedInclude(NSString* shader, NSString* header)
{
    NSString* needle = @"#include \"OgreUnifiedShader.h\"";
    NSRange range = [shader rangeOfString:needle];
    if (range.location == NSNotFound)
    {
        fprintf(stderr, "RoRGame.metal no longer includes OgreUnifiedShader.h\n");
        exit(2);
    }

    NSString* replacement = [NSString stringWithFormat:
        @"#line 1 \"OgreUnifiedShader.h\"\n%@\n#line 2 \"RoRGame.metal\"", header];
    return [shader stringByReplacingCharactersInRange:range withString:replacement];
}

static void RequireFunction(id<MTLLibrary> library, NSString* name)
{
    if (![library newFunctionWithName:name])
    {
        fprintf(stderr, "Metal shader compiled but entry point is missing: %s\n", name.UTF8String);
        exit(3);
    }
}

static id<MTLLibrary> CompileStage(id<MTLDevice> device,
                                   NSString* source,
                                   NSString* stageMacro,
                                   NSArray<NSString*>* requiredFunctions)
{
    MTLCompileOptions* options = [[MTLCompileOptions alloc] init];
    options.preprocessorMacros = @{
        @"OGRE_METAL": @0,
        @"OGRE_NATIVE_GLSL_VERSION_DIRECTIVE": @"",
        stageMacro: @1
    };

    NSError* error = nil;
    id<MTLLibrary> library = [device newLibraryWithSource:source
                                                  options:options
                                                    error:&error];
    if (!library)
    {
        fprintf(stderr, "Metal runtime compilation failed for %s:\n%s\n",
                stageMacro.UTF8String,
                error.localizedDescription.UTF8String);
        exit(4);
    }

    for (NSString* name in requiredFunctions)
        RequireFunction(library, name);
    return library;
}

static void SetAttribute(MTLVertexDescriptor* vd,
                         NSUInteger index,
                         MTLVertexFormat format,
                         NSUInteger offset,
                         NSUInteger bufferIndex)
{
    vd.attributes[index].format = format;
    vd.attributes[index].offset = offset;
    vd.attributes[index].bufferIndex = bufferIndex;
}

static void RequirePipeline(id<MTLDevice> device,
                            id<MTLLibrary> vertexLibrary,
                            NSString* vertexName,
                            id<MTLLibrary> fragmentLibrary,
                            NSString* fragmentName,
                            MTLVertexDescriptor* vertexDescriptor,
                            NSString* label)
{
    MTLRenderPipelineDescriptor* descriptor = [[MTLRenderPipelineDescriptor alloc] init];
    descriptor.label = label;
    descriptor.vertexFunction = [vertexLibrary newFunctionWithName:vertexName];
    descriptor.fragmentFunction = [fragmentLibrary newFunctionWithName:fragmentName];
    descriptor.vertexDescriptor = vertexDescriptor;
    descriptor.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;

    NSError* error = nil;
    id<MTLRenderPipelineState> pipeline = [device newRenderPipelineStateWithDescriptor:descriptor error:&error];
    if (!pipeline)
    {
        fprintf(stderr, "Metal PSO validation failed for %s:\n%s\n",
                label.UTF8String,
                error.localizedDescription.UTF8String);
        exit(5);
    }
}

int main(int argc, char** argv)
{
    @autoreleasepool
    {
        if (argc != 3)
        {
            fprintf(stderr, "usage: MetalShaderProbe <RoRGame.metal> <OgreUnifiedShader.h>\n");
            return 2;
        }

        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (!device)
        {
            fprintf(stderr, "no Metal device available for runtime shader probe\n");
            return 2;
        }

        NSString* shaderPath = [NSString stringWithUTF8String:argv[1]];
        NSString* headerPath = [NSString stringWithUTF8String:argv[2]];
        NSString* source = ResolveUnifiedInclude(ReadUTF8(shaderPath), ReadUTF8(headerPath));

        id<MTLLibrary> vertexLibrary = CompileStage(device, source, @"OGRE_VERTEX_SHADER",
            @[@"ror_game_vp", @"ror_prop_vp", @"ror_terrain_vp", @"ror_vehicle_vp"]);
        id<MTLLibrary> fragmentLibrary = CompileStage(device, source, @"OGRE_FRAGMENT_SHADER",
            @[@"ror_game_fp", @"ror_prop_fp", @"ror_terrain_fp", @"ror_vehicle_fp", @"ror_vehicle_emissive_fp"]);

        MTLVertexDescriptor* gameVD = [MTLVertexDescriptor vertexDescriptor];
        SetAttribute(gameVD, 0, MTLVertexFormatFloat3, 0, 0);   // POSITION
        SetAttribute(gameVD, 3, MTLVertexFormatFloat4, 16, 0);  // COLOR0
        gameVD.layouts[0].stride = 32;
        RequirePipeline(device, vertexLibrary, @"ror_game_vp", fragmentLibrary, @"ror_game_fp", gameVD, @"RoR/Game");

        MTLVertexDescriptor* propVD = [MTLVertexDescriptor vertexDescriptor];
        SetAttribute(propVD, 0, MTLVertexFormatFloat3, 0, 0);   // POSITION
        SetAttribute(propVD, 2, MTLVertexFormatFloat3, 12, 0);  // NORMAL
        SetAttribute(propVD, 8, MTLVertexFormatFloat2, 24, 0);  // TEXCOORD0
        propVD.layouts[0].stride = 32;
        RequirePipeline(device, vertexLibrary, @"ror_prop_vp", fragmentLibrary, @"ror_prop_fp", propVD, @"RoR/Prop-no-COLOR0");

        // This mirrors OGRE Terrain's compatibility (non-compressed) vertex
        // declaration: buffer 0 = float3 POSITION + float2 TEXCOORD0, buffer 1
        // = float2 TEXCOORD1 LOD delta. Our flat Simple2 shader intentionally
        // ignores the second stream but the actual PSO still sees the layout.
        MTLVertexDescriptor* terrainVD = [MTLVertexDescriptor vertexDescriptor];
        SetAttribute(terrainVD, 0, MTLVertexFormatFloat3, 0, 0);   // POSITION
        SetAttribute(terrainVD, 8, MTLVertexFormatFloat2, 12, 0);  // TEXCOORD0
        SetAttribute(terrainVD, 9, MTLVertexFormatFloat2, 0, 1);   // TEXCOORD1 / LOD delta
        terrainVD.layouts[0].stride = 20;
        terrainVD.layouts[1].stride = 8;
        RequirePipeline(device, vertexLibrary, @"ror_terrain_vp", fragmentLibrary, @"ror_terrain_fp", terrainVD, @"RoR/Simple2Terrain");

        MTLVertexDescriptor* vehicleVD = [MTLVertexDescriptor vertexDescriptor];
        SetAttribute(vehicleVD, 0, MTLVertexFormatFloat3, 0, 0);   // POSITION
        SetAttribute(vehicleVD, 3, MTLVertexFormatFloat4, 16, 0);  // COLOR0
        SetAttribute(vehicleVD, 8, MTLVertexFormatFloat2, 32, 0);  // TEXCOORD0
        vehicleVD.layouts[0].stride = 48;
        RequirePipeline(device, vertexLibrary, @"ror_vehicle_vp", fragmentLibrary, @"ror_vehicle_fp", vehicleVD, @"RoR/DAFOfficial");

        printf("OGRE-style Metal runtime shader + prop/terrain/vehicle PSO probes passed.\n");
        return 0;
    }
}
