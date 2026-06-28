#include "metal/MetalMesh.hpp"

#import <AppKit/AppKit.h>

#include <filesystem>
#include <iostream>

id<MTLBuffer> makeMetalBuffer(id<MTLDevice> device, const std::vector<MetalVertex>& vertices) {
    if (vertices.empty()) return nil;
    return [device newBufferWithBytes:vertices.data()
                               length:vertices.size() * sizeof(MetalVertex)
                              options:MTLResourceStorageModeShared];
}

namespace {

id<MTLTexture> loadTexture(id<MTLDevice> device, const std::string& path) {
    if (path.empty() || !std::filesystem::exists(path)) return nil;

    NSString* nsPath = [NSString stringWithUTF8String:path.c_str()];
    NSImage* image = [[NSImage alloc] initWithContentsOfFile:nsPath];
    if (!image) return nil;

    CGImageRef cgImage = [image CGImageForProposedRect:nullptr context:nil hints:nil];
    if (!cgImage) return nil;

    const NSUInteger width = static_cast<NSUInteger>(CGImageGetWidth(cgImage));
    const NSUInteger height = static_cast<NSUInteger>(CGImageGetHeight(cgImage));
    if (width == 0 || height == 0) return nil;

    const NSUInteger bytesPerPixel = 4;
    const NSUInteger bytesPerRow = width * bytesPerPixel;
    std::vector<std::uint8_t> pixels(static_cast<size_t>(bytesPerRow) * height);

    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    CGBitmapInfo bitmapInfo = static_cast<CGBitmapInfo>(
        static_cast<std::uint32_t>(kCGImageAlphaPremultipliedLast) |
        static_cast<std::uint32_t>(kCGBitmapByteOrder32Big));
    CGContextRef context = CGBitmapContextCreate(pixels.data(),
                                                 width,
                                                 height,
                                                 8,
                                                 bytesPerRow,
                                                 colorSpace,
                                                 bitmapInfo);
    CGColorSpaceRelease(colorSpace);
    if (!context) return nil;

    CGContextDrawImage(context, CGRectMake(0.0, 0.0, width, height), cgImage);
    CGContextRelease(context);

    MTLTextureDescriptor* desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                                                    width:width
                                                                                   height:height
                                                                                mipmapped:NO];
    desc.usage = MTLTextureUsageShaderRead;
    id<MTLTexture> texture = [device newTextureWithDescriptor:desc];
    if (!texture) return nil;

    MTLRegion region = MTLRegionMake2D(0, 0, width, height);
    [texture replaceRegion:region mipmapLevel:0 withBytes:pixels.data() bytesPerRow:bytesPerRow];
    return texture;
}

} // namespace

bool MetalMesh::upload(id<MTLDevice> device, const uam::SceneMesh& mesh) {
    vertexBuffer_ = nil;
    sampler_ = nil;
    vertexCount_ = 0;
    parts_.clear();
    if (mesh.vertices.empty()) return false;

    std::vector<MetalVertex> vertices;
    vertices.reserve(mesh.vertices.size());
    for (const auto& v : mesh.vertices) {
        vertices.push_back({
            vector_float3{v.position.x, v.position.y, v.position.z},
            vector_float3{v.color.r, v.color.g, v.color.b},
            vector_float3{v.normal.x, v.normal.y, v.normal.z},
            vector_float2{v.texCoord.x, v.texCoord.y},
            0.0f,
            1.0f
        });
    }

    int texturesTried = 0;
    int texturesLoaded = 0;
    if (!mesh.materialRanges.empty()) {
        parts_.reserve(mesh.materialRanges.size());
        for (const auto& range : mesh.materialRanges) {
            if (range.vertexCount == 0) continue;
            DrawPart part;
            part.firstVertex = static_cast<NSUInteger>(range.firstVertex);
            part.vertexCount = static_cast<NSUInteger>(range.vertexCount);
            if (range.hasTexture) {
                ++texturesTried;
                part.texture = loadTexture(device, range.diffuseTexturePath);
                part.useTexture = part.texture != nil;
                if (part.useTexture) {
                    ++texturesLoaded;
                    for (size_t i = range.firstVertex; i < range.firstVertex + range.vertexCount && i < vertices.size(); ++i) {
                        vertices[i].useTexture = 1.0f;
                    }
                } else {
                    std::cerr << "Texture load failed: " << range.diffuseTexturePath << "\n";
                }
            }
            parts_.push_back(part);
        }
    }
    if (parts_.empty()) {
        parts_.push_back({0, static_cast<NSUInteger>(vertices.size()), nil, false});
    }

    if (texturesTried > 0) {
        std::cout << "Metal textures loaded: " << texturesLoaded << " / " << texturesTried << "\n";
    }

    MTLSamplerDescriptor* samplerDesc = [[MTLSamplerDescriptor alloc] init];
    samplerDesc.minFilter = MTLSamplerMinMagFilterLinear;
    samplerDesc.magFilter = MTLSamplerMinMagFilterLinear;
    samplerDesc.mipFilter = MTLSamplerMipFilterNotMipmapped;
    samplerDesc.sAddressMode = MTLSamplerAddressModeRepeat;
    samplerDesc.tAddressMode = MTLSamplerAddressModeRepeat;
    sampler_ = [device newSamplerStateWithDescriptor:samplerDesc];

    vertexBuffer_ = makeMetalBuffer(device, vertices);
    vertexCount_ = vertexBuffer_ ? static_cast<NSUInteger>(vertices.size()) : 0;
    return valid();
}

void MetalMesh::draw(id<MTLRenderCommandEncoder> encoder) const {
    if (!valid()) return;
    [encoder setVertexBuffer:vertexBuffer_ offset:0 atIndex:0];
    if (sampler_) [encoder setFragmentSamplerState:sampler_ atIndex:0];
    for (const auto& part : parts_) {
        [encoder setFragmentTexture:part.useTexture ? part.texture : nil atIndex:0];
        [encoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:part.firstVertex vertexCount:part.vertexCount];
    }
}
