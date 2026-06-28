#pragma once

#import <Metal/Metal.h>

#include "core/SceneMesh.hpp"

#include <simd/simd.h>
#include <vector>

struct MetalVertex {
    vector_float3 position;
    vector_float3 color;
    vector_float3 normal{0.0f, 1.0f, 0.0f};
    vector_float2 texCoord{0.0f, 0.0f};
    float useTexture = 0.0f;
    float useLighting = 0.0f;
};

class MetalMesh {
public:
    MetalMesh() = default;

    bool upload(id<MTLDevice> device, const uam::SceneMesh& mesh);
    bool valid() const { return vertexBuffer_ != nil && vertexCount_ > 0; }
    NSUInteger vertexCount() const { return vertexCount_; }
    void draw(id<MTLRenderCommandEncoder> encoder) const;

private:
    struct DrawPart {
        NSUInteger firstVertex = 0;
        NSUInteger vertexCount = 0;
        id<MTLTexture> texture = nil;
        bool useTexture = false;
    };

    id<MTLBuffer> vertexBuffer_ = nil;
    id<MTLSamplerState> sampler_ = nil;
    NSUInteger vertexCount_ = 0;
    std::vector<DrawPart> parts_;
};

id<MTLBuffer> makeMetalBuffer(id<MTLDevice> device, const std::vector<MetalVertex>& vertices);
