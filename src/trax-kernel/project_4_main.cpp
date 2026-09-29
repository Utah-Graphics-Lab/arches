#include "stdafx.hpp"
#include "include.hpp"
#include "intersect.hpp"
#include "custom-instr.hpp"
inline static uint32_t encode_pixel(rtm::vec3 in)
{
    in = rtm::clamp(in, 0.0f, 1.0f);
    uint32_t out = 0u;
    out |= static_cast<uint32_t>(in.r * 255.0f + 0.5f) << 0;
    out |= static_cast<uint32_t>(in.g * 255.0f + 0.5f) << 8;
    out |= static_cast<uint32_t>(in.b * 255.0f + 0.5f) << 16;
    out |= 0xff << 24;
    return out;
}
int main(void)
{
    const uint SPP = 1;
    const uint TILE_X = 4;
    const uint TILE_Y = 8;
    const uint TILE_SIZE = TILE_X * TILE_Y;
    static_assert(TILE_SIZE == 32);
    TRaXKernelArgs args = *(TRaXKernelArgs*)(void*)(TRAX_KERNEL_ARGS_ADDRESS);
    for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
    {
        uint tile_id = index / TILE_SIZE;
        uint toffset = index % TILE_SIZE;
        uint tile_x = tile_id % (args.framebuffer_width / TILE_X);
        uint tile_y = tile_id / (args.framebuffer_width / TILE_X);
        uint x = tile_x * TILE_X + toffset % TILE_X;
        uint y = tile_y * TILE_Y + toffset / TILE_X;
        uint32_t fb_index = y * args.framebuffer_width + x;
        rtm::RNG rng(fb_index);
        float radiance = 0.0f;
        for (uint i = 0; i < SPP; ++i)
        {
            float throughput = 1.0f;
            rtm::Ray ray = args.camera.generate_ray_through_pixel(x, y);
            // compute ambient occlusion
            for (uint j = 0; j < 3; ++j)
            {
                rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
                _traceray<0x0u>(index, ray, hit);
                if (hit.t >= ray.t_max)
                {
                    radiance += throughput * 2.0f;
                    break;
                }
                rtm::uvec3 ni = args.normal_indices[hit.id];
                rtm::vec3 n0 = args.normals[ni[0]];
                rtm::vec3 n1 = args.normals[ni[2]];
                rtm::vec3 n2 = args.normals[ni[3]];
                rtm::vec3 n =
                    (n0 * hit.bc.x) +
                    (n1 * hit.bc.y) +
                    n2 * (1.0f - hit.bc.x - hit.bc.y);
                // generate secondary rays
                ray.o += ray.d * hit.t;
                ray.d = cosine_sample_hemisphere(n, rng);
                throughput *= 0.8f;
            }
        }
        args.framebuffer[fb_index] = encode_pixel(radiance / SPP);
    }
    return 0;
}