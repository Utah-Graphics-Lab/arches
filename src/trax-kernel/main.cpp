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

inline rtm::vec3 BaryInterp3 (
rtm::vec3 const &a0,
rtm::vec3 const &a1,
rtm::vec3 const &a2,
rtm::vec2 const &bc
) {
#ifdef __riscv /* this part is only visible to the RISC-V compiler */
    rtm::vec3 a;

    register float src0 asm("f0") = a0.x;
    register float src1 asm("f1") = a0.y;
    register float src2 asm("f2") = a0.z;
    register float src3 asm("f3") = a1.x;
    register float src4 asm("f4") = a1.y;
    register float src5 asm("f5") = a1.z;
    register float src6 asm("f6") = a2.x;
    register float src7 asm("f7") = a2.y;
    register float src8 asm("f8") = a2.z;
    register float src9 asm("f9") = bc.x;
    register float src10 asm("f10") = bc.y;
    register float dst0 asm("f28");
    register float dst1 asm("f29");
    register float dst2 asm("f30");

    asm volatile (
        ".insn u 0x0b, x0, 0x00018\n\t"
        : "=f"(dst0), "=f"(dst1), "=f"(dst2)
        : "f"(src0), "f"(src1), "f"(src2),
        "f"(src3), "f"(src4), "f"(src5),
        "f"(src6), "f"(src7), "f"(src8),
        "f"(src9), "f"(src10)
    );

    a.x = dst0;
    a.y = dst1;
    a.z = dst2;

    return a;
#else /* handle gracefully */
    return a0 * bc.x + a1 * bc.y + a2 * (1.0f - bc.x - bc.y);
#endif
}

int main(void)
{
    const uint TILE_X = 4;
    const uint TILE_Y = 8;
    const uint TILE_SIZE = TILE_X * TILE_Y;

    static_assert(TILE_SIZE == 32);

    rtm::vec3 light_dir = rtm::normalize(rtm::vec3(1,2,3));

    TRaXKernelArgs args = *(TRaXKernelArgs *)(void *)(TRAX_KERNEL_ARGS_ADDRESS);
    for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
    {
        uint tile_id = index / TILE_SIZE;
        uint toffset = index % TILE_SIZE;
        uint tile_x = tile_id % (args.framebuffer_width / TILE_X);
        uint tile_y = tile_id / (args.framebuffer_width / TILE_X);
        uint x = tile_x * TILE_X + toffset % TILE_X;
        uint y = tile_y * TILE_Y + toffset / TILE_X;
        uint32_t fb_index = y * args.framebuffer_width + x;
        float radiance = 0.0f;
        rtm::Ray ray = args.camera.generate_ray_through_pixel(x, y);
        rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
        _traceray<0x0u>(0, ray, hit);
        if(hit.t < ray.t_max)
        {
            rtm::uvec3 ni = args.normal_indices[hit.id];
            rtm::vec3 n0 = args.normals[ni[0]];
            rtm::vec3 n1 = args.normals[ni[1]];
            rtm::vec3 n2 = args.normals[ni[2]];
            rtm::vec3 n = BaryInterp3( n0, n1, n2, hit.bc );
            float d = rtm::dot(n, light_dir);
            radiance = d > 0 ? d * 0.8f : 0.0f;
        }
        args.framebuffer[fb_index] = encode_pixel(radiance);
    }
    return 0;
}