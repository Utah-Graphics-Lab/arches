#include "stdafx.hpp"
#include "include.hpp"
#include "intersect.hpp"
#include "custom-instr.hpp"

#ifndef __riscv
#include "sim-config.hpp"
#endif

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

#ifdef __riscv 
int main()
{
	kernel(*(const TRaXKernelArgs*)TRAX_KERNEL_ARGS_ADDRESS);
	return 0;
}
#else

int main(int argc, char* argv[])
{
	//Same shared --key=value argument system and scene table as the arches
	//simulator, so a native run and a simulated run are configured identically.
	Arches::SimulationConfig sim_config;
	sim_config.parse(argc, argv);

	std::string scene_name = sim_config.get_string("scene-name");
	std::string dataset_dir = sim_config.get_string("dataset-dir");
	uint pregen_bounce = sim_config.get_int("pregen-bounce");

	TRaXKernelArgs args;
	args.framebuffer_width = sim_config.get_int("framebuffer-width");
	args.framebuffer_height = sim_config.get_int("framebuffer-height");
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	std::vector<uint32_t> fb_vec(args.framebuffer_size);
	args.framebuffer = fb_vec.data();

	args.pregen_rays = sim_config.get_int("pregen-rays");
	//SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));
	args.camera = sim_config.camera;

	std::string mesh_path = dataset_dir + "/" + scene_name + ".obj";
	std::string bvh_cache_path = dataset_dir + "/cache/" + scene_name + ".bvh";

	rtm::Mesh mesh(mesh_path);

#if 0
	for(uint32_t i = 4; i < 5; ++i)
	{
		rtm::BVH::BuildArgs ba;
		ba.cache_path = bvh_cache_path.c_str();
		ba.width = 7;
		ba.build_method = rtm::BVH::SAH;
		ba.silent = false;

		ba.max_prims_merge = rtm::BVH::MAX_FTB;
		ba.merge_nodes = true;
		ba.merge_leafs = true;

		if(i == 0)
		{
			printf("\nGreedy Collapse-----------------------------------------------------------------\n");
			ba.leaf_cost = rtm::BVH::LINEAR;
			ba.max_prims_collapse = 3;
			ba.collapse_method = rtm::BVH::GREEDY;
		}

		if(i == 1)
		{
			printf("\nDynamic Collapse----------------------------------------------------------------\n");
			ba.leaf_cost = rtm::BVH::LINEAR;
			ba.max_prims_collapse = 3;
			ba.collapse_method = rtm::BVH::DYNAMIC;
		}

		if(i == 2)
		{
			printf("\nCompression-aware Greedy Collapse-----------------------------------------------\n");
			ba.leaf_cost = 2;
			ba.max_prims_collapse = rtm::BVH::MAX_FTB;
			ba.collapse_method = rtm::BVH::GREEDY;
		}

		if(i == 3)
		{
			printf("\nCompression-aware Dynamic Collapse----------------------------------------------\n");
			ba.leaf_cost = 2;
			ba.max_prims_collapse = rtm::BVH::MAX_FTB;
			ba.collapse_method = rtm::BVH::DYNAMIC;
		}

		if(i == 4)
		{
			printf("\nCompression-aware Dynamic Collapse----------------------------------------------\n");
			ba.leaf_cost = rtm::BVH::LINEAR;
			ba.max_prims_collapse = 1;
			ba.collapse_method = rtm::BVH::DYNAMIC;
			ba.merge_nodes = false;
			ba.merge_leafs = false;
		}


		float node_collapse_time = 0.0f, leaf_collapse_time = 0.0f, node_merge_time = 0.0f, leaf_merge_time = 0.0f;
		for(uint j = 0; j < 16; ++j)
		{
			rtm::Mesh mesh_cpy(mesh);
			rtm::BVH b(mesh_cpy, ba);
			node_collapse_time += b.node_collapse_time;
			leaf_collapse_time += b.leaf_collapse_time;
			node_merge_time += b.node_merge_time;
			leaf_merge_time += b.leaf_merge_time;
			ba.silent = true;
		}
		printf("BVHN: Leaf Collapse Time: %.1fms\n", leaf_collapse_time / 16);
		printf("BVHN: Node Collapse Time: %.1fms\n", node_collapse_time / 16);
		printf("BVHN: Node Merge Time: %.1fms\n", node_merge_time / 16);
		printf("BVHN: Leaf Merge Time: %.1fms\n", leaf_merge_time / 16);
		float total_time = leaf_collapse_time + node_collapse_time + node_merge_time + leaf_merge_time;
		printf("BVHN: Total Time: %.1fms\n", total_time / 16);
	}
	printf("\n\n");
#endif
	//6909.8 //5259.9
	rtm::CWBVH bvh(mesh, bvh_cache_path.c_str(), sim_config.get_int("bvh-preset"), sim_config.get_int("bvh-merging"));
	args.nodes = bvh.nodes.data();

#if USE_HECWBVH_V1
	args.ftbs = (rtm::FTB*)args.nodes;
#else
	args.ft_blocks = bvh.ftbs.data();
#endif

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
	{
		std::string ray_file = scene_name + "-" + std::to_string(args.framebuffer_width) + "-" + std::to_string(pregen_bounce) + ".rays";
		pregen_rays(args.nodes, args.ft_blocks, mesh, args.framebuffer_width, args.framebuffer_height, args.camera, pregen_bounce, rays);
		args.rays = rays.data();
	}

	args.vertex_indices = mesh.vertex_indices.data();
	args.normal_indices = mesh.normal_indices.data();
	args.tex_coord_indices = mesh.tex_coord_indices.data();

	args.vertices = mesh.vertices.data();
	args.normals = mesh.normals.data();
	args.tex_coords = mesh.tex_coords.data();

	args.material_indices = mesh.material_indices.data();
	args.materials = mesh.materials.data();

	printf("\nStarting Traversal\n");
	auto start = std::chrono::high_resolution_clock::now();

	std::vector<std::thread> threads;
	uint thread_count = 1;
	thread_count = max(std::thread::hardware_concurrency() - 2u, 1u);
	for (uint i = 1; i < thread_count; ++i) threads.emplace_back(kernel, args);
	kernel(args);
	for (uint i = 1; i < thread_count; ++i) threads[i - 1].join();

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	printf("Runtime: %dms\n", (uint)duration.count());
	printf("Node steps: %5.2f\n", (float)node_steps.load() / args.framebuffer_size);
	printf("Prim steps: %5.2f\n", (float)prim_steps.load() / args.framebuffer_size);
	printf("Memory Traffic: %.1f B/ray\n", (float)(node_steps.load() + prim_steps.load()) * sizeof(rtm::CWBVH::Node) / args.framebuffer_size);

#if 0
	std::vector<uint32_t> sorted(fb_vec);
	std::sort(sorted.begin(), sorted.end());
	uint32_t lo10 = sorted[args.framebuffer_size / 10];
	uint32_t hi10 = sorted[args.framebuffer_size * 9 / 10];
	printf("Lo: %d\n", lo10);
	printf("Hi: %d\n", hi10);
	printf("Div: %d\n", hi10 - lo10);

	//138624
	for(uint32_t i = 0; i < args.framebuffer_size; ++i)
		args.framebuffer[i] = encode_pixel(palette((args.framebuffer[i] - 4464.0f) / (8664.0f - 4464.0f)));
#endif

	stbi_flip_vertically_on_write(true);
	stbi_write_png("trax_out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	return 0;
}
#endif
