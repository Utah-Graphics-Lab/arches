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

inline rtm::vec3 palette(float t)
{
	t = rtm::clamp(t, 0.0, 1.0f);

	const rtm::vec3 k[] = {
		rtm::vec3( 000.13572138f, 000.09140261f, 000.10667330f),
		rtm::vec3( 004.61539260f, 002.19418839f, 012.64194608f),
		rtm::vec3(-042.66032258f, 004.84296658f,-060.58204836f),
		rtm::vec3( 132.13108234f,-014.18503333f, 110.36276771f),
		rtm::vec3(-152.94239396f, 004.27729857f,-089.90310912f),
		rtm::vec3( 059.28637943f, 002.82956604f, 027.34824973f)
	};
	
	float t_pow = t;
	rtm::vec3 color(k[1] * t + k[0]);
	for(uint i = 2; i < 6; ++i)
	{
		t_pow *= t;
		color += k[i] * t_pow;
	}

	return rtm::clamp(color);
}

#ifndef  __riscv
static std::atomic_uint node_steps = 0;
static std::atomic_uint prim_steps = 0;
#endif

inline static void kernel(const TRaXKernelArgs& args)
{
	constexpr uint32_t SPP = 1;
	constexpr uint TILE_X = 4;
	constexpr uint TILE_Y = 8;
	constexpr uint TILE_SIZE = TILE_X * TILE_Y;
	
	for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint tile_id = index / TILE_SIZE;
		//tile_id = rtm::RNG::fast_hash(tile_id) % (args.framebuffer_size / TILE_SIZE);
		uint32_t tile_x = tile_id % (args.framebuffer_width / TILE_X);
		uint32_t tile_y = tile_id / (args.framebuffer_width / TILE_X);
		uint thread_id = index % TILE_SIZE;
		uint32_t x = tile_x * TILE_X + thread_id % TILE_X;
		uint32_t y = tile_y * TILE_Y + thread_id / TILE_X;
		uint fb_index = y * args.framebuffer_width + x;
		
		rtm::RNG rng(fb_index);
		IntersectStats stats;

		float radiance = 0.0f;
		for(uint32_t i = 0; i < SPP; ++i)
		{
			float throughput = 1.0f;
			rtm::Ray ray = args.pregen_rays ? args.rays[fb_index] : args.camera.generate_ray_through_pixel(x, y);
			for(uint32_t j = 0; j < 3; ++j)
			{
				rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
			#if defined(__riscv) && (TRAX_USE_RT_CORE)
				_traceray<0x0u>(index, ray, hit);
			#else
				intersect(args.nodes, args.ftbs, ray, hit, stats);
			#endif

				if(hit.t >= ray.t_max)
				{
					radiance += throughput * 16.0f;
					break;
				}

				rtm::vec3 n = args.tris[hit.id].normal();
				uint hash = rtm::RNG::hash(hit.id) | 0xff'00'00'00;

				ray.o += ray.d * hit.t;
				ray.d = cosine_sample_hemisphere(n, rng); // generate secondray rays
				throughput *= 0.8f;
			}
		}

		//args.framebuffer[fb_index] = (stats.node_steps * sizeof(rtm::CWBVH::Node) + stats.prim_steps * sizeof(rtm::FTB)) / SPP;

		args.framebuffer[fb_index] = encode_pixel(radiance / SPP);
	#ifndef __riscv
		node_steps += stats.node_steps;
		prim_steps += stats.prim_steps;
	#endif
	}
}

inline static void mandelbrot(const TRaXKernelArgs& args)
{
	constexpr uint MAX_ITERS = 100;
	for(uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		rtm::Ray ray = args.rays[index];

		// convert 1-d loop in to 2-d loop indices
		const int i = index / args.framebuffer_width;
		const int j = index % args.framebuffer_width;

		float zreal = 0.0f;
		float zimag = 0.0f;
		float creal = (j - args.framebuffer_width / 1.4f) / (args.framebuffer_width / 2.0f);
		float cimag = (i - args.framebuffer_height / 2.0f) / (args.framebuffer_height / 2.0f);

		float lengthsq;
		uint k = 0;
		do
		{
			float temp = (zreal * zreal) - (zimag * zimag) + creal;
			zimag = (2.0f * zreal * zimag) + cimag;
			zreal = temp;
			lengthsq = (zreal * zreal) + (zimag * zimag);
			++k;
		}
		while(lengthsq < 4.f && k < MAX_ITERS);

		if(k == MAX_ITERS)
			k = 0;

		args.framebuffer[index] = encode_pixel(k / (float)MAX_ITERS);
	}
}

#ifdef __riscv 
int main()
{
	kernel(*(const TRaXKernelArgs*)TRAX_KERNEL_ARGS_ADDRESS);
	return 0;
}

#else
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stbi/stb_image.h"
#include "stbi/stb_image_write.h"

#include <Windows.h>
int main(int argc, char* argv[])
{
	uint pregen_bounce = 0;
	std::string scene_name = argv[1];

	TRaXKernelArgs args;
	args.framebuffer_width = 1920;
	args.framebuffer_height = 1080;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	std::vector<uint32_t> fb_vec(args.framebuffer_size);
	args.framebuffer = fb_vec.data();

	args.pregen_rays = false;
	SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));
	if(scene_name.compare("sibenik") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(3.0, -13.0, 0.0), rtm::vec3(0, -12.0, 0));
	if(scene_name.compare("crytek-sponza") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	if(scene_name.compare("intel-sponza") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	if(scene_name.compare("san-miguel") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(7.448 + 0.608, 1.014 + 0.026, 12.357 - 0.794));
	if(scene_name.compare("bistro") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-8.0, 2.0, 2.0), rtm::vec3(0.0f, 1.0f, -1.0f));
	if(scene_name.compare("hairball") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(0.0, 0.0, 5.0), rtm::vec3(0.0f, 0.0f, 0.0f));

	std::string mesh_path = "../../../datasets/" + scene_name + ".obj";
	std::string bvh_cache_path = "../../../datasets/cache/" + scene_name + ".bvh";

	rtm::Mesh mesh(mesh_path);

#if 1
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
	rtm::CWBVH bvh(mesh, bvh_cache_path.c_str(), 4, false);
	args.nodes = bvh.nodes.data();

#if USE_HECWBVH
	args.ftbs = (rtm::FTB*)args.nodes;
#else
	args.ftbs = bvh.ftbs.data();
#endif

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
	{
		std::string ray_file = scene_name + "-" + std::to_string(args.framebuffer_width) + "-" + std::to_string(pregen_bounce) + ".rays";
		pregen_rays(args.nodes, args.ftbs, mesh, args.framebuffer_width, args.framebuffer_height, args.camera, pregen_bounce, rays);
		args.rays = rays.data();
	}

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);
	args.tris = tris.data();

	reset_fchthrd();
	printf("\nStarting Taveral\n");
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
	stbi_write_png("./out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	return 0;
}
#endif
