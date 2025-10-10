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
	const float kr[] = {000.13572138f, 004.61539260f,-042.66032258f, 132.13108234f,-152.94239396f, 059.28637943f};
	const float kg[] = {000.09140261f, 002.19418839f, 004.84296658f,-014.18503333f, 004.27729857f, 002.82956604f};
	const float kb[] = {000.10667330f, 012.64194608f,-060.58204836f, 110.36276771f,-089.90310912f, 027.34824973f};

	float t_pow = t = rtm::clamp(t, 0.0f, 1.0f);
	rtm::vec3 color(kr[0], kg[0], kb[0]);
	for(uint i = 1; i < 6; ++i)
	{
		color.r += kr[i] * t_pow;
		color.g += kg[i] * t_pow;
		color.b += kb[i] * t_pow;
		t_pow *= t;
	}

	return rtm::clamp(color);
}

#ifndef  __riscv
static std::atomic_uint node_steps = 0;
static std::atomic_uint prim_steps = 0;
#endif

inline static void kernel(const TRaXKernelArgs& args)
{
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
		rtm::Ray ray = args.pregen_rays ? args.rays[fb_index] : args.camera.generate_ray_through_pixel(x, y);
		rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);

		IntersectStats stats;
		#if defined(__riscv) && (TRAX_USE_RT_CORE)
			_traceray<0x0u>(index, ray, hit);
		#else
			intersect(args.nodes, args.ftbs, ray, hit, stats);
		#endif

		if(hit.t < ray.t_max)
		{
			//rtm::vec3 n = args.tris[hit.id].normal();
			uint hash = rtm::RNG::hash(hit.id) | 0xff'00'00'00;

			args.framebuffer[fb_index] = hash;
			//args.framebuffer[fb_index] = encode_pixel(n * 0.5f + 0.5f);
		}
		else
		{
			args.framebuffer[fb_index] = 0xff000000;
		}

		//args.framebuffer[fb_index] = encode_pixel(palette(rtm::clamp(stats.total_steps() / 64.0f)));
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

int main(int argc, char* argv[])
{
	uint pregen_bounce = 1;
	std::string scene_name = "intel-sponza";

	TRaXKernelArgs args;
	args.framebuffer_width = 1024;
	args.framebuffer_height = 1024;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	args.framebuffer = new uint32_t[args.framebuffer_size];

	args.pregen_rays = true;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));
	if(scene_name.compare("sibenik") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(3.0, -13.0, 0.0), rtm::vec3(0, -12.0, 0));
	if(scene_name.compare("crytek-sponza") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	if(scene_name.compare("intel-sponza") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	if(scene_name.compare("san-miguel") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(7.448 + 0.608, 1.014 + 0.026, 12.357 - 0.794));

	std::string mesh_path = "../../../datasets/" + scene_name + ".obj";
	std::string bvh_cache_path = "../../../datasets/cache/" + scene_name + ".bvh";

	rtm::Mesh mesh(mesh_path);
	rtm::CWBVH bvh(mesh, bvh_cache_path.c_str());

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
	thread_count = std::max(std::thread::hardware_concurrency() - 1u, 1u);
	for (uint i = 1; i < thread_count; ++i) threads.emplace_back(kernel, args);
	kernel(args);
	for (uint i = 1; i < thread_count; ++i) threads[i - 1].join();

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	printf("Runtime: %dms\n", (uint)duration.count());
	printf("Node steps: %5.2f\n", (float)node_steps.load() / args.framebuffer_size);
	printf("Prim steps: %5.2f\n", (float)prim_steps.load() / args.framebuffer_size);
	printf("Memory Traffic: %.1f B/ray\n", (float)(node_steps.load() + prim_steps.load()) * 128 / args.framebuffer_size);

	stbi_flip_vertically_on_write(true);
	stbi_write_png("./out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	delete[] args.framebuffer;
	return 0;
}
#endif
