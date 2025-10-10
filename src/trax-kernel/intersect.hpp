#pragma once
#include "stdafx.hpp"
#include "include.hpp"

template<uint32_t FLAGS>
inline void _traceray(uint id, const rtm::Ray& ray, rtm::Hit& hit)
{
#ifdef __riscv
	register float src0 asm("f0") = ray.o.x;
	register float src1 asm("f1") = ray.o.y;
	register float src2 asm("f2") = ray.o.z;
	register float src3 asm("f3") = ray.t_min;
	register float src4 asm("f4") = ray.d.x;
	register float src5 asm("f5") = ray.d.y;
	register float src6 asm("f6") = ray.d.z;
	register float src7 asm("f7") = ray.t_max;

	register float dst0 asm("f28");
	register float dst1 asm("f29");
	register float dst2 asm("f30");
	register float dst3 asm("f31");

	asm volatile
	(
		"traceray %0, %4, %12\t\n"
		: "=f" (dst0), "=f" (dst1), "=f" (dst2), "=f" (dst3)
		: "f" (src0), "f" (src1), "f" (src2), "f" (src3), "f" (src4), "f" (src5), "f" (src6), "f" (src7), "I" (FLAGS) 
	);

	float _dst3 = dst3;

	hit.t = dst0;
	hit.bc.x = dst1;
	hit.bc.y = dst2;
	hit.id = *(uint*)&_dst3;
#else
	assert(false);
#endif
}

inline float _intersect(const rtm::AABB& aabb, const rtm::Ray& ray, const rtm::vec3& inv_d)
{
#if defined(__riscv) && defined(DS_USE_HARDWARE_INTERSECTORS)
	register float src0 asm("f0") = ray.o.x;
	register float src1 asm("f1") = ray.o.y;
	register float src2 asm("f2") = ray.o.z;
	register float src3 asm("f3") = ray.t_min;
	register float src4 asm("f4") = inv_d.x;
	register float src5 asm("f5") = inv_d.y;
	register float src6 asm("f6") = inv_d.z;
	register float src7 asm("f7") = ray.t_max;

	register float src8 asm("f8") = aabb.min.x;
	register float src9 asm("f9") = aabb.min.y;
	register float src10 asm("f10") = aabb.min.z;
	register float src11 asm("f11") = aabb.max.x;
	register float src12 asm("f12") = aabb.max.y;
	register float src13 asm("f13") = aabb.max.z;

	float t;
	asm volatile ("boxisect %0" : "=f" (t) : "f" (src0), "f" (src1), "f" (src2), "f" (src3), "f" (src4), "f" (src5), "f" (src6), "f" (src7), "f" (src8), "f" (src9), "f" (src10), "f" (src11), "f" (src12), "f" (src13));

	return t;
#else
	return rtm::intersect(aabb, ray, inv_d);
#endif
}

inline bool _intersect(const rtm::Triangle& tri, const rtm::Ray& ray, rtm::Hit& hit)
{
#if defined(__riscv) && defined(DS_USE_HARDWARE_INTERSECTORS)
	register float src0 asm("f0") = ray.o.x;
	register float src1 asm("f1") = ray.o.y;
	register float src2 asm("f2") = ray.o.z;
	register float src3 asm("f3") = ray.t_min;
	register float src4 asm("f4") = ray.d.x;
	register float src5 asm("f5") = ray.d.y;
	register float src6 asm("f6") = ray.d.z;
	register float src7 asm("f7") = ray.t_max;

	register float src8 asm("f8") = tri.vrts[0].x;
	register float src9 asm("f9") = tri.vrts[0].y;
	register float src10 asm("f10") = tri.vrts[0].z;
	register float src11 asm("f11") = tri.vrts[1].x;
	register float src12 asm("f12") = tri.vrts[1].y;
	register float src13 asm("f13") = tri.vrts[1].z;
	register float src14 asm("f14") = tri.vrts[2].x;
	register float src15 asm("f15") = tri.vrts[2].y;
	register float src16 asm("f16") = tri.vrts[2].z;

	register float dst0 asm("f17") = hit.t;
	register float dst1 asm("f18") = hit.bc.x;
	register float dst2 asm("f19") = hit.bc.y;
	register float dst3 asm("f20") = *(float*)&hit.id;

	asm volatile("triisect %0\n\t" : "+f" (dst0), "+f" (dst1), "+f" (dst2), "+f" (dst3) : "f" (src0), "f" (src1), "f" (src2), "f" (src3), "f" (src4), "f" (src5), "f" (src6), "f" (src7), "f" (src8), "f" (src9), "f" (src10), "f" (src11), "f" (src12), "f" (src13), "f" (src14), "f" (src15), "f" (src16));

	bool is_hit = dst0 < hit.t;
	float _dst3 = dst3;

	hit.t = dst0;
	hit.bc.x = dst1;
	hit.bc.y = dst2;
	hit.id = *(uint*)&_dst3;

	return is_hit;
#else
	return rtm::intersect(tri, ray, hit);
#endif
}

#ifndef __riscv
inline bool intersect(const rtm::BVH& bvh, const rtm::Mesh& mesh, const rtm::Ray& ray, rtm::Hit& hit)
{
	rtm::vec3 rcp_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		rtm::BVHPtr ptr;
	};

	NodeStackEntry node_stack[64];
	uint32_t node_stack_size = 1u;
	node_stack[0].t = ray.t_min;
	node_stack[0].ptr.is_int = 1;
	node_stack[0].ptr.child_cnt = 1;
	node_stack[0].ptr.child_idx = 0;
	
	bool found_hit = false;
	do
	{
		NodeStackEntry entry = node_stack[--node_stack_size];
		if(entry.t >= hit.t) continue;

		if(entry.ptr.is_int)
		{
			uint max_insert_depth = node_stack_size;
			for(uint32_t i = 0; i < entry.ptr.child_cnt; i++)
			{
				uint32_t node_id = entry.ptr.child_idx + i;
				float t = _intersect(bvh.nodes[node_id].aabb, ray, rcp_d);

				if(t < hit.t)
				{
					uint j = node_stack_size++;
					for(; j > max_insert_depth; --j)
					{
						if(node_stack[j - 1].t > t) break;
						node_stack[j] = node_stack[j - 1];
					}

					node_stack[j].t = t;
					node_stack[j].ptr = bvh.nodes[node_id].ptr;
				}
			}
		}
		else
		{
			for(uint32_t i = 0; i < entry.ptr.prim_cnt; ++i)
			{
				uint32_t prim_id = entry.ptr.prim_idx + i;
				if(_intersect(mesh.get_triangle(prim_id), ray, hit))
					hit.id = prim_id;
			}
		}
	} while(node_stack_size);

	return found_hit;
}
#endif

struct IntersectStats
{
	uint node_steps = 0;
	uint prim_steps = 0;
	uint total_steps() const { return node_steps + prim_steps; }
};

template <typename N, typename P>
inline bool intersect(const N* nodes, const P* prims, const rtm::Ray& ray, rtm::Hit& hit, IntersectStats& stats)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		rtm::BVHPtr ptr;
	};

	NodeStackEntry node_stack[32 * 16];
	uint32_t node_stack_size = 1u;

	//Decompress and insert nodes
	node_stack[0].t = ray.t_min;
	node_stack[0].ptr.is_int = 1;
	node_stack[0].ptr.child_idx = 0;

	bool found_hit = false;
	do
	{
		NodeStackEntry entry = node_stack[--node_stack_size];
		if(entry.t >= hit.t) continue;

		if(entry.ptr.is_int)
		{
			rtm::BVH::Node bvh_nodes[16];
			uint node_count = rtm::decompress(nodes[entry.ptr.child_idx], bvh_nodes);

			uint max_insert_depth = node_stack_size, last_ptr = ~0u;
			for(uint i = 0; i < node_count; ++i)
			{
				if(bvh_nodes[i].ptr.raw == last_ptr) continue;

				float t = _intersect(bvh_nodes[i].aabb, ray, inv_d);
				if(t < hit.t)
				{
					uint j = node_stack_size++;
					for(; j > max_insert_depth; --j)
					{
						if(node_stack[j - 1].t > t) break;
						node_stack[j] = node_stack[j - 1];
					}
					node_stack[j].t = t;
					node_stack[j].ptr = bvh_nodes[i].ptr;
					last_ptr = bvh_nodes[i].ptr.raw;
				}
			}
			stats.node_steps++;
		}
		else
		{
		#if 1
			rtm::IntersectionTriangle tris[rtm::FTB::MAX_TRIS];
			uint tri_count = rtm::decompress(prims[entry.ptr.prim_idx], tris);
			for(uint i = 0; i < tri_count; ++i)
				if(_intersect(tris[i].tri, ray, hit))
				{
					hit.id = tris[i].id;
					found_hit = true;
				}
		#else
			if(entry.t < hit.t)
			{
				hit.id = entry.ptr.prim_idx;
				hit.t = entry.t;
				found_hit = true;
			}
		#endif
			stats.prim_steps++;
		}
	}
	while(node_stack_size);

	return found_hit;
}

inline static uint32_t morton1(uint32_t x)
{
	x = x & 0x55555555;
	x = (x | (x >> 1)) & 0x33333333;
	x = (x | (x >> 2)) & 0x0F0F0F0F;
	x = (x | (x >> 4)) & 0x00FF00FF;
	x = (x | (x >> 8)) & 0x0000FFFF;
	return x;
}

inline void deinterleave_bits(uint i, uint& x, uint& y)
{
	x = morton1(i);
	y = morton1(i >> 1);
}

#ifndef __riscv 
template <typename N, typename P>
inline void pregen_rays(const N* nodes, const P* prims, rtm::Mesh& mesh, uint framebuffer_width, uint framebuffer_height, const rtm::Camera& camera, uint bounce, std::vector<rtm::Ray>& rays, std::string ray_file = "")
{
	const uint framebuffer_size = framebuffer_width * framebuffer_height;
	printf("Generating bounce %d rays from %d path\n", bounce, framebuffer_size);

	uint num_rays = framebuffer_size;
	for(int index = 0; index < framebuffer_size; index++)
	{
		uint32_t x = index % framebuffer_width;
		uint32_t y = index / framebuffer_width;
		rtm::RNG rng(index);

		rtm::Ray ray = camera.generate_ray_through_pixel(x, y); // Assuming spp = 1
	
		for(uint i = 0; i < bounce; ++i)
		{
			uint steps = 0;
			rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);

			IntersectStats stats;
			if(intersect(nodes, prims, ray, hit, stats))
			{
				rtm::vec3 normal = mesh.get_triangle(hit.id).normal();
				ray.o += ray.d * hit.t;
				ray.d = cosine_sample_hemisphere(normal, rng); // generate secondray rays
			}
			else
			{
				num_rays--;
				ray.t_max = ray.t_min;
				break;
			}
		}
		rays[index] = ray;
	}
	printf("Generated %d rays\n", num_rays);

	if (ray_file.compare("") != 0)
	{
		std::string resultPathName = std::filesystem::current_path().generic_string() + "/" + ray_file;
		std::ofstream file_stream(resultPathName.c_str() , std::ios::binary);
		
		if (file_stream.good())
		{
			file_stream.write((char*)rays.data(), rays.size() * sizeof(rtm::Ray));
			if (file_stream.good())
			{
				printf("Serialized ray data: %s\n", resultPathName.c_str());
			}
		}
		else
		{
			fprintf(stderr, "pregen_rays: ofstream failed to open output file\n");
		}

		file_stream.close();
	}
}

#endif
