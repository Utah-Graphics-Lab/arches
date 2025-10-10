#pragma once

#include "bvh.hpp"
#include "ftb.hpp"
#include "bits.hpp"
#include "uvec3.hpp"

//Following wide BVH algorithm takes in a binary BVH as input and then collapses its notes into wide nodes in a SAH-optimal fashion,  collapsing can be further followed by compression 
//https://research.nvidia.com/publication/2017-07_efficient-incoherent-ray-traversal-gpus-through-compressed-wide-bvhs

namespace rtm {

//NVIDIA Compressed Wide Bounding Volume Hierarchy
class NVCWBVH
{
public:
	const static uint WIDTH = 12;

	struct alignas(64) Node
	{
		struct MetaData
		{
			uint8_t is_int : 1;
			uint8_t prim_cnt : 2;
			uint8_t offset : 5;
		};

		uint64_t base_child_index : 27;
		uint64_t base_prim_index : 29;
		uint64_t e0 : 8;
		vec3 p;
		uint8_t e1;
		uint8_t e2;
		MetaData mdata[WIDTH];
		QAABB8 qaabb[WIDTH];
	};

#ifndef __riscv
	std::vector<Node> nodes;
	std::vector<FTB> ftbs;

	NVCWBVH(Mesh& mesh, const char* cache_path = "")
	{
		rtm::BVH::BuildArgs args;
		args.cache_path = cache_path;
		args.width = WIDTH;
		args.max_prims = FTB::MAX_TRIS;
		args.build_method = BVH::SAH;
		args.collapse_method = BVH::DYNAMIC;
		args.leaf_cost = BVH::FTB;

		rtm::BVH bvh(mesh, args);

		printf("NVCWBVH%d: Building\n", WIDTH);

		std::vector<uint> assignments(bvh.nodes.size(), ~0u);
		assignments[0] = 0; //assign the bvh root node to the compressed root node
		nodes.emplace_back();
		for(uint i = 0; i < bvh.nodes.size(); ++i)
		{
			const BVH::Node& bvh_node = bvh.nodes[i];
			if(bvh_node.ptr.is_int)
			{
				const BVH::Node* children = &bvh.nodes[bvh_node.ptr.child_idx];
				uint* child_assignments = &assignments[bvh_node.ptr.child_idx];
				for(uint j = 0; j < bvh_node.ptr.child_cnt; ++j)
				{
					if(children[j].ptr.is_int)
					{
						child_assignments[j] = nodes.size();
						nodes.emplace_back();
					}
					else
					{
						child_assignments[j] = ftbs.size();
						ftbs.emplace_back();
					}
				}

				if(!compress(children, child_assignments, bvh_node.ptr.child_cnt, nodes[assignments[i]]))
					printf("Error could not compress %d\n", i);
			}
			else
			{
				if(!::rtm::compress(bvh_node.ptr.prim_idx, bvh_node.ptr.prim_cnt, mesh, ftbs[assignments[i]]))
					printf("Error could not compress %d\n", i);
			}
		}

		uint histo[FTB::MAX_TRIS]; uint total_tris = 0;
		for(uint i = 0; i < FTB::MAX_TRIS; ++i) histo[i] = 0;
		for(uint i = 0; i < ftbs.size(); ++i)
		{
			histo[ftbs[i].tri_cnt]++;
			total_tris += ftbs[i].tri_cnt + 1;
		}

		size_t internal_size = sizeof(Node) * nodes.size();
		size_t leaf_size = sizeof(Node) * ftbs.size();
		size_t total_size = internal_size + leaf_size;
		printf("NVCWBVH%d: Node Size:  %6.1f MiB (%4.1f B/tri)\n", WIDTH, (float)internal_size / (1 << 20), (float)internal_size / total_tris);
		printf("NVCWBVH%d: Leaf Size:  %6.1f MiB (%4.1f B/tri)\n", WIDTH, (float)leaf_size / (1 << 20), (float)leaf_size / total_tris);
		printf("NVCWBVH%d: Total Size: %6.1f MiB (%4.1f B/tri)\n", WIDTH, (float)total_size / (1 << 20), (float)total_size / total_tris);
	}

	static bool compress(const BVH::Node* nodes, const uint* indices, uint node_cnt, NVCWBVH::Node& cwnode)
	{
		AABB aabb;
		cwnode.base_child_index = ~0u, cwnode.base_prim_index = ~0u;
		for(uint i = 0; i < node_cnt; ++i)
		{
			aabb.add(nodes[i].aabb);
			if(nodes[i].ptr.is_int) cwnode.base_child_index = min(cwnode.base_child_index, indices[i]);
			else                    cwnode.base_prim_index = min(cwnode.base_prim_index, indices[i]);
		}

		constexpr float denom = 1.0f / 255;
		float32_bf bfe0((aabb.max.x - aabb.min.x) * denom);
		float32_bf bfe1((aabb.max.y - aabb.min.y) * denom);
		float32_bf bfe2((aabb.max.z - aabb.min.z) * denom);
		if(bfe0.mantisa != 0) bfe0.mantisa = 0, bfe0.exp++;
		if(bfe1.mantisa != 0) bfe1.mantisa = 0, bfe1.exp++;
		if(bfe2.mantisa != 0) bfe2.mantisa = 0, bfe2.exp++;

		cwnode.p = aabb.min;
		cwnode.e0 = bfe0.exp;
		cwnode.e1 = bfe1.exp;
		cwnode.e2 = bfe2.exp;

		uint i;
		const vec3 one_over_e(1.0f / bfe0.f32, 1.0f / bfe1.f32, 1.0f / bfe2.f32);
		for(i = 0; i < node_cnt; i++)
		{
			cwnode.mdata[i].is_int = nodes[i].ptr.is_int;
			cwnode.mdata[i].prim_cnt = nodes[i].ptr.prim_cnt;

			uint offset = indices[i];
			if(nodes[i].ptr.is_int) offset -= cwnode.base_child_index;
			else                    offset -= cwnode.base_prim_index;
			if(offset > 31) return false;
			cwnode.mdata[i].offset = offset;

			cwnode.qaabb[i].min[0] = floorf((nodes[i].aabb.min.x - cwnode.p.x) * one_over_e.x);
			cwnode.qaabb[i].min[1] = floorf((nodes[i].aabb.min.y - cwnode.p.y) * one_over_e.y);
			cwnode.qaabb[i].min[2] = floorf((nodes[i].aabb.min.z - cwnode.p.z) * one_over_e.z);
			cwnode.qaabb[i].max[0] = ceilf((nodes[i].aabb.max.x - cwnode.p.x) * one_over_e.x);
			cwnode.qaabb[i].max[1] = ceilf((nodes[i].aabb.max.y - cwnode.p.y) * one_over_e.y);
			cwnode.qaabb[i].max[2] = ceilf((nodes[i].aabb.max.z - cwnode.p.z) * one_over_e.z);
		}
		for(; i < NVCWBVH::WIDTH; ++i)
		{
			cwnode.mdata[i].is_int = 0;
			cwnode.mdata[i].prim_cnt = 0;
			cwnode.mdata[i].offset = 0;
			cwnode.qaabb[i].min[0] = 255;
			cwnode.qaabb[i].min[1] = 255;
			cwnode.qaabb[i].min[2] = 255;
			cwnode.qaabb[i].max[0] = 0;
			cwnode.qaabb[i].max[1] = 0;
			cwnode.qaabb[i].max[2] = 0;
			continue;
		}

		return true;
	}
#endif
};

inline uint decompress(const NVCWBVH::Node& cwnode, BVH::Node nodes[NVCWBVH::WIDTH])
{
	const rtm::vec3 p = cwnode.p;
	const rtm::vec3 e(float32_bf(0, cwnode.e0, 0).f32, float32_bf(0, cwnode.e1, 0).f32, float32_bf(0, cwnode.e2, 0).f32);
	for(uint i = 0; i < NVCWBVH::WIDTH; i++)
	{
		nodes[i].ptr.is_int = cwnode.mdata[i].is_int;
		nodes[i].aabb.min = vec3(cwnode.qaabb[i].min[0], cwnode.qaabb[i].min[1], cwnode.qaabb[i].min[2]) * e + p;
		nodes[i].aabb.max = vec3(cwnode.qaabb[i].max[0], cwnode.qaabb[i].max[1], cwnode.qaabb[i].max[2]) * e + p;
		if(nodes[i].ptr.is_int)
		{
			nodes[i].ptr.child_cnt = 1;
			nodes[i].ptr.child_idx = cwnode.base_child_index + cwnode.mdata[i].offset;
		}
		else
		{
			nodes[i].ptr.prim_cnt = 1;
			nodes[i].ptr.prim_idx = cwnode.base_prim_index + cwnode.mdata[i].offset;
		}
	}
	return  NVCWBVH::WIDTH;
}

}