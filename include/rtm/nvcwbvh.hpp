#pragma once

#include "bvh.hpp"
#include "ftb.hpp"
#include "bits.hpp"
#include "uvec3.hpp"

namespace rtm {

//Compressed Wide Bounding Volume Hierarchy
class HE2CWBVH
{
public:
	const static uint WIDTH = 7;

	struct alignas(64) Node
	{
		uint32_t base_child_index;
		uint32_t base_prim_index;
		struct
		{
			uint32_t exp    : 8;
			uint32_t anchor : 24;
		}exts[3];
		uint16_t i_mask : WIDTH;
		uint16_t o_mask : WIDTH;
		QAABB8 qaabb[WIDTH];
	};

#ifndef __riscv
	std::vector<Node> nodes;
	std::vector<FTB> ftbs;

	HE2CWBVH(Mesh& mesh, const char* cache_path = "", uint preset = 0, bool merge = false)
	{
		sizeof(Node);
		rtm::BVH::BuildArgs args;
		args.cache_path = cache_path;
		args.width = WIDTH;
		args.build_method = BVH::SAH;

		if(merge)
		{
			args.max_prims_merge = BVH::FTB;
			args.merge_nodes = true;
			args.merge_leafs = true;
		}

		if(preset == 0)
		{
			args.leaf_cost = BVH::LINEAR;
			args.max_prims_collapse = 3;
			args.collapse_method = BVH::DYNAMIC;
		}

		if(preset == 1)
		{
			args.leaf_cost = 2;
			args.max_prims_collapse = 3;
			args.collapse_method = BVH::DYNAMIC;
		}

		if(preset == 2)
		{
			args.leaf_cost = 2;
			args.max_prims_collapse = BVH::FTB;
			args.collapse_method = BVH::DYNAMIC;
		}

		if(preset == 3)
		{
			args.leaf_cost = 2;
			args.max_prims_collapse = BVH::FTB;
			args.collapse_method = BVH::GREEDY;
			args.merge_leafs = false;
		}

		if(preset == 4)
		{
			args.leaf_cost = BVH::LINEAR;
			args.max_prims_collapse = 1;
			args.collapse_method = BVH::DYNAMIC;
		}

		rtm::BVH bvh(mesh, args);

		printf("NVCWBVH%d: Building\n", WIDTH);

		std::vector<uint> assignments(bvh.nodes.size(), ~0u);
		assignments[0] = 0; //assign the bvh root node to the compressed root node
		nodes.emplace_back();

		uint last_ptr = ~0u;
		for(uint i = 0; i < bvh.nodes.size(); ++i)
		{
			const BVH::Node& bvh_node = bvh.nodes[i];
			if(bvh_node.ptr.raw == last_ptr) continue;
			last_ptr = bvh_node.ptr.raw;

			if(bvh_node.ptr.is_int)
			{
				const BVH::Node* children = &bvh.nodes[bvh_node.ptr.child_idx];
				uint* child_assignments = &assignments[bvh_node.ptr.child_idx];

				uint last_child_ptr = ~0u;
				for(uint j = 0; j < bvh_node.ptr.child_cnt; ++j)
				{
					if(children[j].ptr.raw == last_child_ptr)
					{
						child_assignments[j] = child_assignments[j - 1];
						continue;
					}

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

					last_child_ptr = children[j].ptr.raw;
				}

				if(!compress(children, child_assignments, bvh_node.ptr.child_cnt, nodes[assignments[i]]))
					printf("Error could not compress %d\n", i);
			}
			else
			{
				if(!::rtm::compress(bvh_node.ptr.prim_idx, bvh_node.ptr.prim_cnt, mesh, &ftbs[assignments[i]]))
					printf("Error could not compress leaf %d\n", i);
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
		size_t leaf_size = sizeof(FTB) * ftbs.size();
		size_t total_size = internal_size + leaf_size;
		printf("NVCWBVH%d: Node Size:  %6.1f MiB (%4.1f B/tri)\n", WIDTH, (float)internal_size / (1 << 20), (float)internal_size / total_tris);
		printf("NVCWBVH%d: Leaf Size:  %6.1f MiB (%4.1f B/tri)\n", WIDTH, (float)leaf_size / (1 << 20), (float)leaf_size / total_tris);
		printf("NVCWBVH%d: Total Size: %6.1f MiB (%4.1f B/tri)\n", WIDTH, (float)total_size / (1 << 20), (float)total_size / total_tris);
	}

	static bool compress(const BVH::Node* nodes, const uint* indices, uint node_cnt, HE2CWBVH::Node& cwnode)
	{
		AABB aabb;
		cwnode.base_child_index = ~0u, cwnode.base_prim_index = ~0u;
		for(uint i = 0; i < node_cnt; ++i)
		{
			aabb.add(nodes[i].aabb);
			if(nodes[i].ptr.is_int) cwnode.base_child_index = min(cwnode.base_child_index, indices[i]);
			else                    cwnode.base_prim_index = min(cwnode.base_prim_index, indices[i]);
		}

		rtm::vec3 p = rtm::vec3(f32_to_bf24(aabb.min.x), f32_to_bf24(aabb.min.y), f32_to_bf24(aabb.min.z));

		constexpr float denom = 1.0f / 255;
		float32_bf bfe0((aabb.max.x - p.x) * denom);
		float32_bf bfe1((aabb.max.y - p.y) * denom);
		float32_bf bfe2((aabb.max.z - p.z) * denom);
		if(bfe0.mantisa != 0) bfe0.mantisa = 0, bfe0.exp++;
		if(bfe1.mantisa != 0) bfe1.mantisa = 0, bfe1.exp++;
		if(bfe2.mantisa != 0) bfe2.mantisa = 0, bfe2.exp++;

		cwnode.exts[0].anchor = as_u32(p[0]) >> 8;
		cwnode.exts[1].anchor = as_u32(p[1]) >> 8;
		cwnode.exts[2].anchor = as_u32(p[2]) >> 8;
		cwnode.exts[0].exp = bfe0.exp;
		cwnode.exts[1].exp = bfe1.exp;
		cwnode.exts[2].exp = bfe2.exp;

		cwnode.i_mask = 0;
		cwnode.o_mask = 0;

		uint i = 0, last_ptr = ~0u;
		const vec3 one_over_e(1.0f / bfe0.f32, 1.0f / bfe1.f32, 1.0f / bfe2.f32);
		for(i = 0; i < node_cnt; i++)
		{
			cwnode.i_mask |= nodes[i].ptr.is_int << i;
			if(indices[i] != last_ptr)
			{
				cwnode.o_mask |= 1 << i;
				last_ptr = indices[i];
			}

			cwnode.qaabb[i].min[0] = floorf((nodes[i].aabb.min.x - p.x) * one_over_e.x);
			cwnode.qaabb[i].min[1] = floorf((nodes[i].aabb.min.y - p.y) * one_over_e.y);
			cwnode.qaabb[i].min[2] = floorf((nodes[i].aabb.min.z - p.z) * one_over_e.z);
			cwnode.qaabb[i].max[0] = ceilf((nodes[i].aabb.max.x - p.x) * one_over_e.x);
			cwnode.qaabb[i].max[1] = ceilf((nodes[i].aabb.max.y - p.y) * one_over_e.y);
			cwnode.qaabb[i].max[2] = ceilf((nodes[i].aabb.max.z - p.z) * one_over_e.z);
		}
		for(; i < HE2CWBVH::WIDTH; ++i)
		{
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

inline uint decompress(const HE2CWBVH::Node& cwnode, BVH::Node nodes[HE2CWBVH::WIDTH])
{
	const rtm::vec3 p(as_f32(cwnode.exts[0].anchor << 8), as_f32(cwnode.exts[1].anchor << 8), as_f32(cwnode.exts[2].anchor << 8));
	const rtm::vec3 e(as_f32(cwnode.exts[0].exp << 23), as_f32(cwnode.exts[1].exp << 23), as_f32(cwnode.exts[2].exp << 23));

	uint offset0 = -1;
	uint offset1 = -1;
	for(uint i = 0; i < HE2CWBVH::WIDTH; i++)
	{
		nodes[i].ptr.is_int = (cwnode.i_mask >> i) & 0x1;
		nodes[i].aabb.min = vec3(cwnode.qaabb[i].min[0], cwnode.qaabb[i].min[1], cwnode.qaabb[i].min[2]) * e + p;
		nodes[i].aabb.max = vec3(cwnode.qaabb[i].max[0], cwnode.qaabb[i].max[1], cwnode.qaabb[i].max[2]) * e + p;
		if(nodes[i].ptr.is_int)
		{
			if((cwnode.o_mask >> i) & 0x1) offset0++;
			nodes[i].ptr.child_cnt = 1;
			nodes[i].ptr.child_idx = cwnode.base_child_index + offset0;
		}
		else
		{
			if((cwnode.o_mask >> i) & 0x1) offset1++;
			nodes[i].ptr.prim_cnt = 1;
			nodes[i].ptr.prim_idx = cwnode.base_prim_index + offset1;
		}
	}
	return  HE2CWBVH::WIDTH;
}

}