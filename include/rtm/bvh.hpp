#pragma once

#include "int.hpp"
#include "aabb.hpp"
#include "mesh.hpp"
#include "ftb.hpp"

#ifndef __riscv
#include <vector>
#include <deque>
#include <algorithm>
#include <cassert>
#include <fstream>
#endif

namespace rtm {

union BVHPtr
{
	struct
	{
		uint32_t is_int   : 1;
		uint32_t prim_cnt : 5;
		uint32_t prim_idx : 26;
	};
	struct
	{
		uint32_t           : 1;
		uint32_t child_cnt : 5;
		uint32_t child_idx : 26;
	};
	uint32_t raw;
};

class BVH
{
public:
	const static uint32_t VERSION = 51;

	enum BuildMethod
	{
		LBVH,
		//HPLOC, //TODO
		//SBVH,  //TODO
		SAH_32BIN,
		SAH,
	};

	enum CollapseMethod
	{
		GREEDY,
		DYNAMIC,
	};

	enum LeafCost
	{
		LINEAR,
		CONSTANT,
	};

	enum MaxPrims : uint8_t
	{
		PAIR = 0x20 + 2,
		FTB = 0x40 + FTB::MAX_TRIS,
	};


	struct BuildArgs
	{
		const char* cache_path{""};
		uint8_t width{2};
		uint8_t max_prims_collapse{1};
		uint8_t max_prims_merge{1};
		uint8_t build_method{SAH};
		uint8_t collapse_method{DYNAMIC};
		uint8_t leaf_cost{CONSTANT};
		bool merge_nodes{false};
		bool merge_leafs{false};
		bool silent{false};
	};

	struct alignas(32) Node
	{
		AABB aabb;
		BVHPtr ptr;
	};

#ifndef __riscv
	BuildArgs args{};
	float sah_cost{0.0f};
	std::vector<Node> nodes;

	float node_collapse_time{0.0};
	float leaf_collapse_time{0.0};
	float leaf_merge_time{0.0};
	float node_merge_time{0.0};

private:
	struct BuildObject
	{
		AABB  aabb{};
		float_t cost{0.0f};
		uint32_t index{~0u};
		uint64_t morton_code{~0u};
	};

	struct BuildEvent
	{
		uint32_t start;
		uint32_t end;
		uint32_t node_index;
	};

	struct FileHeader
	{
		uint32_t version;
		uint32_t build_method;
		uint32_t num_nodes;
		uint32_t num_build_objects;
	};

	const Mesh* _mesh{nullptr}; //used only for FTB leaf cost 
	uint8_t _max_prims{1};

public:
	BVH() = default;

	BVH(Mesh& mesh, BuildArgs args) : args(args)
	{
		std::vector<BuildObject> bld_objs;
		bld_objs.resize(mesh.size());

		bool build = true;
		if(args.cache_path != "")
		{
			if(!args.silent) printf("BVH2: Loading: %s\n", args.cache_path);
			build = !_deserialize(args.cache_path, bld_objs);
		}

		if(build)
		{
			if(!args.silent) printf("BVH2: Failed to load: %s\n", args.cache_path);
			if(!args.silent) printf("BVH2: Building\n");

			_get_build_objs(mesh, bld_objs);
			_build(bld_objs);
			if(args.cache_path != "")
				_serialize(args.cache_path, bld_objs);
		}

		_reorder(mesh, bld_objs);
		_mesh = &mesh; //gross

		sah_cost = _compute_cost(bld_objs);
		if(!args.silent) _print_stats_bvh2();

		uint total_time = 0;

		if(args.max_prims_collapse > 1 && args.collapse_method == GREEDY)
		{
			_max_prims = args.max_prims_collapse;
			if(!args.silent) printf("BVH%d: Collapsing Leafs: ", args.width);
			auto start = std::chrono::high_resolution_clock::now();
			_collapse_leafs(bld_objs);

			auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
			if(!args.silent) printf("%dms\n", (uint)time.count());
			total_time += time.count();
			leaf_collapse_time = time.count();
		}

		if(args.width > 2)
		{
			_max_prims = args.max_prims_collapse;
			if(!args.silent) printf("BVH%d: Collapsing Nodes: ", args.width);
			auto start = std::chrono::high_resolution_clock::now();
			_collapse_nodes(bld_objs);

			auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
			if(!args.silent) printf("%dms\n", (uint)time.count());
			total_time += time.count();
			node_collapse_time = time.count();
		}

		if(args.merge_nodes)
		{
			if(!args.silent) printf("BVH%d: Merging Nodes: ", args.width);
			auto start = std::chrono::high_resolution_clock::now();
			_merge_nodes();

			auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
			if(!args.silent) printf("%dms\n", (uint)time.count());
			total_time += time.count();
			node_merge_time = time.count();
		}

		if(args.merge_leafs)
		{
			_max_prims = args.max_prims_merge;
			if(!args.silent) printf("BVH%d: Merging Leafs: ", args.width);
			auto start = std::chrono::high_resolution_clock::now();
			_merge_leafs(bld_objs);

			auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
			if(!args.silent) printf("%dms\n", (uint)time.count());
			total_time += time.count();
			leaf_merge_time = time.count();
		}

		if(!args.silent) printf("BVH%d: Total Time: %dms\n", args.width, total_time);

		sah_cost = _compute_cost(bld_objs);
		if(!args.silent) _print_stats_wbvh();
	}

private:
	static void _get_build_objs(const Mesh& mesh, std::vector<BuildObject>& bld_objs)
	{
		bld_objs.resize(mesh.size());
		for(uint32_t i = 0; i < mesh.size(); ++i)
		{
			Triangle tri = mesh.get_triangle(i);
			bld_objs[i].aabb = tri.aabb();
			bld_objs[i].cost = 1.0f;
			bld_objs[i].index = i;
		}
	}

	static void _compute_morton_codes(std::vector<BuildObject>& bld_objs)
	{
		AABB aabb;
		for(auto& build_object : bld_objs)
			aabb.add(build_object.aabb);

		uint max = (1ull << 21) - 1;
		rtm::vec3 scale = 1.0f / (aabb.max - aabb.min);
		for(auto& build_object : bld_objs)
		{
			rtm::vec3 centroid(build_object.aabb.centroid() - aabb.min);
			centroid = centroid * scale * max;

			build_object.morton_code = 
				_pdep_u64((uint32_t)centroid.x, 0b0001001001001001001001001001001001001001001001001001001001001001ull) |
				_pdep_u64((uint32_t)centroid.y, 0b0010010010010010010010010010010010010010010010010010010010010010ull) |
				_pdep_u64((uint32_t)centroid.z, 0b0100100100100100100100100100100100100100100100100100100100100100ull);
		}
	}

	static void _reorder(Mesh& mesh, std::vector<BuildObject>& bld_objs)
	{
		std::vector<rtm::uvec3> tmp_vrt_inds(mesh.vertex_indices);
		std::vector<rtm::uvec3> tmp_nrml_inds(mesh.normal_indices);
		std::vector<rtm::uvec3> tmp_txcd_inds(mesh.tex_coord_indices);
		std::vector<uint>       tmp_mat_inds(mesh.material_indices);
		for(uint32_t i = 0; i < bld_objs.size(); ++i)
		{
			mesh.vertex_indices[i] = tmp_vrt_inds[bld_objs[i].index];
			mesh.normal_indices[i] = tmp_nrml_inds[bld_objs[i].index];
			mesh.tex_coord_indices[i] = tmp_txcd_inds[bld_objs[i].index];
			mesh.material_indices[i] = tmp_mat_inds[bld_objs[i].index];
			bld_objs[i].index = i;
		}
	}

	void _refit(const std::vector<BuildObject>& bld_objs)
	{
		for(int i = nodes.size() - 1; i >= 0; --i)
		{
			BVH::Node& node = nodes[i];
			node.aabb = AABB();
			if(node.ptr.is_int)
			{
				for(uint j = 0; j < nodes[i].ptr.child_cnt; ++j)
					node.aabb.add(nodes[node.ptr.child_idx + j].aabb);
			}
			else
			{
				for(uint j = 0; j < node.ptr.prim_cnt; ++j)
					node.aabb.add(bld_objs[node.ptr.prim_idx + j].aabb);
			}
		}
	}

	uint _split(std::vector<BuildObject>& bld_objs, uint start, uint end) const
	{
		uint size = end - start;
		if(size <= 1) return ~0u;

		if(args.build_method == LBVH) return _split_radix(bld_objs, start, end);
		else if(args.build_method == SAH_32BIN) return _split_sah_binned(bld_objs, start, end, 32);
		else if(args.build_method == SAH) return _split_sah(bld_objs, start, end);
		else return _split_sah(bld_objs, start, end);
	}

	uint _split_radix(std::vector<BuildObject>& bld_objs, uint start, uint end) const
	{
		uint64_t mask = 0x0ull;
		for(uint i = start; i < end; ++i)
			mask |= bld_objs[i].morton_code ^ bld_objs[start].morton_code;

		uint64_t prefix = _lzcnt_u64(mask);
		if(prefix == 64)
			return (start + end) / 2;

		uint i = start, j = end;
		uint64_t sort_key = 1ull << (63 - prefix);
		while(i < j)
		{
			const BuildObject& obj = bld_objs[i];
			if(obj.morton_code & sort_key) 
			{
				j--;
				std::swap(bld_objs[i], bld_objs[j]);
			}
			else i++;
		}

		return i;
	}

	uint _split_on_plane(std::vector<BuildObject>& bld_objs, uint start, uint end, uint split_axis, float split_pos, float& cost) const
	{
		AABB aabb_left, aabb_right;
		float cost_left = 0.0f, cost_right = 0.0f;
		uint i = start, j = end;
		while(i < j)
		{
			const BuildObject& obj = bld_objs[i];
			float center = obj.aabb.centroid()[split_axis];
			if(center <= split_pos)
			{
				i++;
				cost_left += obj.cost;
				aabb_left.add(obj.aabb);
			}
			else
			{
				j--;
				std::swap(bld_objs[i], bld_objs[j]);
				cost_right += obj.cost;
				aabb_right.add(obj.aabb);
			}
		}

		uint count_left = i - start;
		uint count_right = end - i;
		if(count_left == 0 || count_right == 0)
		{
			cost = INFINITY;
			return (start + end) / 2;
		}

		cost = cost_left  * aabb_left.surface_area() +  cost_right * aabb_right.surface_area();
		return i;
	}

	uint _split_sah_binned(std::vector<BuildObject>& bld_objs, uint start, uint end, uint bins = 32) const
	{
		AABB aabb;
		for(uint i = start; i < end; ++i)
			aabb.add(bld_objs[i].aabb);

		float best_sah = INFINITY;
		uint best_split_axis = 0;
		float best_split_pos = 0.0f;

		for(uint split_axis = 0; split_axis < 3; ++split_axis)
			for(uint b = 1; b < bins; ++b)
			{
				float split_ratio = (float)b / bins;
				float split_pos = (1.0f - split_ratio) * aabb.min[split_axis] + split_ratio * aabb.max[split_axis];
				float sah; _split_on_plane(bld_objs, start, end, split_axis, split_pos, sah);
				if(sah < best_sah)
				{
					best_split_axis = split_axis;
					best_sah = sah;
					best_split_pos = split_pos;
				}
			}

		if(best_sah == INFINITY)
			return (start + end) / 2;

		return _split_on_plane(bld_objs, start, end, best_split_axis, best_split_pos, best_sah);
	}

	uint _split_sah(std::vector<BuildObject>& bld_objs, uint start, uint end) const
	{
		uint size = end - start;
		if(size <= 1) return ~0u;

		AABB aabb;
		for(uint i = start; i < end; ++i)
			aabb.add(bld_objs[i].aabb);

		float best_sah = INFINITY;
		uint best_split_axis = 0;
		uint best_split_index = 0;

		std::vector<float> cost_left(size);
		std::vector<float> cost_right(size);
		for(uint split_axis = 0; split_axis < 3; ++split_axis)
		{
			std::sort(bld_objs.begin() + start, bld_objs.begin() + end, [&](const BuildObject& a, const BuildObject& b)
			{
				return a.aabb.centroid()[split_axis] < b.aabb.centroid()[split_axis];
			});

			const BuildObject* objs = bld_objs.data() + start;

			AABB left_aabb; uint left_cost_sum = 0.0f;
			for(uint i = 0; i < size; ++i)
			{
				cost_left[i] = left_cost_sum * left_aabb.surface_area();
				left_aabb.add(objs[i].aabb);
				left_cost_sum += objs[i].cost;
			}
			cost_left[0] = 0.0f;

			AABB right_aabb; uint right_cost_sum = 0.0f;
			for(int i = size - 1; i >= 0; --i)
			{
				right_cost_sum += objs[i].cost;
				right_aabb.add(objs[i].aabb);
				cost_right[i] = right_cost_sum * right_aabb.surface_area();
			}

			for(uint i = 1; i < size; ++i)
			{
				float cost = cost_left[i] + cost_right[i];
				if(cost < best_sah)
				{
					best_sah = cost;
					best_split_axis = split_axis;
					best_split_index = i;
				}
			}
		}

		std::sort(bld_objs.begin() + start, bld_objs.begin() + end, [&](const BuildObject& a, const BuildObject& b)
		{
			return a.aabb.centroid()[best_split_axis] < b.aabb.centroid()[best_split_axis];
		});

		if(best_sah == INFINITY)
			return (start + end) / 2;

		return start + best_split_index;
	}

	void _build(std::vector<BuildObject>& bld_objs)
	{
		if(args.build_method == LBVH) _compute_morton_codes(bld_objs);

		std::deque<BuildEvent> event_queue;
		event_queue.emplace_back(0, bld_objs.size(), 0);
		
		nodes.reserve(bld_objs.size() * 2 - 1);
		nodes.emplace_back();

		while(!event_queue.empty())
		{
			BuildEvent event = event_queue.front(); event_queue.pop_front();

			uint splitting_index = _split(bld_objs, event.start, event.end);
			if(splitting_index != ~0)
			{
				nodes[event.node_index].ptr.is_int = 1;
				nodes[event.node_index].ptr.child_cnt = 2;
				nodes[event.node_index].ptr.child_idx = nodes.size();

				event_queue.emplace_back(event.start, splitting_index, (uint)nodes.size() + 0);
				event_queue.emplace_back(splitting_index,   event.end, (uint)nodes.size() + 1);

				for(uint i = 0; i < 2; ++i)
					nodes.emplace_back();
			}
			else
			{
				//didn't do any splitting meaning this build event can become a leaf node
				uint size = event.end - event.start;
				nodes[event.node_index].ptr.is_int = 0;
				nodes[event.node_index].ptr.prim_cnt = size;
				nodes[event.node_index].ptr.prim_idx = event.start;
			}
		}

		_refit(bld_objs);
	}

	float _cost_leaf(const std::vector<BuildObject>& bld_objs, uint prim_id0, uint prim_cnt)
	{
		float cost_leaf = INFINITY;
		if(prim_cnt <= (_max_prims & 0x1f))
		{
			if(_max_prims == PAIR)
			{
				std::set<uint> unique_vrts;
				for(uint i = 0; i < prim_cnt; ++i)
					for(uint j = 0; j < prim_cnt; ++j)
						unique_vrts.insert(_mesh->vertex_indices[i][j]);
				if(unique_vrts.size() > 4) return cost_leaf;
			}
			else if(_max_prims == FTB)
			{
				if(!compress(prim_id0, prim_cnt, *_mesh))
					return cost_leaf;
			}

			if(args.leaf_cost == LINEAR)
			{
				cost_leaf = 0.0f;
				for(uint i = 0; i < prim_cnt; ++i)
					cost_leaf += bld_objs[prim_id0 + i].cost;
			}
			else
			{
				cost_leaf = args.leaf_cost;
			}
		}
		return cost_leaf;
	}

	void _collapse_leafs(const std::vector<BuildObject>& bld_objs)
	{
		for(int n = nodes.size() - 1; n >= 0; --n)
		{
			Node& node = nodes[n];
			if(!node.ptr.is_int) continue;
			
			Node* child = &nodes[node.ptr.child_idx];
			if(child[0].ptr.is_int || child[1].ptr.is_int) continue;

			uint prim_idx = child[0].ptr.prim_idx;
			uint prim_cnt = child[0].ptr.prim_cnt + child[1].ptr.prim_cnt;
			if(prim_cnt > FTB::MAX_TRIS) continue;

			//float cost_leaf = node.aabb.surface_area() * _cost_leaf(bld_objs, prim_idx, prim_cnt);
			if(_cost_leaf(bld_objs, prim_idx, prim_cnt) == INFINITY) continue;

			//float cost_internal = child[0].aabb.surface_area() + child[1].aabb.surface_area();
			//if(cost_leaf <= cost_internal)
			{
				node.ptr.is_int = 0;
				node.ptr.prim_idx = child[0].ptr.prim_idx;
				node.ptr.prim_cnt = prim_cnt;
				child[0].ptr.prim_cnt = 0;
				child[1].ptr.prim_cnt = 0;
			}
		}
	}

	void _collapse_nodes(const std::vector<BuildObject>& bld_objs)
	{
		if(args.collapse_method == GREEDY)
		{
			_collapse_greedy();
		}
		else if(args.collapse_method == DYNAMIC)
		{
			_collapse_dynamic(bld_objs);
		}
		else assert(false);
	}

	void _collapse_greedy()
	{
		struct Event
		{
			uint bvh2_index;
			uint wbvh_index;
		};

		std::deque<Event> event_queue;
		event_queue.emplace_back(0, 0);

		std::vector<Node> wnodes;
		wnodes.emplace_back();

		std::vector<uint> node_set;
		node_set.reserve(args.width);

		while(!event_queue.empty())
		{
			Event event = event_queue.front(); event_queue.pop_front();
			const Node& node = nodes[event.bvh2_index];
			Node& wnode = wnodes[event.wbvh_index];
			wnode = node;

			if(!node.ptr.is_int) continue;

			node_set.clear();
			node_set.push_back(event.bvh2_index);
			while(node_set.size() < args.width)
			{
				float best_sa = -INFINITY;
				uint best_index = ~0u;
				for(uint i = 0; i < node_set.size(); ++i)
				{
					if(!nodes[node_set[i]].ptr.is_int) continue;
					float sa = nodes[node_set[i]].aabb.surface_area();
					if(sa <= best_sa) continue;
					best_index = i;
					best_sa = sa;
				}
				if(best_index == ~0u) break;

				uint base_index = nodes[node_set[best_index]].ptr.child_idx;
				node_set[best_index] = base_index;
				node_set.insert(node_set.begin() + best_index + 1, base_index + 1);
			}

			wnode.ptr.child_idx = wnodes.size();
			wnode.ptr.child_cnt = node_set.size();
			for(uint i = 0; i < node_set.size(); ++i)
			{
				event_queue.emplace_back(node_set[i], wnodes.size());
				wnodes.emplace_back();
			}
		}

		nodes = wnodes;
	}

	void _collapse_dynamic(const std::vector<BuildObject>& bld_objs)
	{
		enum Type
		{
			LEAF,
			INTERNAL,
			DISTRIBUTE
		};
		struct Decision
		{
			uint8_t type;
			uint8_t distribute_left;
			uint8_t distribute_right;
			uint8_t _pad;
			float cost;
		};
		union alignas(64) Cache
		{
			struct
			{
				uint32_t prim_cnt;
				uint32_t prim_idx;
			};
			Decision decisions[16];
		};
		std::vector<Cache> caches(nodes.size());

		for(int n = nodes.size() - 1; n >= 0; --n)
		{
			Node& node = nodes[n];
			Cache& cache = caches[n];
			if(node.ptr.is_int)
			{
				//Collect prims for internal
				uint left_idx = node.ptr.child_idx + 0, right_idx = node.ptr.child_idx + 1;
				cache.prim_cnt = (uint)caches[left_idx].prim_cnt + (uint)caches[right_idx].prim_cnt;
				cache.prim_idx = caches[left_idx].prim_idx;
			}
			else
			{
				//Collect prims for leaf
				cache.prim_cnt = node.ptr.prim_cnt;
				cache.prim_idx = node.ptr.prim_idx;
			}

			//Leaf cost
			cache.decisions[1].cost = _cost_leaf(bld_objs, cache.prim_idx, cache.prim_cnt) * node.aabb.surface_area();
			if(cache.decisions[1].cost != cache.decisions[1].cost) cache.decisions[1].cost = INFINITY;
			cache.decisions[1].type = LEAF;

			if(node.ptr.is_int)
			{
				uint left_idx = node.ptr.child_idx + 0, right_idx = node.ptr.child_idx + 1;

				//Internal cost
				uint j = args.width;
				for(uint k = 1; k < j; ++k)
				{
					float cost_internal = node.aabb.surface_area() +
						caches[left_idx].decisions[k].cost +
						caches[right_idx].decisions[j - k].cost;

					if(cost_internal < cache.decisions[1].cost)
					{
						cache.decisions[1].distribute_left = k;
						cache.decisions[1].distribute_right = j - k;
						cache.decisions[1].cost = cost_internal;
						cache.decisions[1].type = INTERNAL;
					}
				}

				//Distribute cost
				for(uint j = 2; j < args.width; ++j)
				{
					cache.decisions[j] = cache.decisions[j - 1];
					for(uint k = 1; k < j; ++k)
					{
						float cost_distribute = caches[left_idx].decisions[k].cost + caches[right_idx].decisions[j - k].cost;
						if(cost_distribute < cache.decisions[j].cost)
						{
							uint d = j == args.width ? 1 : j;
							cache.decisions[d].distribute_left = k;
							cache.decisions[d].distribute_right = j - k;
							cache.decisions[d].cost = cost_distribute;
							cache.decisions[d].type = DISTRIBUTE;
						}
					}
				}
			}
			else
			{
				for(uint j = 2; j < args.width; ++j)
					cache.decisions[j] = cache.decisions[j - 1];
			}

			if(cache.decisions[1].cost == INFINITY) __debugbreak();
		}

		struct CostItem
		{
			uint n;
			uint i;
		};

		struct Event
		{
			CostItem item;
			uint wbvh_index;
		};

		std::deque<Event> event_queue;
		event_queue.emplace_back(CostItem(0, 1), 0);

		std::vector<Node> wnodes;
		wnodes.emplace_back();

		std::vector<CostItem> node_set;
		node_set.reserve(args.width);

		while(!event_queue.empty())
		{
			Event event = event_queue.front();  event_queue.pop_front();
			const Node& node = nodes[event.item.n];
			Node& wnode = wnodes[event.wbvh_index];
			wnode = node;

			Decision d = caches[event.item.n].decisions[event.item.i];
			if(d.type == INTERNAL)
			{
				node_set.clear();
				node_set.emplace_back(node.ptr.child_idx + 0, d.distribute_left);
				node_set.emplace_back(node.ptr.child_idx + 1, d.distribute_right);

				//collect nodes
				uint i = 0;
				while(i < node_set.size())
				{
					CostItem item = node_set[i];
					const Node& n = nodes[item.n];
					d = caches[item.n].decisions[item.i];
					if(d.type == DISTRIBUTE)
					{
						node_set[i] = CostItem(n.ptr.child_idx, d.distribute_left);
						node_set.insert(node_set.begin() + i + 1, CostItem(n.ptr.child_idx + 1, d.distribute_right));
					}
					else i++;
				}

				wnode.ptr.child_idx = wnodes.size();
				wnode.ptr.child_cnt = node_set.size();
				//map children
				for(uint i = 0; i < node_set.size(); ++i)
				{
					event_queue.emplace_back(node_set[i], wnodes.size());
					wnodes.emplace_back();
				}
			}
			else if(d.type == LEAF)
			{
				wnode.ptr.is_int = 0;
				wnode.ptr.prim_cnt = caches[event.item.n].prim_cnt;
				wnode.ptr.prim_idx = caches[event.item.n].prim_idx;
			}
			else assert(false);
		}

		nodes = wnodes;
	}

	bool _try_merge_nodes(BVH::Node& node, uint target_child, uint source_child)
	{
		if(!node.ptr.is_int) return false;

		BVH::Node& target = nodes[node.ptr.child_idx + target_child];
		BVH::Node& source = nodes[node.ptr.child_idx + source_child];
		if(source.ptr.is_int != target.ptr.is_int) return false;
		if(!target.ptr.is_int) return false;

		uint child_cnt = target.ptr.child_cnt + source.ptr.child_cnt;
		if(child_cnt > args.width) return false;

		//apply the merge
		target.ptr.child_cnt = child_cnt;
		for(uint i = target_child; i <= source_child; ++i)
			nodes[node.ptr.child_idx + i].ptr = target.ptr;

		return true;
	}

	void _merge_nodes()
	{
		for(uint i = 0; i < nodes.size(); ++i)
		{
			BVH::Node& node = nodes[i];
			if(!node.ptr.is_int) continue;

			uint target = 0;
			for(uint source = 1; source < node.ptr.child_cnt; ++source)
				if(!_try_merge_nodes(node, target, source))
					target = source;
		}
	}

	bool _try_merge_leafs(const std::vector<BuildObject>& bld_objs, BVH::Node& node, uint target_child, uint source_child)
	{
		if(!node.ptr.is_int) return false;

		BVH::Node& target = nodes[node.ptr.child_idx + target_child];
		BVH::Node& source = nodes[node.ptr.child_idx + source_child];
		if(source.ptr.is_int != target.ptr.is_int) return false;
		if(target.ptr.is_int) return false;

		uint prim_cnt = target.ptr.prim_cnt + source.ptr.prim_cnt;
		if(_cost_leaf(bld_objs, target.ptr.prim_idx, prim_cnt) == INFINITY) return false;
		
		//apply the merge
		target.ptr.prim_cnt = prim_cnt;
		for(uint i = target_child; i <= source_child; ++i)
			nodes[node.ptr.child_idx + i].ptr = target.ptr;

		return true;
	}

	void _merge_leafs(const std::vector<BuildObject>& bld_objs)
	{
		uint last_ptr = ~0;
		for(uint i = 0; i < nodes.size(); ++i)
		{
			BVH::Node& node = nodes[i];
			if(!node.ptr.is_int || node.ptr.raw == last_ptr) continue;
			last_ptr = node.ptr.raw;

			uint target = 0;
			for(uint source = 1; source < node.ptr.child_cnt; ++source)
				if(!_try_merge_leafs(bld_objs, node, target, source))
					target = source;
		}
	}

	float _compute_cost(const std::vector<BuildObject>& bld_objs) const
	{
		float cost = 0.0f, root_surface_area = nodes[0].aabb.surface_area();
		for(int i = nodes.size() - 1; i >= 0; --i)
		{
			const BVH::Node& node = nodes[i];
			float A = node.aabb.surface_area() / root_surface_area;
			if(node.ptr.is_int)
			{
				float c_node = 1.0f;
				cost += A * c_node;
			}
			else
			{
				float c_prim = 0.0f;
				for(uint j = 0; j < node.ptr.prim_cnt; ++j)
					c_prim += bld_objs[node.ptr.prim_idx + j].cost;
				cost += A;
			}
		}
		return cost;
	}

	void _print_stats_bvh2() const
	{
		std::string build_method = "NA";
		if(args.build_method == LBVH) build_method = "LBVH";
		if(args.build_method == SAH_32BIN) build_method = "SAH_32BIN";
		if(args.build_method == SAH) build_method = "SAH";

		printf("BVH2: Size: %0.1f MiB\n", (float)sizeof(Node) * nodes.size() / (1 << 20));
		printf("BVH2: SAH Cost: %.2f\n", sah_cost);
		printf("BVH2: Build Type: %s\n", build_method.c_str());
	}

	void _print_stats_wbvh() const
	{
		std::string collapse_method = "NA";
		if(args.collapse_method == DYNAMIC)  collapse_method = "DYNAMIC";
		if(args.collapse_method == GREEDY) collapse_method = "GREEDY";

		std::string leaf_cost = "NA";
		if(args.leaf_cost == LINEAR)  leaf_cost = "LINEAR";
		else                          leaf_cost = std::to_string(args.leaf_cost).c_str();

		std::string max_prims_collapse = std::to_string((uint)args.max_prims_collapse).c_str();
		if(args.max_prims_collapse == PAIR) max_prims_collapse = "PAIR";
		if(args.max_prims_collapse == FTB)  max_prims_collapse = "FTB";

		std::string max_prims_merge = std::to_string((uint)args.max_prims_merge).c_str();
		if(args.max_prims_merge == PAIR) max_prims_merge = "PAIR";
		if(args.max_prims_merge == FTB)  max_prims_merge = "FTB";

		printf("BVH%d: Size: %0.1f MiB\n", args.width, (float)sizeof(Node) * nodes.size() / (1 << 20));
		printf("BVH%d: SAH Cost: %.2f\n", args.width, sah_cost);
		printf("BVH%d: Leaf Cost: %s\n", args.width, leaf_cost.c_str());
		printf("BVH%d: Collapse: %s\n", args.width, collapse_method.c_str());
		printf("BVH%d: Max Clps: %s\n", args.width, max_prims_collapse.c_str());
		printf("BVH%d: Node Merg: %d\n", args.width, args.merge_nodes);
		printf("BVH%d: Leaf Merg: %d\n", args.width, args.merge_leafs);
		printf("BVH%d: Max Merg: %s\n", args.width, max_prims_merge.c_str());

		uint node_histo[32];
		for(uint i = 0; i < 32; ++i) node_histo[i] = 0;

		uint leaf_histo[32];
		for(uint i = 0; i < 32; ++i) leaf_histo[i] = 0;

		uint total_aabbs = 0, total_prims = 0, total_int = 0, total_leaf = 0;
		for(uint i = 0; i < nodes.size(); ++i)
		{
			const Node& node = nodes[i];
			if(node.ptr.is_int)
			{
				total_int++;
				total_aabbs += node.ptr.child_cnt;
				node_histo[node.ptr.child_cnt]++;
			}
			else
			{
				total_leaf++;
				total_prims += node.ptr.prim_cnt;
				leaf_histo[node.ptr.prim_cnt]++;
			}
		}

		printf("BVH%d: Node Fullness: %.2f\n", args.width, (float)total_aabbs / total_int);
		for(uint i = 2; i <= args.width; ++i)
		{
			float precent = 100.0f * node_histo[i] / total_int;
			printf("%02d: %5.1f%% %.*s\n", i, precent, (uint)std::round(precent),
				"....................................................................................................");
		}

		printf("BVH%d: Leaf Fullness: %.2f\n", args.width, (float)total_prims / total_leaf);
		for(uint i = 1; i <= (_max_prims & 0x1f); ++i)
		{
			float precent = 100.0f * leaf_histo[i] / total_leaf;
			printf("%02d: %5.1f%% %.*s\n", i, precent, (uint)std::round(precent),
				 "....................................................................................................");
		}
	}

	void _serialize(const char* file_path, const std::vector<BuildObject>& bld_objs) const
	{
		std::ofstream file_stream(file_path, std::ios::binary);

		FileHeader header;
		header.version = VERSION;
		header.build_method = args.build_method;
		header.num_nodes = nodes.size();
		header.num_build_objects = bld_objs.size();

		file_stream.write((char*)&header, sizeof(FileHeader));
		file_stream.write((char*)nodes.data(), sizeof(Node) * nodes.size());
		file_stream.write((char*)bld_objs.data(), sizeof(BuildObject) * bld_objs.size());
	}

	bool _deserialize(const char* file_path, std::vector<BuildObject>& bld_objs)
	{
		bool succeeded = false;
		std::ifstream file_stream(file_path, std::ios::binary);
		if(file_stream.is_open())
		{
			FileHeader header;
			file_stream.read((char*)&header, sizeof(FileHeader));

			if(header.version == VERSION &&
				header.num_build_objects == bld_objs.size() &&
				header.build_method == args.build_method)
			{
				nodes.resize(header.num_nodes);
				bld_objs.resize(header.num_build_objects);
				file_stream.read((char*)nodes.data(), sizeof(Node) * nodes.size());
				file_stream.read((char*)bld_objs.data(), sizeof(BuildObject) * bld_objs.size());
				succeeded = true;
			}
		}

		return succeeded;
	}
#endif
};

struct RestartTrail
{
	const static uint N = 15;
	const static uint BITS_PER_LEVEL = 4;

	rtm::BitArray<128> data;

	uint find_parent_level(uint level) const
	{
		for(uint i = level - 1; i < ~0u; --i)
			if(get(i) < N)
				return i;
		return ~0u;
	}

	uint get(uint level) const
	{
		assert(level < 64 / BITS_PER_LEVEL);
		return data.read(level * BITS_PER_LEVEL, BITS_PER_LEVEL);
	}

	void set(uint level, uint value)
	{
		assert(level < 64 / BITS_PER_LEVEL);
		data.write(level * BITS_PER_LEVEL, BITS_PER_LEVEL, value);
	}

	void clear(uint start_level)
	{
		for(uint i = start_level; i < 64 / BITS_PER_LEVEL; ++i)
			set(i, 0);
	}

	bool is_done()
	{
		return get(0) >= N;
	}

	void mark_done()
	{
		set(0, N);
	}
};

}