#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"

namespace Arches { namespace Units { namespace TRaX {

// Instrumentation for the live visualizer's BVH treemap (RusticArches).
//
// Node ids are unified across the HE2CWBVH arrays: internal nodes keep their index in `nodes`
// (0 .. internal_count - 1) and leaves are `internal_count + FTB index`.
//
// Every RT core owns one BvhProbe, so the hot path never touches shared state. main.cpp merges
// and clears the probes once per logging interval, while the simulator is between cycles.

// Read-only data shared by all probes.
struct BvhProbeShared
{
	uint32_t internal_count{0};
	std::vector<uint32_t> parent; // ~0u for the root

	// Camera used to map a primary ray back to its pixel. The kernel generates rays itself, so the
	// RT core never sees a pixel index; inverting Camera::generate_ray_through_pixel recovers it.
	rtm::Camera camera;
	uint32_t width{0}, height{0};

	// Full traversal traces are recorded for pixels where x % stride == 0 && y % stride == 0.
	uint32_t trace_stride{16};

	uint32_t pixel_of(const rtm::Ray& ray) const
	{
		if(ray.o.x != camera._position.x || ray.o.y != camera._position.y || ray.o.z != camera._position.z)
			return ~0u;

		float w = -rtm::dot(ray.d, camera._z);
		if(!(w > 0.0f)) return ~0u;
		float u = rtm::dot(ray.d, camera._x) / (rtm::dot(camera._x, camera._x) * w);
		float v = rtm::dot(ray.d, camera._y) / (rtm::dot(camera._y, camera._y) * w);
		float fx = (u + 0.5f) * width;
		float fy = (v + 0.5f) * height;
		if(!(fx >= 0.0f && fy >= 0.0f)) return ~0u;
		uint32_t x = (uint32_t)fx, y = (uint32_t)fy;
		if(x >= width || y >= height) return ~0u;
		return y * width + x;
	}

	bool is_traced(uint32_t pixel) const
	{
		if(pixel == ~0u || trace_stride == 0) return false;
		return (pixel % width) % trace_stride == 0 && (pixel / width) % trace_stride == 0;
	}
};

class BvhProbe
{
public:
	// Which level served a fetch, from MemoryReturn::Flags::hit_level.
	enum Served : uint
	{
		SERVED_MEMORY = 0, // missed every cache level (L3 miss, served by DRAM)
		SERVED_L1 = 1,
		SERVED_L2 = 2,
		SERVED_L3 = 3,
		NUM_SERVED = 4,
	};

	struct NodeCounters
	{
		uint32_t visits{0};    // fetches issued for this node (restart revisits included)
		uint32_t wasted{0};    // visits by rays whose closest hit is not inside this node's subtree
		uint32_t tri_tests{0}; // triangle intersection tests (leaves only)
		uint32_t served[NUM_SERVED]{}; // fetched sectors by the level that served them
	};

	// Per in-flight ray, kept in the RT core's RayState.
	struct RayTrace
	{
		uint32_t pixel{~0u};
		uint32_t hit_leaf{~0u};
		std::vector<uint32_t> visits;
	};

	const BvhProbeShared* shared{nullptr};

	// Accumulated since the last collect().
	std::unordered_map<uint32_t, NodeCounters> nodes;
	std::vector<uint8_t> rays;   // RayRecord stream, see append_ray()
	std::vector<uint8_t> traces; // trace stream, see end_ray()
	uint32_t ray_count{0};
	uint32_t trace_count{0};

	explicit BvhProbe(const BvhProbeShared* shared) : shared(shared) {}

	uint32_t leaf_id(uint32_t ftb_index) const { return shared->internal_count + ftb_index; }

	void begin_ray(RayTrace& trace, const rtm::Ray& ray)
	{
		trace.pixel = shared->pixel_of(ray);
		trace.hit_leaf = ~0u;
		trace.visits.clear();
	}

	void visit(RayTrace& trace, uint32_t node_id)
	{
		trace.visits.push_back(node_id);
		nodes[node_id].visits++;
	}

	void served(uint32_t node_id, uint level)
	{
		nodes[node_id].served[level < NUM_SERVED ? level : SERVED_MEMORY]++;
	}

	void tri_tests(uint32_t node_id, uint count)
	{
		nodes[node_id].tri_tests += count;
	}

	void end_ray(RayTrace& trace, bool hit)
	{
		if(!hit) trace.hit_leaf = ~0u;

		// Nodes on the path from the root to the closest-hit leaf; every other visit was wasted.
		uint32_t path[64]; uint path_len = 0;
		for(uint32_t n = trace.hit_leaf; n != ~0u && path_len < 64; n = shared->parent[n])
			path[path_len++] = n;

		uint32_t leaves = 0;
		for(uint32_t id : trace.visits)
		{
			if(id >= shared->internal_count) leaves++;
			bool on_path = false;
			for(uint i = 0; i < path_len; ++i)
				if(path[i] == id) { on_path = true; break; }
			if(!on_path) nodes[id].wasted++;
		}

		if(trace.pixel != ~0u)
		{
			// RayRecord: u32 pixel, u32 hit_leaf (~0 on a miss), u16 nodes visited, u16 leaves visited
			uint16_t n16 = (uint16_t)std::min<size_t>(trace.visits.size() - leaves, 0xffff);
			uint16_t l16 = (uint16_t)std::min<uint32_t>(leaves, 0xffff);
			_put(rays, trace.pixel);
			_put(rays, trace.hit_leaf);
			_put(rays, n16);
			_put(rays, l16);
			ray_count++;

			if(shared->is_traced(trace.pixel))
			{
				// Trace: u32 pixel, u32 hit_leaf, u32 length, length x u32 node ids in visit order
				_put(traces, trace.pixel);
				_put(traces, trace.hit_leaf);
				_put(traces, (uint32_t)trace.visits.size());
				size_t at = traces.size();
				traces.resize(at + trace.visits.size() * sizeof(uint32_t));
				std::memcpy(traces.data() + at, trace.visits.data(), trace.visits.size() * sizeof(uint32_t));
				trace_count++;
			}
		}

		trace.visits.clear();
	}

	void clear()
	{
		nodes.clear();
		rays.clear();
		traces.clear();
		ray_count = 0;
		trace_count = 0;
	}

	// Merges every probe's interval data into one BVH_STATS payload and clears the probes.
	//   u64 cycle
	//   u32 node record count, records of { u32 id, u32 visits, u32 wasted, u32 tri_tests,
	//                                       u32 served_memory, u32 served_l1, u32 served_l2, u32 served_l3 }
	//   u32 ray count, records of { u32 pixel, u32 hit_leaf, u16 nodes, u16 leaves }
	//   u32 trace count, traces of { u32 pixel, u32 hit_leaf, u32 length, length x u32 node id }
	static std::vector<uint8_t> collect(std::vector<BvhProbe>& probes, uint64_t cycle)
	{
		std::unordered_map<uint32_t, NodeCounters> merged;
		size_t ray_bytes = 0, trace_bytes = 0;
		uint32_t ray_count = 0, trace_count = 0;
		for(BvhProbe& probe : probes)
		{
			for(const auto& [id, c] : probe.nodes)
			{
				NodeCounters& m = merged[id];
				m.visits += c.visits;
				m.wasted += c.wasted;
				m.tri_tests += c.tri_tests;
				for(uint i = 0; i < NUM_SERVED; ++i) m.served[i] += c.served[i];
			}
			ray_bytes += probe.rays.size();
			trace_bytes += probe.traces.size();
			ray_count += probe.ray_count;
			trace_count += probe.trace_count;
		}

		std::vector<std::pair<uint32_t, NodeCounters>> sorted(merged.begin(), merged.end());
		std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

		std::vector<uint8_t> out;
		out.reserve(8 + 4 + sorted.size() * 32 + 4 + ray_bytes + 4 + trace_bytes);
		_put(out, cycle);
		_put(out, (uint32_t)sorted.size());
		for(const auto& [id, c] : sorted)
		{
			_put(out, id);
			_put(out, c.visits);
			_put(out, c.wasted);
			_put(out, c.tri_tests);
			for(uint i = 0; i < NUM_SERVED; ++i) _put(out, c.served[i]);
		}
		_put(out, ray_count);
		for(BvhProbe& probe : probes) out.insert(out.end(), probe.rays.begin(), probe.rays.end());
		_put(out, trace_count);
		for(BvhProbe& probe : probes) out.insert(out.end(), probe.traces.begin(), probe.traces.end());

		for(BvhProbe& probe : probes) probe.clear();
		return out;
	}

#ifndef __riscv
	// Builds the BVH payload (sent once after HELLO) and fills in `shared`'s topology.
	//   u32 version (1), u32 width, u32 internal count, u32 leaf count, u32 trace stride
	//   per node, internal nodes first, then leaves:
	//     u32 parent (~0 for the root), f32 min[3], f32 max[3], f32 triangle area,
	//     u32 first triangle, u8 triangle count, u8 child count, child count x u32 child id
	// Bounds are the dequantized child boxes stored in the parent, so they are conservative.
	static std::vector<uint8_t> build_topology(const rtm::HE2CWBVH& bvh, BvhProbeShared& shared)
	{
		const uint32_t internal_count = (uint32_t)bvh.nodes.size();
		const uint32_t leaf_count = (uint32_t)bvh.ftbs.size();
		const uint32_t total = internal_count + leaf_count;

		shared.internal_count = internal_count;
		shared.parent.assign(total, ~0u);

		std::vector<rtm::AABB> bounds(total);
		std::vector<std::vector<uint32_t>> children(internal_count);

		for(uint32_t n = 0; n < internal_count; ++n)
		{
			rtm::BVH::Node slots[rtm::HE2CWBVH::WIDTH];
			rtm::decompress(bvh.nodes[n], slots);
			const rtm::HE2CWBVH::Node& cw = bvh.nodes[n];

			uint32_t last = ~0u;
			for(uint i = 0; i < rtm::HE2CWBVH::WIDTH; ++i)
			{
				const rtm::AABB& box = slots[i].aabb;
				if(box.min.x > box.max.x || box.min.y > box.max.y || box.min.z > box.max.z)
					continue; // unused slot

				uint32_t child = slots[i].ptr.is_int ? (uint32_t)slots[i].ptr.child_idx : internal_count + (uint32_t)slots[i].ptr.prim_idx;
				if(child >= total) continue;

				// Consecutive slots can point at the same child (merged boxes); the child's box is their union.
				bounds[child].add(box);
				if(child != last && ((cw.o_mask >> i) & 0x1))
				{
					children[n].push_back(child);
					shared.parent[child] = n;
				}
				last = child;
			}
		}

		// The root's box is the union of its children's boxes.
		if(internal_count > 0)
			for(uint32_t child : children[0])
				bounds[0].add(bounds[child]);

		std::vector<uint8_t> out;
		out.reserve(20 + (size_t)total * 40);
		_put(out, (uint32_t)1);
		_put(out, (uint32_t)rtm::HE2CWBVH::WIDTH);
		_put(out, internal_count);
		_put(out, leaf_count);
		_put(out, shared.trace_stride);

		auto put_box = [&](const rtm::AABB& box)
		{
			_put(out, box.min.x); _put(out, box.min.y); _put(out, box.min.z);
			_put(out, box.max.x); _put(out, box.max.y); _put(out, box.max.z);
		};

		for(uint32_t n = 0; n < internal_count; ++n)
		{
			_put(out, shared.parent[n]);
			put_box(bounds[n]);
			_put(out, 0.0f);
			_put(out, ~0u);
			_put(out, (uint8_t)0);
			_put(out, (uint8_t)children[n].size());
			for(uint32_t child : children[n]) _put(out, child);
		}

		for(uint32_t l = 0; l < leaf_count; ++l)
		{
			rtm::IntersectionTriangle tris[rtm::FTB::MAX_TRIS];
			uint tri_count = rtm::decompress(bvh.ftbs[l], tris);
			float area = 0.0f;
			for(uint i = 0; i < tri_count; ++i) area += tris[i].tri.surface_area();

			uint32_t id = internal_count + l;
			_put(out, shared.parent[id]);
			put_box(bounds[id]);
			_put(out, area);
			_put(out, (uint32_t)bvh.ftbs[l].prim_idx);
			_put(out, (uint8_t)tri_count);
			_put(out, (uint8_t)0);
		}

		return out;
	}
#endif

private:
	template<typename T>
	static void _put(std::vector<uint8_t>& out, T v)
	{
		size_t at = out.size();
		out.resize(at + sizeof(T));
		std::memcpy(out.data() + at, &v, sizeof(T));
	}
};

}}}
