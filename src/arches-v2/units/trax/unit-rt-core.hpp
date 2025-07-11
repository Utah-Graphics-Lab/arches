#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"

#include "../unit-base.hpp"
#include "../unit-memory-base.hpp"

namespace Arches { namespace Units { namespace TRaX {

template<typename NT, typename PT>
class UnitRTCore : public UnitMemoryBase
{
public:
	struct Configuration
	{
		uint num_clients{1};
		uint max_rays{1};
		paddr_t node_base_addr{0x0ull};
		paddr_t tri_base_addr{0x0ull};
		paddr_t vrt_base_addr{0x0ull};

		UnitMemoryBase* cache{nullptr};
		uint cache_port{0};
		uint num_cache_ports{1};
		uint cache_port_stride{1};
	};

private:
	enum class IssueType
	{
		NODE_FETCH,
		TRI_FETCH,
		POP_CULL,
		HIT_RETURN,
		NUM_TYPES,
	};

	struct StackEntry
	{
		float t;
		rtm::WBVH::Node::Data data;
		bool is_last;

		StackEntry() {}
	};

	struct StagingBuffer
	{
		paddr_t address;
		uint bytes_filled;
		uint type;
		uint id;

		union
		{
			uint8_t data[1];
			NT node;
			PT prim;
		};

		StagingBuffer() {}
	};

	struct RayState
	{
		enum class Phase
		{
			RAY_FETCH,
			SCHEDULER,
			HIT_RETURN,
			NODE_FETCH,
			TRI_FETCH,
			VRT_FETCH,
			NODE_ISECT,
			TRI_ISECT,
			NUM_PHASES,
		}
		phase;

		rtm::Ray ray;
		rtm::vec3 inv_d;
		rtm::Hit hit;

		const static uint STACK_SIZE = 4;
		rtm::RestartTrail restart_trail;
		StackEntry stack[STACK_SIZE + rtm::WBVH::WIDTH];
		uint8_t stack_size;
		uint8_t level;
		bool update_restart_trail;

		MemoryRequest::Flags flags;
		BitStack58 dst;

		StagingBuffer buffer;

		bool done;

		RayState() {};
	};

	struct FetchItem
	{
		paddr_t addr;
		uint8_t size;
		uint16_t ray_id;
	};

	//interconnects
	RequestCascade _request_network;
	ReturnCascade _return_network;
	UnitMemoryBase* _cache;
	uint _cache_port;
	uint _cache_port_stride;

	std::vector<std::queue<MemoryRequest>> _cache_fetch_queues;
	
	//ray scheduling hardware
	std::queue<uint> _ray_scheduling_queue;
	std::queue<uint> _ray_return_queue;
	uint _divide_issue_count{0};

	std::set<uint> _free_ray_ids;
	std::vector<RayState> _ray_states;

	//node pipline
	std::queue<uint> _node_isect_queue;
	LatencyFIFO<uint> _box_pipline;
	uint _box_issue_count{0};

	//tri pipline
	std::queue<uint> _tri_isect_queue;
	LatencyFIFO<uint> _tri_pipline;
	uint _tri_issue_count{0};

	//meta data
	uint _max_rays;
	paddr_t _node_base_addr;
	paddr_t _tri_base_addr;
	paddr_t _vrt_base_addr;
	uint _last_ray_id{0};
	
	std::set<uint> _rows_accessed;

	bool _drain_phase{false};
	bool _pop_culling{false};

	uint _stall_cycles{0};
public:
	UnitRTCore(const Configuration& config);

	void clock_rise() override;

	void clock_fall() override;

	bool request_port_write_valid(uint port_index) override
	{
		return _request_network.is_write_valid(port_index);
	}

	void write_request(const MemoryRequest& request) override
	{
		_request_network.write(request, request.port);
	}

	bool return_port_read_valid(uint port_index) override
	{
		return _return_network.is_read_valid(port_index);
	}

	const MemoryReturn& peek_return(uint port_index) override
	{
		return _return_network.peek(port_index);
	}

	const MemoryReturn read_return(uint port_index) override
	{
		return _return_network.read(port_index);
	}

private:
	paddr_t _align_address(paddr_t addr)
	{
		return (addr >> log2i(MemoryRequest::MAX_SIZE)) << log2i(MemoryRequest::MAX_SIZE);
	}

	bool _try_queue_node(uint ray_id, uint node_id);
	bool _try_queue_tri(uint ray_id, uint tri_id);
	bool _try_queue_vrts(uint ray_id);
	bool _try_queue_prefetch(paddr_t addr, uint size, uint cache_mask);

	void _read_requests();
	void _read_returns();
	void _schedule_ray();
	void _simualte_node_pipline();
	void _simualte_tri_pipline();

	void _issue_requests();
	void _issue_returns();

public:
	class Log
	{
	private:
		constexpr static uint NUM_COUNTERS = 32;

	public:
		union
		{
			struct
			{
				uint64_t rays;
				uint64_t nodes;
				uint64_t strips;
				uint64_t tris;
				uint64_t restarts;
				uint64_t hits_returned;
				uint64_t issue_counters[(uint)IssueType::NUM_TYPES];
				uint64_t stall_counters[(uint)RayState::Phase::NUM_PHASES];
			};
			uint64_t counters[NUM_COUNTERS];
		};

		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] = 0;
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] += other.counters[i];
		}

		void print(uint num_units = 1)
		{
			const static std::string phase_names[] =
			{
				"RAY_FETCH",
				"SCHEDULER",
				"HIT_RETURN",
				"NODE_FETCH",
				"TRI_FETCH",
				"VRT_FETCH",
				"NODE_ISECT",
				"TRI_ISECT",
				"NUM_PHASES",
			};

			const static std::string issue_names[] =
			{
				"NODE_FETCH",
				"TRI_FETCH",
				"POP_CULL",
				"HIT_RETURN",
				"NUM_TYPES",
			};

			printf("Rays: %lld\n", rays / num_units);
			printf("Nodes: %lld\n", nodes / num_units);
			printf("Strips: %lld\n", strips / num_units);
			printf("Tris: %lld\n", tris / num_units);
			printf("Restarts: %lld\n", restarts / num_units);
			printf("\n");
			printf("Nodes/Ray: %.2f\n", (double)nodes / rays);
			printf("Strips/Ray: %.2f\n", (double)strips / rays);
			printf("Tris/Ray: %.2f\n", (double)tris / rays);
			printf("Restarts/Ray: %.2f\n", (double)restarts / rays);

			uint64_t issue_total = 0;
			std::vector<std::pair<const char*, uint64_t>> _issue_counter_pairs;
			for(uint i = 0; i < (uint)IssueType::NUM_TYPES; ++i)
			{
				issue_total += issue_counters[i];
				_issue_counter_pairs.push_back({issue_names[i].c_str(), issue_counters[i]});
			}
			std::sort(_issue_counter_pairs.begin(), _issue_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			uint64_t stall_total = 0;
			std::vector<std::pair<const char*, uint64_t>> _data_stall_counter_pairs;
			for(uint i = 0; i < (uint)RayState::Phase::NUM_PHASES; ++i)
			{
				stall_total += stall_counters[i];
				_data_stall_counter_pairs.push_back({phase_names[i].c_str(), stall_counters[i]});
			}
			std::sort(_data_stall_counter_pairs.begin(), _data_stall_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			uint64_t total = stall_total + issue_total;

			printf("\nIssue Cycles: %lld (%.2f%%)\n", issue_total / num_units, 100.0f * issue_total / total);
			for(uint i = 0; i < _issue_counter_pairs.size(); ++i)
				if(_issue_counter_pairs[i].second) printf("\t%s: %lld (%.2f%%)\n", _issue_counter_pairs[i].first, _issue_counter_pairs[i].second / num_units, 100.0 * _issue_counter_pairs[i].second / total);

			printf("\nStall Cycles: %lld (%.2f%%)\n", stall_total / num_units, 100.0f * stall_total / total);
			for(uint i = 0; i < _data_stall_counter_pairs.size(); ++i)
				if(_data_stall_counter_pairs[i].second) printf("\t%s: %lld (%.2f%%)\n", _data_stall_counter_pairs[i].first, _data_stall_counter_pairs[i].second / num_units, 100.0 * _data_stall_counter_pairs[i].second / total);
		};
	}log;
};

}}}