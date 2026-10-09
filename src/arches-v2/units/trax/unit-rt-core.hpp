#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"
#include "rtm/compact-ftb.hpp"
#include <stdexcept>
#include <type_traits>
#include <memory>
#include "ftb-wait-diagnostics.hpp"
#include "memory-path-origins.hpp"

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
		bool stack_trend_prefetch{false};
		uint ttp_max_distance{16};
		bool ttp_leaf_prefetch{false};
		uint prefetch_queue_size{16}; // 32-byte sectors
		bool ftb_wait_diagnostics{false}; // Host-only event tracing; no simulated state/ports.
		bool ftb_memory_diagnostics{false};
		uint16_t memory_path_unit_id{0}; // Host token namespace, not a modeled wire.
		std::function<uint32_t(uint32_t)> canonical_ftb_observer{};
		std::function<void(const rtm::Ray&, const rtm::Hit&)> hit_observer{};
		std::function<uint64_t(const MemoryRequest&)> ray_id_observer{};
		std::function<void(uint64_t, const rtm::Ray&, const rtm::Hit&)> identified_hit_observer{};
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
		rtm::BVHPtr data;
		bool is_last;

		StackEntry() {}
	};
	struct StagingBuffer
	{
		paddr_t address;
		uint bytes_filled; // Compact FTB: seen32B-sector mask; other node/primitive formats: returned byte count.
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

		const static uint STACK_SIZE = 32;
		rtm::RestartTrail restart_trail;
		StackEntry stack[STACK_SIZE + 31];
		uint8_t stack_size;
		uint8_t level;
		bool update_restart_trail;

		MemoryRequest::Flags flags;
		BitStack58 dst;

		StagingBuffer buffer;

		bool done;

		uint64_t prefetch_epoch{0};
		uint64_t original_ray_id{~0ull}; // Host-only validation metadata.

		// TTP keeps its position across pops; pushes reset the scan window.
		uint8_t ttp_state{0};
		int16_t ttp_cursor{-1};
		int16_t ttp_floor{0};
		bool ttp_pending_push{false};
		bool ttp_pending_trim{false};

		StackEntry ttp_snapshot;
		bool ttp_snapshot_valid{false};
		bool ttp_scan_queued{false};

		RayState() {};
	};

	struct FetchItem
	{
		paddr_t addr;
		uint8_t size;
		uint16_t ray_id;
	};

	struct PrefetchItem
	{
		paddr_t addr;
		uint ray_id;
		uint64_t epoch;
		uint8_t cache_mask;
	};

	//interconnects
	RequestCascade _request_network;
	ReturnCascade _return_network;
	UnitMemoryBase* _cache;
	uint _cache_port;
	uint _cache_port_stride;

	std::vector<std::queue<MemoryRequest>> _cache_fetch_queues;
	// One additional read-only stack port serves at most one candidate per global cycle.
	std::queue<uint> _ttp_scan_queue;
	std::deque<PrefetchItem> _prefetch_queue;
	std::set<paddr_t> _queued_prefetch_sectors;
	bool _stack_trend_prefetch;
	uint _ttp_max_distance;
	bool _ttp_leaf_prefetch;
	static constexpr bool _compact_ftb = std::is_same_v<NT, rtm::HE2CWBVH::Node> && std::is_same_v<PT, rtm::FTB>;
	static constexpr uint FTB_BASIC_DECODE_LATENCY = 2; // Fixed abstract baseline, not synthesized timing.
	uint _prefetch_queue_size;
	uint _next_prefetch_cache_port{0};
	bool _prefetch_stack_read_used{false};
	std::unique_ptr<FTBWaitDiagnostics> _ftb_wait;
	std::unique_ptr<MemoryPathOrigins> _memory_path;
	std::function<uint32_t(uint32_t)> _canonical_ftb_observer;
	uint64_t _ftb_wait_cycle{0};
	
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
	// Full physical fetch is followed by one fixed baseline delay. No data is rewritten.
	std::vector<uint8_t> _ftb_decode_countdown; // One actual target byte per HE2/FTB resident slot.
	// Pure host observations below do not grant/reject/reorder target work.
	uint64_t _ftb_decode_cycle{0};
	std::vector<uint8_t> _ftb_decode_ready_counts; // Host-only per-port invariant/counter sampling.
	struct BasicDecodeAccess
	{
		uint64_t original_ray_id{~0ull}, leaf_ordinal{0};
		uint32_t canonical_ftb_id{~0u}, port{0}, slot{0};
		uint64_t full_data_admit{0}, decode_ready{0}, first_tri_issue{0};
	};
	std::vector<BasicDecodeAccess> _ftb_basic_accesses;
	std::vector<size_t> _ftb_basic_index;
	LatencyFIFO<uint> _tri_pipline;
	uint _tri_issue_count{0};

	//meta data
	uint _max_rays;
	paddr_t _node_base_addr;
	paddr_t _tri_base_addr;
	paddr_t _vrt_base_addr;
	std::function<void(const rtm::Ray&, const rtm::Hit&)> _hit_observer;
	std::function<uint64_t(const MemoryRequest&)> _ray_id_observer;
	std::function<void(uint64_t, const rtm::Ray&, const rtm::Hit&)> _identified_hit_observer;
	uint _last_ray_id{0};
	
	std::set<uint> _rows_accessed;

	bool _drain_phase{false};
	bool _pop_culling{false};

	uint _stall_cycles{0};
public:
	UnitRTCore(const Configuration& config);

	void clock_rise() override;

	void clock_fall() override;
	FTBWaitDiagnostics::Totals write_ftb_wait_diagnostics(std::ostream& accesses, std::ostream& rays, uint core) const
	{
		return _ftb_wait ? _ftb_wait->write(accesses, rays, core) : FTBWaitDiagnostics::Totals{};
	}
	MemoryPathOrigins::Totals write_memory_path_diagnostics(std::ostream& out, uint core) const
	{
		return _memory_path ? _memory_path->write(out, core) : MemoryPathOrigins::Totals{};
	}

	struct BasicDecodeTotals
	{
		uint64_t blocks{0}, errors{0}, decode_cycles{0}, ready_to_first_issue{0};
	};
	static void basic_decode_header(std::ostream& out)
	{
		out << "core,original_ray_id,leaf_ordinal,canonical_ftb_id,port,slot,full_data_admit,decode_ready,first_tri_issue,decode_cycles,ready_to_first_issue\n";
	}
	BasicDecodeTotals write_basic_decode_diagnostics(std::ostream& out, uint core) const
	{
		BasicDecodeTotals totals;
		for(const auto& a : _ftb_basic_accesses)
		{
			++totals.blocks;
			if(a.original_ray_id == ~0ull || a.canonical_ftb_id == ~0u || !a.full_data_admit ||
				a.decode_ready != a.full_data_admit + FTB_BASIC_DECODE_LATENCY || a.first_tri_issue < a.decode_ready)
				++totals.errors;
			const uint64_t decode = a.decode_ready >= a.full_data_admit ? a.decode_ready - a.full_data_admit : 0;
			const uint64_t queue = a.first_tri_issue >= a.decode_ready ? a.first_tri_issue - a.decode_ready : 0;
			totals.decode_cycles += decode; totals.ready_to_first_issue += queue;
			out << core << ',' << a.original_ray_id << ',' << a.leaf_ordinal << ',' << a.canonical_ftb_id << ',' << a.port << ',' << a.slot << ',' << a.full_data_admit << ',' << a.decode_ready << ',' << a.first_tri_issue << ',' << decode << ',' << queue << '\n';
		}
		return totals;
	}

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
	paddr_t _primitive_address(uint encoded_id) const;
	uint _primitive_bytes(uint encoded_id) const;
	void _record_ftb_completion(uint ray_id, uint required_bytes, uint requested_sectors);
	bool _try_queue_tri(uint ray_id, uint tri_id);
	bool _try_queue_vrts(uint ray_id);
	bool _try_queue_prefetch(uint ray_id, paddr_t addr, uint size, uint cache_mask);
	bool _prefetch_enabled() const { return _stack_trend_prefetch; }
	void _invalidate_prefetch(uint ray_id);
	void _reset_stack_trend_prefetch(uint ray_id);
	void _stack_trend_pop(uint ray_id);
	void _queue_stack_trend_scan(uint ray_id);
	bool _scan_stack_trend_candidate();

	void _read_requests();
	void _read_returns();
	void _queue_tri_isect(uint ray_id);
	void _advance_ftb_basic_decode();
	void _schedule_ray();
	void _simualte_node_pipline();
	void _simualte_tri_pipline();

	void _issue_requests();
	void _issue_returns();

public:
	class Log
	{
	private:
		constexpr static uint NUM_COUNTERS = 6 + (uint)IssueType::NUM_TYPES +
			(uint)RayState::Phase::NUM_PHASES + 6 + 17;

	public:
		struct FTBBasicDecodeDiagnostics
		{
			bool enabled{false};
			uint resident_slots{0}, ports{0};
			uint64_t started{0}, completed{0}, first_issues{0}, service_ray_cycles{0}, ready_per_port_cycle_max{0};
			void reset() { *this = FTBBasicDecodeDiagnostics(); }
			void accumulate(const FTBBasicDecodeDiagnostics& other)
			{
				enabled = enabled || other.enabled;
				resident_slots = std::max(resident_slots, other.resident_slots); ports = std::max(ports, other.ports);
				started += other.started; completed += other.completed; first_issues += other.first_issues;
				service_ray_cycles += other.service_ray_cycles;
				ready_per_port_cycle_max = std::max(ready_per_port_cycle_max, other.ready_per_port_cycle_max);
			}
			void print() const
			{
				if(!enabled) return;
				printf("\nFTB Basic Decode Latency Cycles: %u\n", FTB_BASIC_DECODE_LATENCY);
				printf("FTB Basic Decode Ports Per Core: %u\n", ports);
				printf("FTB Basic Decode Started Blocks Total: %llu\n", (unsigned long long)started);
				printf("FTB Basic Decode Completed Blocks Total: %llu\n", (unsigned long long)completed);
				printf("FTB Basic Decode First Triangle Issues Total: %llu\n", (unsigned long long)first_issues);
				printf("FTB Basic Decode Service Ray-Cycles Total: %llu\n", (unsigned long long)service_ray_cycles);
				printf("FTB Basic Decode Ready Per-Port Per-Cycle Max: %llu\n", (unsigned long long)ready_per_port_cycle_max);
				printf("FTB Basic Decode Countdown Storage Bytes/Core: %zu\n", size_t(resident_slots) * sizeof(uint8_t));
				printf("FTB Basic Decode Control Storage Bytes/Core: 0\n");
				printf("FTB Basic Decode Total Additional State Bytes/Core: %zu\n", size_t(resident_slots) * sizeof(uint8_t));
				printf("FTB Basic Decode: fixed two global cycles after full physical data admission; original return ports provide II1; original strict triangle FIFO/two subpasses/22-tick triangle pipeline; encoded staging remains read-only; baseline abstract cost, logic/ports/area/timing not synthesized\n");
			}
		} ftb_basic_decode;
		struct FTBPhysicalFetchDiagnostics
		{
			bool enabled{false};
			uint64_t completed_blocks{0}, demand_sectors_requested{0}, demand_sectors_returned{0};
			uint64_t required_sectors[4]{}, requested_sectors[4]{}, physical_blocks[2]{};
			uint64_t demand_physical_sectors{0}, demand_sectors_saved{0};
			void reset() { *this = FTBPhysicalFetchDiagnostics(); }
			void accumulate(const FTBPhysicalFetchDiagnostics& other)
			{
				enabled = enabled || other.enabled;
				completed_blocks += other.completed_blocks;
				demand_sectors_requested += other.demand_sectors_requested; demand_sectors_returned += other.demand_sectors_returned;
				demand_physical_sectors += other.demand_physical_sectors; demand_sectors_saved += other.demand_sectors_saved;
				for(uint i = 0; i < 4; ++i) { required_sectors[i] += other.required_sectors[i]; requested_sectors[i] += other.requested_sectors[i]; }
				for(uint i = 0; i < 2; ++i) physical_blocks[i] += other.physical_blocks[i];
			}
			void print() const
			{
				if(!enabled) return;
				printf("\nFTB Physical Fetch: full compact64/128 allocation upfront; aligned32B requests; no selective tail/header parse/retry path\n");
				printf("FTB Physical Fetch Completed Blocks Total: %llu\n", (unsigned long long)completed_blocks);
				for(uint i = 0; i < 4; ++i)
				{
					printf("FTB Required %u Sectors Completed Blocks: %llu\n", i + 1, (unsigned long long)required_sectors[i]);
					printf("FTB Requested %u Sectors Completed Blocks: %llu\n", i + 1, (unsigned long long)requested_sectors[i]);
				}
				printf("FTB Demand Requested Sectors Total: %llu\n", (unsigned long long)demand_sectors_requested);
				printf("FTB Demand Returned Sectors Total: %llu\n", (unsigned long long)demand_sectors_returned);
				printf("FTB Physical 64B Completed Blocks: %llu\n", (unsigned long long)physical_blocks[0]);
				printf("FTB Physical 128B Completed Blocks: %llu\n", (unsigned long long)physical_blocks[1]);
				printf("FTB Demand Physical Sectors Total: %llu\n", (unsigned long long)demand_physical_sectors);
				printf("FTB Demand Sectors Saved Vs Legacy128 Total: %llu\n", (unsigned long long)demand_sectors_saved);
				printf("FTB Physical Fetch Control Storage Bytes/Core: 0\n");
				printf("FTB Sector Accounting: Required=encoded diagnostic length; Requested=Physical=entire allocation; no encoded length drives request generation\n");
			}
		} ftb_fetch;

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
				uint64_t prefetch_candidates;
				uint64_t prefetch_queued_sectors;
				uint64_t prefetch_dropped_sectors;
				uint64_t prefetch_issued_sectors;
				uint64_t node_fetch_ray_cycles;
				uint64_t tri_fetch_ray_cycles;
				uint64_t ttp_pop_states[3];
				uint64_t ttp_resets;
				uint64_t ttp_push_resets;
				uint64_t ttp_leaf_continuation_resets;
				uint64_t ttp_restart_resets;
				uint64_t ttp_trim_resets;
				uint64_t ttp_reuse_resets;
				uint64_t ttp_complete_resets;
				uint64_t ttp_scan_reads;
				uint64_t ttp_queue_retries;
				uint64_t ttp_queue_full;
				uint64_t ttp_filtered_type;
				uint64_t ttp_filtered_hit;
				uint64_t ttp_node_candidates;
				uint64_t ttp_leaf_candidates;
			};
			uint64_t counters[NUM_COUNTERS];
		};

		Log() { reset(); }

		void reset()
		{
			ftb_fetch.reset();
			ftb_basic_decode.reset();
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] = 0;
		}

		void accumulate(const Log& other)
		{
			ftb_fetch.accumulate(other.ftb_fetch);
			ftb_basic_decode.accumulate(other.ftb_basic_decode);
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
			printf("Node Fetch Ray-Cycles: %llu\n", static_cast<unsigned long long>(node_fetch_ray_cycles / num_units));
			printf("Tri Fetch Ray-Cycles: %llu\n", static_cast<unsigned long long>(tri_fetch_ray_cycles / num_units));
			printf("Prefetch Candidates: %llu\n", static_cast<unsigned long long>(prefetch_candidates / num_units));
			printf("Prefetch Queued Sectors: %llu\n", static_cast<unsigned long long>(prefetch_queued_sectors / num_units));
			printf("Prefetch Dropped Sectors: %llu\n", static_cast<unsigned long long>(prefetch_dropped_sectors / num_units));
			printf("Prefetch Issued Sectors: %llu\n", static_cast<unsigned long long>(prefetch_issued_sectors / num_units));
			if(ttp_resets || ttp_pop_states[0] || ttp_pop_states[1] || ttp_pop_states[2])
			{
				for(uint i = 0; i < 3; ++i)
					printf("TTP Pop S%u Total: %llu\n", i + 1, static_cast<unsigned long long>(ttp_pop_states[i]));
				printf("TTP Resets Total: %llu\n", static_cast<unsigned long long>(ttp_resets));
				printf("TTP Push Resets Total: %llu\n", static_cast<unsigned long long>(ttp_push_resets));
				printf("TTP Leaf Continuation Resets Total: %llu\n", static_cast<unsigned long long>(ttp_leaf_continuation_resets));
				printf("TTP Restart Resets Total: %llu\n", static_cast<unsigned long long>(ttp_restart_resets));
				printf("TTP Trim Resets Total: %llu\n", static_cast<unsigned long long>(ttp_trim_resets));
				printf("TTP Reuse Resets Total: %llu\n", static_cast<unsigned long long>(ttp_reuse_resets));
				printf("TTP Complete Resets Total: %llu\n", static_cast<unsigned long long>(ttp_complete_resets));
				printf("TTP Scan Reads Total: %llu\n", static_cast<unsigned long long>(ttp_scan_reads));
				printf("TTP Queue Retries Total: %llu\n", static_cast<unsigned long long>(ttp_queue_retries));
				printf("TTP Queue Full Total: %llu\n", static_cast<unsigned long long>(ttp_queue_full));
				printf("TTP Filtered Type Total: %llu\n", static_cast<unsigned long long>(ttp_filtered_type));
				printf("TTP Filtered Hit Total: %llu\n", static_cast<unsigned long long>(ttp_filtered_hit));
				printf("TTP Node Candidates Total: %llu\n", static_cast<unsigned long long>(ttp_node_candidates));
				printf("TTP Leaf Candidates Total: %llu\n", static_cast<unsigned long long>(ttp_leaf_candidates));
			}

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
			ftb_fetch.print();
			ftb_basic_decode.print();

		};
	}log;
};

}}}
