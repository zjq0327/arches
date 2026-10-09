#pragma once 
#include "stdafx.hpp"

#include "util/arbitration.hpp"
#include "unit-cache-base.hpp"

namespace Arches {
namespace Units {

class UnitCache : public UnitCacheBase
{
public:
	struct Configuration
	{
		uint level{0};
		
		bool miss_alloc{false};
		bool block_prefetch{false};
		UnitCacheBase::Policy policy{UnitCacheBase::Policy::LRU};

		uint size{1024};
		uint block_size{CACHE_BLOCK_SIZE};
		uint sector_size{CACHE_SECTOR_SIZE};
		uint associativity{1};

		uint num_mshr{1};
		uint num_subentries{1};
		uint pf_mshr_limit{8};
		uint return_queue_size{32};
		uint latency{1};

		uint num_ports{1};
		uint crossbar_width{1};
		uint num_slices{1};
		uint num_banks{1};


		uint64_t slice_select_mask;
		uint64_t bank_select_mask;

		std::vector<UnitMemoryBase*> mem_highers{nullptr};
		uint                         mem_higher_port{0};
		uint                         mem_higher_port_stride{1};
	};

	struct PowerConfig
	{
		//Energy is joules, power in watts
		float tag_energy{0.0f};
		float read_energy{0.0f};
		float write_energy{0.0f};
		float leakage_power{0.0f};
	};

	UnitCache(Configuration config);
	virtual ~UnitCache();

	void clock_rise() override;
	void clock_fall() override;

	bool request_port_write_valid(uint port_index) override;
	void write_request(const MemoryRequest& request) override;

	bool return_port_read_valid(uint port_index) override;
	const MemoryReturn& peek_return(uint port_index) override;
	const MemoryReturn read_return(uint port_index) override;

protected:
	struct Bank
	{
		//request path (per bank)
		FIFO<MemoryReturn> return_queue;
		LatencyFIFO<MemoryRequest> request_pipline;
		LatencyFIFO<MemoryReturn> return_pipline;
		Bank(Configuration config);
	};

	struct MSHR //Miss Status Handling Register
	{
		std::queue<MemoryRequest> subentries;
		bool prefetch{false};
		bool demand_seen{false};
		bool filled{false};
		uint64_t host_fill_token{0}; // FTB observation only; never used by cache control.
		MSHR() = default;
	};

	struct Slice
	{
		std::vector<Bank> banks;

		//miss path (per partition)
		Cascade<MemoryRequest> miss_network;
		std::map<paddr_t, MSHR> mshrs;

		std::queue<MemoryRequest> mem_higher_request_queue;
		std::queue<MemoryRequest> prefetch_request_queue;
		std::queue<MemoryRequest> prefetch_miss_queue;
		uint prefetch_mshrs{0};
		uint mem_higher_port;

		Slice(Configuration config);
	};

	std::vector<UnitMemoryBase*> _mem_highers;
	RequestCrossBar _request_network;

	std::vector<Slice> _slices;
	ReturnCrossBar _return_network;

	uint _level;
	uint _num_mshr;
	uint _num_subentries;
	uint _pf_mshr_limit;
	std::set<paddr_t> _prefetched_sectors;
	bool _block_prefetch;
	bool _miss_alloc;

	// All mutable trace state is owned by this unit's clock methods. External
	// write_request only stamps a local transaction copy; read_return is unchanged.
	struct MemoryPathRequest
	{
		uint64_t token{0}, address{0}, local_address{0};
		uint64_t entry{0}, accepted{0}, lookup_ready{0}, lookup{0};
		uint64_t miss_enqueued{0}, resolve{0}, fill_token{0}, fill_ready{0};
		uint64_t data_ready{0}, response_enqueue{0}, return_emit{0};
		uint size{0}, port{0};
		bool prefetch_origin{false}, initial_lookup_hit{false}, source_prefetch{false};
		std::string path;
	};
	struct MemoryPathFill
	{
		uint64_t token{0}, address{0}, local_address{0}, trigger_token{0};
		uint64_t allocated{0}, issue{0}, fill_ready{0}, retired{0};
		bool prefetch_origin{false};
		std::string source;
	};
	bool _memory_path_enabled{false};
	uint16_t _memory_path_unit{0};
	uint _memory_path_partition{0}, _memory_path_partitions{1}, _memory_path_stride{1};
	std::pair<paddr_t, paddr_t> _memory_path_ftb_range{0, 0};
	uint64_t _memory_path_sequence{0}, _memory_path_errors{0};
	std::vector<MemoryPathRequest> _memory_path_requests;
	std::vector<MemoryPathFill> _memory_path_fills;
	std::unordered_map<uint64_t, size_t> _memory_path_request_index, _memory_path_fill_index;
	uint64_t _memory_path_cycle() const;
	paddr_t _memory_path_global_address(paddr_t address) const;
	bool _memory_path_ftb(paddr_t address, uint size) const;
	void _memory_path_error();
	MemoryPathRequest* _memory_path_request(uint64_t token);
	void _memory_path_accept(const MemoryRequest& request);
	void _memory_path_lookup_ready(const MemoryRequest& request);
	void _memory_path_lookup(const MemoryRequest& request, bool hit);
	void _memory_path_miss_enqueue(const MemoryRequest& request);
	void _memory_path_hit(const MemoryRequest& request, const char* path);
	void _memory_path_attach(const MemoryRequest& request, const MSHR& mshr, const char* path);
	void _memory_path_new_fill(MSHR& mshr, MemoryRequest& fill, const MemoryRequest* trigger, const char* source);
	void _memory_path_fill_ready(MSHR& mshr, const MemoryReturn& ret);
	void _memory_path_response(const MemoryRequest& request);
	void _memory_path_emit(const MemoryReturn& ret);

public:
	struct MemoryPathTotals
	{
		uint64_t requests{0}, fills{0}, errors{0}, incomplete_requests{0}, incomplete_fills{0};
	};
	void configure_memory_path_diagnostics(bool enabled, uint16_t diagnostic_unit,
		std::pair<paddr_t, paddr_t> global_ftb_range, uint partition = 0, uint partitions = 1, uint stride = 1);
	static void memory_path_headers(std::ostream& requests, std::ostream& fills);
	MemoryPathTotals write_memory_path_diagnostics(std::ostream& requests, std::ostream& fills) const;

protected:

	uint _get_bank(paddr_t addr)
	{
		return (addr / _block_size) % _slices[0].banks.size();
	}

	void _recive_return();
	void _recive_request();
	void _send_request();
	bool _queue_prefetch(Slice& slice, const MemoryRequest& request);
	void _allocate_for_fill(paddr_t sector_addr);

	virtual UnitMemoryBase* _get_mem_higher(paddr_t addr) { return _mem_highers[0]; }

public:
	class Log
	{
	public:
		const static uint NUM_COUNTERS = 20;
		union
		{
			struct
			{
				uint64_t hits;
				uint64_t misses;
				uint64_t half_misses;
				uint64_t uncached_requests;
				uint64_t mshr_stalls;
				uint64_t bytes_read;
				uint64_t tag_array_access;
				uint64_t data_array_reads;
				uint64_t data_array_writes;
				uint64_t pf_received;
				uint64_t pf_cache_redundant;
				uint64_t pf_inflight_redundant;
				uint64_t pf_issued;
				uint64_t pf_drop;
				uint64_t pf_fill;
				uint64_t pf_timely_useful;
				uint64_t pf_late;
				uint64_t pf_unused_evicted;
				uint64_t pf_lookup_hits;
				uint64_t pf_stalls;
			};
			uint64_t counters[NUM_COUNTERS];
		};
		std::map<paddr_t, uint64_t> profile_counters;

	public:
		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] = 0;

			profile_counters.clear();
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] += other.counters[i];

			for(auto& a : other.profile_counters)
				profile_counters[a.first] += a.second;
		}

		uint64_t get_total() { return hits + half_misses + misses; }
		uint64_t get_total_data_array_accesses() { return data_array_reads + data_array_writes; }

		void print(cycles_t cycles, uint units = 1)
		{
			uint64_t total = get_total();
			printf("Total: %lld\n", total / units);
			printf("Hits: %lld (%.2f%%)\n", hits / units, 100.0f * hits / total);
			printf("Half Misses: %lld (%.2f%%)\n", half_misses / units, 100.0f * half_misses / total);
			printf("Misses: %lld (%.2f%%)\n", misses / units, 100.0f * misses / total);
			printf("\n");
			printf("Uncached Requests: %lld\n", uncached_requests / units);
			printf("\n");
			printf("MSHR Stalls: %lld\n", mshr_stalls / units);
			printf("\n");
			printf("Tag Array Access: %lld\n", tag_array_access / units);
			printf("Data Array Reads: %lld\n", data_array_reads / units);
			printf("Data Array Writes: %lld\n", data_array_writes / units);
			printf("Prefetch Received: %llu\n", static_cast<unsigned long long>(pf_received / units));
			printf("Prefetch Cache Redundant: %llu\n", static_cast<unsigned long long>(pf_cache_redundant / units));
			printf("Prefetch Inflight Redundant: %llu\n", static_cast<unsigned long long>(pf_inflight_redundant / units));
			printf("Prefetch Issued: %llu\n", static_cast<unsigned long long>(pf_issued / units));
			printf("Prefetch Dropped: %llu\n", static_cast<unsigned long long>(pf_drop / units));
			printf("Prefetch Filled: %llu\n", static_cast<unsigned long long>(pf_fill / units));
			printf("Prefetch Timely Useful: %llu\n", static_cast<unsigned long long>(pf_timely_useful / units));
			printf("Prefetch Late: %llu\n", static_cast<unsigned long long>(pf_late / units));
			printf("Prefetch Unused Evicted: %llu\n", static_cast<unsigned long long>(pf_unused_evicted / units));
			printf("Prefetch Lookup Hits: %llu\n", static_cast<unsigned long long>(pf_lookup_hits / units));
			printf("Prefetch Stalls: %llu\n", static_cast<unsigned long long>(pf_stalls / units));
		}

		void print_short(cycles_t cycles, uint units = 1)
		{
			uint64_t total = get_total();
			printf("Hits/Half/Misses: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * hits / total, 100.0 * half_misses / total, 100.0 * total);
		}

		float print_power(PowerConfig power_config, float time_delta, uint units = 1)
		{
			float read_energy = data_array_reads * power_config.read_energy / units;
			float write_energy = data_array_writes * power_config.write_energy / units;
			float tag_energy = tag_array_access * power_config.tag_energy / units;
			float leakage_energy = time_delta * power_config.leakage_power / units;

			float total_energy = read_energy + write_energy + tag_energy + leakage_energy;
			float total_power = total_energy / time_delta;

			printf("\n");
			printf("Total Energy: %.2f mJ\n", total_energy * 1000.0f);
			printf("Total Power: %.2f W\n", total_power);

			return total_power;
		}
	}
	log;
};

}
}