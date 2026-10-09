#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <ostream>
#include <stdexcept>
#include <vector>

namespace Arches { namespace Units { namespace TRaX {

// Host observation only. No member below is read by simulated scheduling,
// request generation, cache policy, or completion admission. Timestamp 0 means
// "not applicable/not observed"; real global cycles start at 1.
class FTBWaitDiagnostics
{
public:
	struct Access
	{
		bool active{false};
		uint64_t original_ray_id{~0ull}, leaf_ordinal{0};
		uint32_t slot{0}, port{0}, encoded_leaf_id{0}, canonical_ftb_id{~0u};
		uint64_t address{0}, begin{0}, admit{0};
		uint32_t physical_bytes{0}, required_bytes{0};
		uint8_t queued_mask{0}, issued_mask{0}, returned_mask{0};
		std::array<uint64_t, 4> enqueue{}, issue{}, returned{};
		std::array<uint64_t, 4> enqueue_depth{};
		std::array<uint8_t, 4> return_order{};
		uint32_t returns{0};
	};
	struct Ray
	{
		bool active{false};
		uint64_t original_ray_id{~0ull}, admission{0}, hit_return{0}, leaf_count{0};
		uint32_t slot{0};
		std::array<uint64_t, 8> phase{};
	};
	struct Totals
	{
		uint64_t rays{0}, blocks{0}, sectors{0}, errors{0};
		uint64_t total_wait{0}, initial_queue{0}, upfront_delivery{0};
		uint64_t completion_wait{0};
		uint64_t sector_queue{0}, sector_service{0};
		uint64_t return1_before0{0};
		uint64_t first_admission{~0ull}, last_admission{0}, last_hit{0};
		std::array<uint64_t, 256> return_order_hist{};
		void accumulate(const Totals& x)
		{
			rays += x.rays; blocks += x.blocks; sectors += x.sectors; errors += x.errors;
			total_wait += x.total_wait; initial_queue += x.initial_queue; upfront_delivery += x.upfront_delivery;
			completion_wait += x.completion_wait;
			sector_queue += x.sector_queue; sector_service += x.sector_service;
			return1_before0 += x.return1_before0;
			first_admission = std::min(first_admission, x.first_admission);
			last_admission = std::max(last_admission, x.last_admission); last_hit = std::max(last_hit, x.last_hit);
			for(uint32_t i = 0; i < return_order_hist.size(); ++i) return_order_hist[i] += x.return_order_hist[i];
		}
	};

private:
	std::vector<Access> _active;
	std::vector<Ray> _rays;
	std::vector<Access> _completed;
	std::vector<Ray> _completed_rays;
	uint64_t _errors{0};
	void check(bool condition) { if(!condition) ++_errors; }

public:
	explicit FTBWaitDiagnostics(uint32_t slots) : _active(slots), _rays(slots) {}
	void admission(uint32_t slot, uint64_t original_id, uint64_t cycle)
	{
		check(!_rays[slot].active && !_active[slot].active && original_id != ~0ull);
		_rays[slot] = Ray{};
		auto& r = _rays[slot]; r.active = true; r.slot = slot; r.original_ray_id = original_id; r.admission = cycle;
	}
	void phase(uint32_t slot, uint32_t phase)
	{
		if(_rays[slot].active) { check(phase < 8); if(phase < 8) ++_rays[slot].phase[phase]; }
	}
	void begin(uint32_t slot, uint32_t port, uint32_t leaf, uint32_t canonical, uint64_t address,
		uint32_t physical, uint64_t cycle)
	{
		check(_rays[slot].active && !_active[slot].active && canonical != ~0u);
		_active[slot] = Access{};
		auto& a = _active[slot]; a.active = true; a.original_ray_id = _rays[slot].original_ray_id;
		a.leaf_ordinal = _rays[slot].leaf_count++; a.slot = slot; a.port = port;
		a.encoded_leaf_id = leaf; a.canonical_ftb_id = canonical; a.address = address;
		a.physical_bytes = physical; a.begin = cycle;
	}
	void enqueue(uint32_t slot, uint32_t sector, uint64_t cycle, uint64_t depth)
	{
		auto& a = _active[slot]; const uint8_t bit = uint8_t(1u << sector);
		check(a.active && sector < 4 && !(a.queued_mask & bit));
		if(sector >= 4) return;
		a.queued_mask |= bit; a.enqueue[sector] = cycle; a.enqueue_depth[sector] = depth;
	}
	void issue(uint32_t slot, uint32_t sector, uint64_t cycle)
	{
		auto& a = _active[slot]; const uint8_t bit = uint8_t(1u << sector);
		check(a.active && sector < 4 && (a.queued_mask & bit) && !(a.issued_mask & bit));
		if(sector >= 4) return;
		a.issued_mask |= bit; a.issue[sector] = cycle;
	}
	void returned(uint32_t slot, uint32_t sector, uint64_t cycle)
	{
		auto& a = _active[slot]; const uint8_t bit = uint8_t(1u << sector);
		check(a.active && sector < 4 && (a.issued_mask & bit) && !(a.returned_mask & bit) && a.returns < 4);
		if(sector >= 4 || a.returns >= 4) return;
		a.returned_mask |= bit; a.returned[sector] = cycle; a.return_order[a.returns++] = uint8_t(sector);
	}
	const Access& active_access(uint32_t slot) const { return _active[slot]; }
	void complete(uint32_t slot, uint32_t required_bytes, uint64_t cycle)
	{
		auto& a = _active[slot]; check(a.active); a.required_bytes = required_bytes; a.admit = cycle;
		_completed.push_back(a); a.active = false;
	}
	void hit_return(uint32_t slot, uint64_t cycle)
	{
		auto& r = _rays[slot]; check(r.active && !_active[slot].active); r.hit_return = cycle;
		_completed_rays.push_back(r); r.active = false;
	}
	static void access_header(std::ostream& out)
	{
		out << "core,original_ray_id,leaf_ordinal,slot,port,encoded_leaf_id,canonical_ftb_id,address,physical_bytes,required_bytes,begin,header_return,request_set_ready,physical_data_ready,completion_ready,completion_admit,queued_mask,issued_mask,returned_mask,return_order_code,initial_queue,physical_delivery,completion_wait,total_wait,sector_queue_sum,sector_service_sum";
		for(uint32_t s = 0; s < 4; ++s) out << ",s" << s << "_enqueue,s" << s << "_issue,s" << s << "_return,s" << s << "_enqueue_depth";
		out << '\n';
	}
	static void ray_header(std::ostream& out)
	{
		out << "core,original_ray_id,slot,admission,hit_return,residence,leaf_count,phase_ray_fetch,phase_scheduler,phase_hit_return,phase_node_fetch,phase_tri_fetch,phase_vrt_fetch,phase_node_isect,phase_tri_isect\n";
	}
	Totals write(std::ostream& accesses, std::ostream& rays, uint32_t core) const
	{
		Totals t; t.errors = _errors;
		for(const auto& a : _active) t.errors += a.active;
		for(const auto& r : _rays) t.errors += r.active;
		for(const auto& a : _completed)
		{
			++t.blocks;
			uint64_t data = 0, queue_sum = 0, service_sum = 0, first_issue = ~0ull;
			uint32_t order = 0, requested = 0;
			for(uint32_t s = 0; s < 4; ++s)
			{
				if(!(a.queued_mask & (1u << s))) continue;
				++t.sectors; ++requested; data = std::max(data, a.returned[s]); first_issue = std::min(first_issue, a.issue[s]);
				t.errors += !(a.enqueue[s] == a.begin && a.enqueue[s] <= a.issue[s] && a.issue[s] <= a.returned[s] && a.returned[s] <= a.admit);
				queue_sum += a.issue[s] - a.enqueue[s]; service_sum += a.returned[s] - a.issue[s];
			}
			for(uint32_t i = 0; i < a.returns; ++i) order = (order << 2) | a.return_order[i];
			++t.return_order_hist[order]; t.return1_before0 += a.returned[1] < a.returned[0];
			// All physical requests exist at begin; no parser or dependent tail participates.
			const uint64_t queue = first_issue >= a.begin ? first_issue - a.begin : 0;
			const uint64_t delivery = data >= first_issue ? data - first_issue : 0;
			const uint64_t completion = a.admit >= data ? a.admit - data : 0;
			const uint64_t total = a.admit >= a.begin ? a.admit - a.begin : 0;
			const uint32_t physical_sectors = a.physical_bytes / 32;
			const uint32_t expected_mask = (a.physical_bytes == 64 || a.physical_bytes == 128) ? (1u << physical_sectors) - 1 : 0;
			t.errors += !expected_mask || a.queued_mask != expected_mask || a.issued_mask != expected_mask || a.returned_mask != expected_mask;
			t.errors += requested != physical_sectors || a.returns != physical_sectors;
			t.errors += !(a.required_bytes >= 32 && a.required_bytes <= a.physical_bytes && a.required_bytes % 32 == 0);
			t.errors += !(a.returned[0] && data <= a.admit && first_issue >= a.begin && data >= first_issue);
			t.errors += total != queue + delivery + completion;
			t.initial_queue += queue; t.upfront_delivery += delivery; t.completion_wait += completion; t.total_wait += total;
			t.sector_queue += queue_sum; t.sector_service += service_sum;
			accesses << core << ',' << a.original_ray_id << ',' << a.leaf_ordinal << ',' << a.slot << ',' << a.port << ',' << a.encoded_leaf_id << ',' << a.canonical_ftb_id << ',' << a.address << ',' << a.physical_bytes << ',' << a.required_bytes << ',' << a.begin << ',' << a.returned[0] << ',' << a.begin << ',' << data << ',' << data << ',' << a.admit << ',' << uint32_t(a.queued_mask) << ',' << uint32_t(a.issued_mask) << ',' << uint32_t(a.returned_mask) << ',' << order << ',' << queue << ',' << delivery << ',' << completion << ',' << total << ',' << queue_sum << ',' << service_sum;
			for(uint32_t s = 0; s < 4; ++s) accesses << ',' << a.enqueue[s] << ',' << a.issue[s] << ',' << a.returned[s] << ',' << a.enqueue_depth[s];
			accesses << '\n';
		}
		for(const auto& r : _completed_rays)
		{
			++t.rays; t.first_admission = std::min(t.first_admission, r.admission);
			t.last_admission = std::max(t.last_admission, r.admission); t.last_hit = std::max(t.last_hit, r.hit_return);
			uint64_t phases = 0; for(uint64_t x : r.phase) phases += x;
			t.errors += !(r.admission <= r.hit_return && phases == r.hit_return - r.admission);
			rays << core << ',' << r.original_ray_id << ',' << r.slot << ',' << r.admission << ',' << r.hit_return << ',' << r.hit_return - r.admission << ',' << r.leaf_count;
			for(uint64_t x : r.phase) rays << ',' << x;
			rays << '\n';
		}
		return t;
	}
};

}}}
