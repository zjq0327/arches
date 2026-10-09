#include "unit-rt-core.hpp"

namespace Arches { namespace Units { namespace TRaX {

//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 4)
//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 22 && ray_id == 0)

#ifndef ENABLE_RT_DEBUG_PRINTS 
#define ENABLE_RT_DEBUG_PRINTS (false)
#endif

template<typename NT, typename PT>
UnitRTCore<NT, PT>::UnitRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _node_base_addr(config.node_base_addr), _tri_base_addr(config.tri_base_addr), _vrt_base_addr(config.vrt_base_addr),
	_cache(config.cache), _request_network(config.num_clients, 1), _return_network(1, config.num_clients),
	_box_pipline(12), _tri_pipline(22), _cache_port(config.cache_port), _cache_port_stride(config.cache_port_stride),
	_stack_trend_prefetch(config.stack_trend_prefetch),
	_ttp_max_distance(config.ttp_max_distance), _ttp_leaf_prefetch(config.ttp_leaf_prefetch),
	_prefetch_queue_size(config.prefetch_queue_size),
	_canonical_ftb_observer(config.canonical_ftb_observer),
	_hit_observer(config.hit_observer), _ray_id_observer(config.ray_id_observer),
	_identified_hit_observer(config.identified_hit_observer)
{
	if(config.ftb_wait_diagnostics)
	{
		if(!(std::is_same_v<NT, rtm::HE2CWBVH::Node> && std::is_same_v<PT, rtm::FTB>) ||
			!_ray_id_observer || !_canonical_ftb_observer)
			throw std::invalid_argument("FTB wait diagnostics require HE2/FTB and host identity observers");
		_ftb_wait = std::make_unique<FTBWaitDiagnostics>(config.max_rays);
	}
	if(config.ftb_memory_diagnostics)
	{
		if(!_ftb_wait) throw std::invalid_argument("FTB memory diagnostics require FTB wait diagnostics");
		_memory_path = std::make_unique<MemoryPathOrigins>(config.memory_path_unit_id);
	}
	log.ftb_fetch.enabled = _compact_ftb;
	log.ftb_basic_decode.enabled = _compact_ftb;
	log.ftb_basic_decode.resident_slots = config.max_rays;
	log.ftb_basic_decode.ports = config.num_cache_ports;
	_assert(_ttp_max_distance >= 2 && _ttp_max_distance <= RayState::STACK_SIZE);
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
	{
		_ray_states[i].phase = RayState::Phase::RAY_FETCH;
		_free_ray_ids.insert(i);
	}

	_cache_fetch_queues.resize(config.num_cache_ports);
	if(_compact_ftb)
	{
		_ftb_decode_countdown.resize(config.max_rays, 0);
		_ftb_decode_ready_counts.resize(config.num_cache_ports, 0); // Host observation, not target arbitration.
		if(_ftb_wait) _ftb_basic_index.resize(config.max_rays, ~size_t(0));
	}
}
template<typename NT, typename PT>
void UnitRTCore<NT, PT>::clock_rise()
{
	if(_compact_ftb)
	{
		++_ftb_decode_cycle;
	}
	_prefetch_stack_read_used = false;
	if(_ftb_wait)
	{
		++_ftb_wait_cycle;
		for(uint ray_id = 0; ray_id < _ray_states.size(); ++ray_id)
			_ftb_wait->phase(ray_id, static_cast<uint>(_ray_states[ray_id].phase));
	}
	// Phase occupancy is sampled once per global cycle, not once per scheduler subcycle.
	for(const RayState& ray_state : _ray_states)
	{
		if(ray_state.phase == RayState::Phase::NODE_FETCH) log.node_fetch_ray_cycles++;
		if(ray_state.phase == RayState::Phase::TRI_FETCH) log.tri_fetch_ray_cycles++;
	}

	_request_network.clock();
	_read_requests();
	// Advance old admissions once per global cycle, before any new data returns.
	if(_compact_ftb) _advance_ftb_basic_decode();
	_read_returns();

	//n stack ops per cycle. In reality this would need to be multi banked
	for(uint i = 0; i < 2; ++i)
	{
		_schedule_ray();
		_simualte_node_pipline();
		_simualte_tri_pipline();
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::clock_fall()
{
	// The additional read-only stack port reads at most one candidate per global cycle.
	if(_stack_trend_prefetch) _scan_stack_trend_candidate();
	_issue_requests();
	_issue_returns();
	_return_network.clock();
}

template<typename NT, typename PT>
bool UnitRTCore<NT, PT>::_try_queue_node(uint ray_id, uint node_id)
{
	paddr_t start = _node_base_addr + node_id * sizeof(NT);
	paddr_t end = start + sizeof(NT);

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;
	ray_state.buffer.type = 0;
	ray_state.buffer.id = node_id;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _align_address(addr + MemoryRequest::MAX_SIZE));

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.paddr = addr;
		req.size = next_boundry - addr;
		req.dst.push(ray_id, 10);
		_cache_fetch_queues[ray_id % _cache_fetch_queues.size()].push(req);

		addr += req.size;
	}

	return true;
}

template<typename NT, typename PT>
paddr_t UnitRTCore<NT, PT>::_primitive_address(uint encoded_id) const
{
	if(_compact_ftb)
		return _tri_base_addr + static_cast<paddr_t>(rtm::compact_ftb::leaf_slot(encoded_id)) * 64;
	if(typeid(NT) == typeid(rtm::HECWBVH::Node))
		return _node_base_addr + encoded_id * sizeof(NT);
	return _tri_base_addr + encoded_id * sizeof(PT);
}

template<typename NT, typename PT>
uint UnitRTCore<NT, PT>::_primitive_bytes(uint encoded_id) const
{
	return _compact_ftb ? rtm::compact_ftb::leaf_bytes(encoded_id) : sizeof(PT);
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_record_ftb_completion(uint ray_id, uint required_bytes, uint requested_sectors)
{
	const uint physical = _primitive_bytes(_ray_states[ray_id].buffer.id) / MemoryRequest::MAX_SIZE;
	_assert(requested_sectors == physical);
	++log.ftb_fetch.completed_blocks;
	++log.ftb_fetch.required_sectors[required_bytes / MemoryRequest::MAX_SIZE - 1];
	++log.ftb_fetch.requested_sectors[requested_sectors - 1];
	++log.ftb_fetch.physical_blocks[physical / 2 - 1];
	log.ftb_fetch.demand_physical_sectors += physical;
	log.ftb_fetch.demand_sectors_saved += sizeof(rtm::FTB) / MemoryRequest::MAX_SIZE - requested_sectors;
}

template<typename NT, typename PT>
bool UnitRTCore<NT, PT>::_try_queue_tri(uint ray_id, uint tri_id)
{
	if(_compact_ftb)
	{
		_assert(_ftb_decode_countdown[ray_id] == 0);
		if(!_ftb_basic_index.empty()) _ftb_basic_index[ray_id] = ~size_t(0);
	}
	const paddr_t start = _primitive_address(tri_id);
	const uint physical_bytes = _primitive_bytes(tri_id);
	paddr_t end = start + physical_bytes;

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;
	ray_state.buffer.type = 1;
	ray_state.buffer.id = tri_id;
	if(_ftb_wait) _ftb_wait->begin(ray_id, ray_id % _cache_fetch_queues.size(), tri_id,
		_canonical_ftb_observer(tri_id), start, physical_bytes, _ftb_wait_cycle);
	if(_compact_ftb)
	{
		// Preserve the original128B staging capacity and never expose stale short-block tails.
		std::memset(&ray_state.buffer.prim, 0, sizeof(PT));
		log.ftb_fetch.enabled = true;
		_assert(start % MemoryRequest::MAX_SIZE == 0 && (physical_bytes == 64 || physical_bytes == 128));
	}

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _align_address(addr + MemoryRequest::MAX_SIZE));

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.paddr = addr;
		req.size = next_boundry - addr;
		req.dst.push(ray_id, 10);
		_cache_fetch_queues[ray_id % _cache_fetch_queues.size()].push(req);
		if(_ftb_wait) _ftb_wait->enqueue(ray_id, static_cast<uint>((addr - start) / MemoryRequest::MAX_SIZE),
			_ftb_wait_cycle, _cache_fetch_queues[ray_id % _cache_fetch_queues.size()].size() - 1);
		if(_compact_ftb) ++log.ftb_fetch.demand_sectors_requested;

		addr += req.size;
	}

	return true;
}

template<typename NT, typename PT>
bool UnitRTCore<NT, PT>::_try_queue_prefetch(uint ray_id, paddr_t addr, uint size, uint cache_mask)
{
	if(size == 0) return true;
	std::vector<paddr_t> missing_sectors;
	const paddr_t end = addr + size;
	for(paddr_t sector = _align_address(addr); sector < end; sector += MemoryRequest::MAX_SIZE)
	{
		if(_queued_prefetch_sectors.find(sector) == _queued_prefetch_sectors.end())
			missing_sectors.push_back(sector);
	}

	// A node is admitted as a whole, so a full 64-byte node needs two queue slots.
	if(missing_sectors.size() > _prefetch_queue_size - _prefetch_queue.size())
	{
		// TTP retains the candidate for retry without advancing its cursor.
		return false;
	}

	const uint64_t epoch = _ray_states[ray_id].prefetch_epoch;
	for(paddr_t sector : missing_sectors)
	{
		_prefetch_queue.push_back({sector, ray_id, epoch, static_cast<uint8_t>(cache_mask)});
		_queued_prefetch_sectors.insert(sector);
	}
	log.prefetch_queued_sectors += missing_sectors.size();
	return true;
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_invalidate_prefetch(uint ray_id)
{
	if(!_prefetch_enabled()) return;
	RayState& ray_state = _ray_states[ray_id];
	++ray_state.prefetch_epoch;

	for(auto it = _prefetch_queue.begin(); it != _prefetch_queue.end();)
	{
		if(it->ray_id != ray_id)
		{
			++it;
			continue;
		}
		_queued_prefetch_sectors.erase(it->addr);
		it = _prefetch_queue.erase(it);
		log.prefetch_dropped_sectors++;
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_reset_stack_trend_prefetch(uint ray_id)
{
	RayState& ray_state = _ray_states[ray_id];
	_invalidate_prefetch(ray_id);
	ray_state.ttp_snapshot_valid = false;
	ray_state.ttp_state = 0;
	ray_state.ttp_cursor = static_cast<int16_t>(ray_state.stack_size) - 1;
	ray_state.ttp_floor = ray_state.stack_size;
	ray_state.ttp_pending_push = false;
	ray_state.ttp_pending_trim = false;
	++log.ttp_resets;
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_queue_stack_trend_scan(uint ray_id)
{
	RayState& ray_state = _ray_states[ray_id];
	bool& queued = ray_state.ttp_scan_queued;
	if(ray_state.ttp_cursor >= ray_state.ttp_floor && !queued)
	{
		_ttp_scan_queue.push(ray_id);
		queued = true;
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_stack_trend_pop(uint ray_id)
{
	RayState& ray_state = _ray_states[ray_id];
	ray_state.ttp_state = static_cast<uint8_t>(std::min(3u, static_cast<uint>(ray_state.ttp_state) + 1));
	++log.ttp_pop_states[ray_state.ttp_state - 1];
	const uint distance = ray_state.ttp_state < 3 ? ray_state.ttp_state : _ttp_max_distance;
	ray_state.ttp_floor = static_cast<int16_t>(std::max(0, static_cast<int>(ray_state.stack_size) - static_cast<int>(distance)));
	const int16_t top = static_cast<int16_t>(ray_state.stack_size) - 1;
	if(ray_state.ttp_cursor > top)
	{
		// A pending candidate that has become this pop's demand is no longer speculative.
		ray_state.ttp_cursor = top;
		ray_state.ttp_snapshot_valid = false;
	}
	_queue_stack_trend_scan(ray_id);
}

template<typename NT, typename PT>
bool UnitRTCore<NT, PT>::_scan_stack_trend_candidate()
{
	std::queue<uint>& scan_queue = _ttp_scan_queue;
	const uint pending = static_cast<uint>(scan_queue.size());
	for(uint scanned = 0; scanned < pending; ++scanned)
	{
		const uint ray_id = scan_queue.front();
		scan_queue.pop();
		RayState& ray_state = _ray_states[ray_id];
		ray_state.ttp_scan_queued = false;
		if(ray_state.phase == RayState::Phase::RAY_FETCH || ray_state.phase == RayState::Phase::HIT_RETURN) continue;
		// The simulator computes intersection results before their latency FIFO retires.
		// Do not observe its early stack or hit writes while either pipeline is active.
		if(ray_state.phase == RayState::Phase::NODE_ISECT || ray_state.phase == RayState::Phase::TRI_ISECT)
		{
			_queue_stack_trend_scan(ray_id);
			continue;
		}
		if(ray_state.ttp_cursor < ray_state.ttp_floor) continue;
		_assert(ray_state.ttp_cursor >= 0 && ray_state.ttp_cursor < ray_state.stack_size);

		bool& snapshot_valid = ray_state.ttp_snapshot_valid;
		StackEntry& snapshot = ray_state.ttp_snapshot;
		const bool retry = snapshot_valid;
		if(retry) ++log.ttp_queue_retries;
		else
		{
			if(_prefetch_stack_read_used)
			{
				_queue_stack_trend_scan(ray_id);
				continue;
			}
			snapshot = ray_state.stack[ray_state.ttp_cursor];
			snapshot_valid = true;
			_prefetch_stack_read_used = true;
			++log.ttp_scan_reads;
		}

		const StackEntry& entry = snapshot;
		bool advance = true;
		if(!entry.data.is_int && !_ttp_leaf_prefetch) ++log.ttp_filtered_type;
		else if(!(entry.t < ray_state.hit.t)) ++log.ttp_filtered_hit;
		else
		{
			if(!retry)
			{
				++log.prefetch_candidates;
				if(entry.data.is_int) ++log.ttp_node_candidates;
				else ++log.ttp_leaf_candidates;
			}
			paddr_t addr;
			uint size;
			if(entry.data.is_int)
			{
				addr = _node_base_addr + entry.data.child_idx * sizeof(NT);
				size = sizeof(NT);
			}
			else
			{
				// Legacy entries can describe several blocks. Compact entries
				// describe one 64/128B allocation, whose size flag stays in prim_idx.
				addr = _primitive_address(entry.data.prim_idx);
				size = _primitive_bytes(entry.data.prim_idx);
			}
			advance = _try_queue_prefetch(ray_id, addr, size, 0);
			if(!advance) ++log.ttp_queue_full;
		}
		if(advance)
		{
			--ray_state.ttp_cursor;
			snapshot_valid = false;
		}
		_queue_stack_trend_scan(ray_id);
		return true; // A retry or filtered entry also consumes this cycle's candidate opportunity.
	}
	return false;
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_read_requests()
{
	//simulate waves of rays
	//if(_free_ray_ids.size() == _max_rays) _drain_phase = false;

	if(!_drain_phase && !_free_ray_ids.empty() && _request_network.is_read_valid(0))
	{
		//creates a ray entry and queue up the ray
		const MemoryRequest request = _request_network.read(0);

		uint ray_id = *_free_ray_ids.begin();
		_free_ray_ids.erase(ray_id);

		RayState& ray_state = _ray_states[ray_id];
		std::memcpy(&ray_state.ray, request.data, sizeof(rtm::Ray));
		ray_state.original_ray_id = _ray_id_observer ? _ray_id_observer(request) : ~0ull;
		if(_ftb_wait) _ftb_wait->admission(ray_id, ray_state.original_ray_id, _ftb_wait_cycle);
		ray_state.inv_d = rtm::vec3(1.0f) / ray_state.ray.d;
		ray_state.hit.t = ray_state.ray.t_max;
		ray_state.hit.bc = rtm::vec2(0.0f);
		ray_state.hit.id = ~0u;
		ray_state.stack[0].t = ray_state.ray.t_min;
		ray_state.stack[0].data.is_int = 1;
		ray_state.stack[0].data.child_idx = 0;
		ray_state.stack[0].is_last = false;
		ray_state.stack_size = 1;
		ray_state.level = 0;
		ray_state.update_restart_trail = false;
		ray_state.restart_trail = rtm::RestartTrail();
		ray_state.flags = request.flags;
		ray_state.dst = request.dst;
		ray_state.dst.push(request.port, 8);
		ray_state.phase = RayState::Phase::SCHEDULER;
		if(_stack_trend_prefetch)
		{
			_reset_stack_trend_prefetch(ray_id);
			++log.ttp_reuse_resets;
		}
		_ray_scheduling_queue.push(ray_id);

		log.rays++;

		//_ray_return_queue.push(ray_id);
	}

	//if(_free_ray_ids.empty()) _drain_phase = true;

	_stall_cycles++;
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_queue_tri_isect(uint ray_id)
{
	_ray_states[ray_id].phase = RayState::Phase::TRI_ISECT;
	if(_compact_ftb)
	{
		_assert(_ftb_decode_countdown[ray_id] == 0);
		_ftb_decode_countdown[ray_id] = FTB_BASIC_DECODE_LATENCY;
		auto& diagnostic = log.ftb_basic_decode;
		diagnostic.enabled = true; diagnostic.resident_slots = _max_rays;
		diagnostic.ports = static_cast<uint>(_cache_fetch_queues.size());
		++diagnostic.started;
		if(!_ftb_basic_index.empty())
		{
			const auto& access = _ftb_wait->active_access(ray_id);
			_ftb_basic_index[ray_id] = _ftb_basic_accesses.size();
			_ftb_basic_accesses.push_back({access.original_ray_id, access.leaf_ordinal,
				access.canonical_ftb_id, access.port, ray_id, _ftb_decode_cycle, 0, 0});
		}
	}
	_tri_isect_queue.push(ray_id);
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_advance_ftb_basic_decode()
{
	// Constant latency preserves the original return-port II1: each port admits
	// at most one full block per cycle, so at most one becomes ready two cycles later.
	// Per-port counts are observation only; no separate hardware completion arbiter exists.
	std::fill(_ftb_decode_ready_counts.begin(), _ftb_decode_ready_counts.end(), uint8_t(0));
	for(uint ray_id = 0; ray_id < _ftb_decode_countdown.size(); ++ray_id)
	{
		auto& countdown = _ftb_decode_countdown[ray_id];
		if(!countdown) continue;
		_assert(_ray_states[ray_id].phase == RayState::Phase::TRI_ISECT);
		++log.ftb_basic_decode.service_ray_cycles;
		if(--countdown == 0)
		{
			++log.ftb_basic_decode.completed;
			const uint port = ray_id % _cache_fetch_queues.size();
			const uint ready_count = ++_ftb_decode_ready_counts[port];
			_assert(ready_count == 1);
			log.ftb_basic_decode.ready_per_port_cycle_max = std::max<uint64_t>(log.ftb_basic_decode.ready_per_port_cycle_max, ready_count);
			if(!_ftb_basic_index.empty()) _ftb_basic_accesses[_ftb_basic_index[ray_id]].decode_ready = _ftb_decode_cycle;
		}
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_read_returns()
{
	for(uint i = 0; i < _cache_fetch_queues.size(); ++i)
	{
		uint port = _cache_port + i * _cache_port_stride;
		if(_cache->return_port_read_valid(port))
		{
			MemoryReturn ret = _cache->read_return(port);
			uint16_t ray_id = ret.dst.pop(10);
			RayState& ray_state = _ray_states[ray_id];
			StagingBuffer& buffer = ray_state.buffer;

			//if(ENABLE_RT_DEBUG_PRINTS) printf("ret %xll\n", ret.paddr);

			if(buffer.type == 0)
			{
				uint offset = (ret.paddr - buffer.address);
				std::memcpy((uint8_t*)&buffer.data + offset, ret.data, ret.size);
				buffer.bytes_filled += ret.size;
				if(buffer.bytes_filled == sizeof(NT))
				{
					ray_state.phase = RayState::Phase::NODE_ISECT;
					_node_isect_queue.push(ray_id);
				}
			}
			else if(buffer.type == 1)
			{
				if constexpr(std::is_same_v<PT, rtm::FTB>)
				{
					if(_compact_ftb)
					{
						if(ray_state.phase != RayState::Phase::TRI_FETCH) continue;
						const uint physical_bytes = _primitive_bytes(buffer.id);
						if(ret.paddr < buffer.address || ret.paddr >= buffer.address + physical_bytes)
							throw std::runtime_error("FTB return lies outside its physical allocation");
						const uint offset = static_cast<uint>(ret.paddr - buffer.address);
						if(offset % MemoryRequest::MAX_SIZE || ret.size != MemoryRequest::MAX_SIZE)
							throw std::runtime_error("FTB return violates aligned physical fetch accounting");
						const uint sector = offset / MemoryRequest::MAX_SIZE;
						const uint bit = 1u << sector;
						if(buffer.bytes_filled & bit) throw std::runtime_error("FTB physical fetch received a duplicate sector");
						if(_ftb_wait) _ftb_wait->returned(ray_id, sector, _ftb_wait_cycle);
						if(_memory_path) _memory_path->returned(ret.host_trace_token, ret.paddr, _ftb_wait_cycle);
						// Reuse the existing counter as a mask, so repeated data cannot complete a missing sector.
						std::memcpy((uint8_t*)&buffer.data + offset, ret.data, ret.size);
						buffer.bytes_filled |= bit;
						++log.ftb_fetch.demand_sectors_returned;
						const uint expected_mask = (1u << (physical_bytes / MemoryRequest::MAX_SIZE)) - 1;
						if(buffer.bytes_filled == expected_mask)
						{
							const uint required = rtm::compact_ftb::required_bytes(buffer.prim); // Validation/observation only.
							if(required > physical_bytes) throw std::runtime_error("FTB encoded length exceeds physical allocation");
							_record_ftb_completion(ray_id, required, physical_bytes / MemoryRequest::MAX_SIZE);
							if(_ftb_wait) _ftb_wait->complete(ray_id, required, _ftb_wait_cycle);
							_queue_tri_isect(ray_id);
						}
						continue;
					}
				}
				uint offset = (ret.paddr - buffer.address);
				std::memcpy((uint8_t*)&buffer.data + offset, ret.data, ret.size);
				buffer.bytes_filled += ret.size;
				if(buffer.bytes_filled == _primitive_bytes(buffer.id)) _queue_tri_isect(ray_id);
			}
		}
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_schedule_ray()
{
	//pop a entry from next rays stack and queue it up
	if(!_ray_scheduling_queue.empty())
	{
		_stall_cycles = 0;
		const uint ray_id = _ray_scheduling_queue.front();
		_ray_scheduling_queue.pop();

		RayState& ray_state = _ray_states[ray_id];

		StackEntry entry;
		bool actual_pop = true;
		if(ray_state.update_restart_trail)
		{
			uint parent_level = ray_state.restart_trail.find_parent_level(ray_state.level);
			if(parent_level == ~0u)
			{
				//Ray complete
				//stack empty or anyhit found return the hit
				ray_state.phase = RayState::Phase::HIT_RETURN;
				if(_stack_trend_prefetch)
				{
					_reset_stack_trend_prefetch(ray_id);
					++log.ttp_complete_resets;
				}
				_ray_return_queue.push(ray_id);

				log.issue_counters[(uint)IssueType::HIT_RETURN]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d HIT_RETURN: %d\n", ray_id, ray_state.hit.id);

				return;
			}

			ray_state.restart_trail.set(parent_level, ray_state.restart_trail.get(parent_level) + 1);
			ray_state.restart_trail.clear(parent_level + 1);

			if(ray_state.stack_size == 0)
			{
				//Restart
				entry.t = ray_state.ray.t_min;
				entry.data.is_int = 1;
				entry.data.child_idx = 0;
				ray_state.level = 0;
				actual_pop = false;
				if(_stack_trend_prefetch)
				{
					_reset_stack_trend_prefetch(ray_id);
					++log.ttp_restart_resets;
				}
				log.restarts++;
			}
			else
			{
				entry = ray_state.stack[--ray_state.stack_size];
				if(entry.is_last)
					ray_state.restart_trail.set(parent_level, rtm::RestartTrail::N);
				ray_state.level = parent_level + 1;
			}
		}
		else
		{
			_assert(ray_state.stack_size > 0);
			entry = ray_state.stack[--ray_state.stack_size];
		}

		ray_state.update_restart_trail = true;
		if(_stack_trend_prefetch && actual_pop) _stack_trend_pop(ray_id);

		if(!_pop_culling || entry.t < ray_state.hit.t)
		{
			if(entry.data.is_int)
			{
				_try_queue_node(ray_id, entry.data.child_idx);
				ray_state.phase = RayState::Phase::NODE_FETCH;

				log.issue_counters[(uint)IssueType::NODE_FETCH]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d NODE_FETCH: %d\n", ray_id, entry.data.child_idx);
			}
			else
			{
				if(_compact_ftb && entry.data.prim_cnt != 1)
					throw std::runtime_error("Compact FTB leaf must contain exactly one encoded allocation");
				_try_queue_tri(ray_id, entry.data.prim_idx);
				if(entry.data.prim_cnt > 1)
				{
					entry.data.prim_cnt--;
					entry.data.prim_idx++;
					ray_state.stack[ray_state.stack_size++] = entry;
					ray_state.update_restart_trail = false;
					if(_stack_trend_prefetch)
					{
						_reset_stack_trend_prefetch(ray_id);
						++log.ttp_push_resets;
						++log.ttp_leaf_continuation_resets;
					}
				}

				ray_state.phase = RayState::Phase::TRI_FETCH;

				log.issue_counters[(uint)IssueType::TRI_FETCH]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d TRI_FETCH: %d:%d\n", ray_id, entry.data.prim_idx, entry.data.prim_cnt);
			}
		}
		else //pop cull
		{
			_ray_scheduling_queue.push(ray_id);

			log.issue_counters[(uint)IssueType::POP_CULL]++;
			if(ENABLE_RT_DEBUG_PRINTS)
				printf("%03d POP_CULL\n", ray_id);
		}
	}
	else
	{
		uint phase = (uint)_ray_states[_last_ray_id].phase;
		if(++_last_ray_id == _ray_states.size()) _last_ray_id = 0;
		log.stall_counters[phase]++;
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_simualte_node_pipline()
{
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		_stall_cycles = 0;
		uint ray_id = _node_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];

		rtm::Hit& hit = ray_state.hit;
		const rtm::Ray& ray = ray_state.ray;
		const rtm::vec3& inv_d = ray_state.inv_d;

		rtm::BVH::Node nodes[32];
		uint node_count;
		if constexpr(std::is_same_v<NT, rtm::HE2CWBVH::Node> && std::is_same_v<PT, rtm::FTB>)
			node_count = _compact_ftb ? rtm::compact_ftb::decompress(ray_state.buffer.node, nodes) :
				rtm::decompress(ray_state.buffer.node, nodes);
		else node_count = rtm::decompress(ray_state.buffer.node, nodes);

		_box_issue_count += 8;
		if(_box_issue_count >= node_count)
		{
			uint k = ray_state.restart_trail.get(ray_state.level);

			uint nodes_pushed = 0, last_ptr = ~0u, last_j = ~0u;
			for(uint i = 0; i < node_count; i++)
			{
				float t = rtm::intersect(nodes[i].aabb, ray, inv_d);
				if(t < hit.t)
				{
					if(nodes[i].ptr.raw == last_ptr)
					{
						if(ray_state.stack[last_j].t <= t)
							continue;

						ray_state.stack[last_j].t = t;
						for(uint j = last_j; j < ray_state.stack_size + nodes_pushed - 1; ++j)
						{
							if(ray_state.stack[j + 1].t <= ray_state.stack[j].t) break;
							std::swap(ray_state.stack[j], ray_state.stack[j + 1]);
						}
						continue;
					}

					uint j = ray_state.stack_size + nodes_pushed++;
					for(; j > ray_state.stack_size; --j)
					{
						if(ray_state.stack[j - 1].t > t) break;
						ray_state.stack[j] = ray_state.stack[j - 1];
					}

					ray_state.stack[j].t = t;
					ray_state.stack[j].is_last = false;
					ray_state.stack[j].data = nodes[i].ptr;
					last_ptr = nodes[i].ptr.raw;
					last_j = j;
				}
			}

			if(k == rtm::RestartTrail::N) nodes_pushed = 1;
			else                          nodes_pushed -= std::min(nodes_pushed, k);

			if(nodes_pushed > 0)
			{
				ray_state.update_restart_trail = false;
				if(nodes_pushed == 1) ray_state.restart_trail.set(ray_state.level, rtm::RestartTrail::N);
				else                  ray_state.stack[ray_state.stack_size].is_last = true;
				ray_state.stack_size += nodes_pushed;
				ray_state.level++;

				if(ray_state.stack_size > RayState::STACK_SIZE)
				{
					if(_stack_trend_prefetch) ray_state.ttp_pending_trim = true;
					uint drain_count = ray_state.stack_size - RayState::STACK_SIZE;
					for(uint i = 0; i < RayState::STACK_SIZE; ++i)
						ray_state.stack[i] = ray_state.stack[i + drain_count];
					ray_state.stack_size = RayState::STACK_SIZE;
				}
				if(_stack_trend_prefetch) ray_state.ttp_pending_push = true;
			}

			_box_pipline.write(ray_id);
			_node_isect_queue.pop();
			_box_issue_count = 0;
		}
		else
		{
			_box_pipline.write(~0);
		}
	}

	_box_pipline.clock();

	if(_box_pipline.is_read_valid())
	{
		uint ray_id = _box_pipline.read();
		if(ray_id != ~0u)
		{
			RayState& ray_state = _ray_states[ray_id];
			if(_stack_trend_prefetch && ray_state.ttp_pending_push)
			{
				const bool trimmed = ray_state.ttp_pending_trim;
				_reset_stack_trend_prefetch(ray_id);
				++log.ttp_push_resets;
				if(trimmed) ++log.ttp_trim_resets;
			}
			ray_state.phase = RayState::Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);
			log.nodes++;
		}
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_simualte_tri_pipline()
{
	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid() &&
		(!_compact_ftb || _ftb_decode_countdown[_tri_isect_queue.front()] == 0))
	{
		_stall_cycles = 0;
		uint ray_id = _tri_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;
		if(_compact_ftb && _tri_issue_count == 0)
		{
			++log.ftb_basic_decode.first_issues;
			if(!_ftb_basic_index.empty()) _ftb_basic_accesses[_ftb_basic_index[ray_id]].first_tri_issue = _ftb_decode_cycle;
		}

		rtm::IntersectionTriangle tris[rtm::FTB::MAX_TRIS];
		// Functional reconstruction already understands the encoded common prefixes.
		// The separate fixed baseline gate above charges its abstract target delay.
		const uint tri_count = rtm::decompress(buffer.prim, tris);

		_tri_issue_count += 1;
		if(_tri_issue_count >= tri_count)
		{
			rtm::Ray& ray = ray_state.ray;
			rtm::vec3& inv_d = ray_state.inv_d;
			rtm::Hit& hit = ray_state.hit;

			for(uint i = 0; i < tri_count; ++i)
				if(rtm::intersect(tris[i].tri, ray, hit))
					hit.id = tris[i].id;

			_tri_pipline.write(ray_id);
			_tri_isect_queue.pop();
			_tri_issue_count = 0;
			log.strips++;
			log.tris += tri_count;
		}
		else
		{
			_tri_pipline.write(~0u);
		}
	}

	_tri_pipline.clock();

	if(_tri_pipline.is_read_valid())
	{
		uint ray_id = _tri_pipline.read();
		if(ray_id != ~0u)
		{
			RayState& ray_state = _ray_states[ray_id];
			ray_state.phase = RayState::Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);
		}

	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_issue_requests()
{
	std::vector<uint8_t> demand_selected;
	if(_prefetch_enabled()) demand_selected.resize(_cache_fetch_queues.size(), 0);
	for(uint i = 0; i < _cache_fetch_queues.size(); ++i)
	{
		uint port = _cache_port + i * _cache_port_stride;
		if(!_cache_fetch_queues[i].empty() && _cache->request_port_write_valid(port))
		{
			_cache_fetch_queues[i].front().port = port;
			if(_memory_path)
			{
				auto& request = _cache_fetch_queues[i].front();
				auto dst = request.dst;
				const uint ray_id = dst.pop(10);
				const auto& state = _ray_states[ray_id];
				if(state.phase == RayState::Phase::TRI_FETCH && state.buffer.type == 1)
					request.host_trace_token = _memory_path->issue(_ftb_wait->active_access(ray_id),
						static_cast<uint>((request.paddr - state.buffer.address) / MemoryRequest::MAX_SIZE), request.paddr, _ftb_wait_cycle);
			}
			_cache->write_request(_cache_fetch_queues[i].front());
			if(_ftb_wait)
			{
				const MemoryRequest& request = _cache_fetch_queues[i].front();
				auto dst = request.dst;
				const uint ray_id = dst.pop(10);
				const auto& state = _ray_states[ray_id];
				if(state.phase == RayState::Phase::TRI_FETCH && state.buffer.type == 1)
					_ftb_wait->issue(ray_id, static_cast<uint>((request.paddr - state.buffer.address) / MemoryRequest::MAX_SIZE), _ftb_wait_cycle);
			}
			_cache_fetch_queues[i].pop();
			if(_prefetch_enabled()) demand_selected[i] = 1;
		}
	}

	// Demand has selected all ports before any prefetch is considered.
	if(!_prefetch_enabled() || _prefetch_queue.empty()) return;
	while(!_prefetch_queue.empty())
	{
		const PrefetchItem& item = _prefetch_queue.front();
		if(item.epoch == _ray_states[item.ray_id].prefetch_epoch) break;
		_queued_prefetch_sectors.erase(item.addr);
		_prefetch_queue.pop_front();
		log.prefetch_dropped_sectors++;
	}
	if(_prefetch_queue.empty()) return;

	for(uint offset = 0; offset < _cache_fetch_queues.size(); ++offset)
	{
		const uint i = (_next_prefetch_cache_port + offset) % _cache_fetch_queues.size();
		if(demand_selected[i] || !_cache_fetch_queues[i].empty()) continue;
		const uint port = _cache_port + i * _cache_port_stride;
		if(!_cache->request_port_write_valid(port)) continue;

		MemoryRequest request;
		request.type = MemoryRequest::Type::PREFECTH;
		request.paddr = _prefetch_queue.front().addr;
		request.size = MemoryRequest::MAX_SIZE;
		request.flags.omit_cache = _prefetch_queue.front().cache_mask;
		request.port = port;
		std::memset(request.data, 0, request.size);
		_cache->write_request(request);
		_queued_prefetch_sectors.erase(request.paddr);
		_prefetch_queue.pop_front();
		_next_prefetch_cache_port = (i + 1) % _cache_fetch_queues.size();
		log.prefetch_issued_sectors++;
		break; // At most one 32-byte prefetch sector per global clock_fall.
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_issue_returns()
{
	if(!_ray_return_queue.empty())
	{
		uint ray_id = _ray_return_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		if(ray_state.phase != RayState::Phase::HIT_RETURN) return;

		if(_return_network.is_write_valid(0))
		{
			//fetch the next block
			MemoryReturn ret;
			ret.size = sizeof(rtm::Hit);
			ret.port = ray_state.dst.pop(8);
			ret.dst = ray_state.dst;
			ret.paddr = 0xdeadbeefull;
			std::memcpy(ret.data, &ray_state.hit, sizeof(rtm::Hit));
			_return_network.write(ret, 0);
			if(_hit_observer) _hit_observer(ray_state.ray, ray_state.hit);
			if(_identified_hit_observer) _identified_hit_observer(ray_state.original_ray_id, ray_state.ray, ray_state.hit);
			if(_ftb_wait) _ftb_wait->hit_return(ray_id, _ftb_wait_cycle);

			ray_state.phase = RayState::Phase::RAY_FETCH;
			_free_ray_ids.insert(ray_id);
			_ray_return_queue.pop();
			log.hits_returned++;
		}
	}
}

template class UnitRTCore<rtm::HE2CWBVH::Node, rtm::FTB>;
template class UnitRTCore<rtm::HECWBVH::Node, rtm::FTB>;

}}}
