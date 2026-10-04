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
	_node_prefetch_depth(config.node_prefetch_depth), _prefetch_queue_size(config.prefetch_queue_size),
	_hit_observer(config.hit_observer)
{
	_assert(_node_prefetch_depth <= 2);
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
	{
		_ray_states[i].phase = RayState::Phase::RAY_FETCH;
		_free_ray_ids.insert(i);
	}

	_cache_fetch_queues.resize(config.num_cache_ports);
}
template<typename NT, typename PT>
void UnitRTCore<NT, PT>::clock_rise()
{
	_prefetch_stack_read_used = false;
	// Phase occupancy is sampled once per global cycle, not once per scheduler subcycle.
	for(const RayState& ray_state : _ray_states)
	{
		if(ray_state.phase == RayState::Phase::NODE_FETCH) log.node_fetch_ray_cycles++;
		if(ray_state.phase == RayState::Phase::TRI_FETCH) log.tri_fetch_ray_cycles++;
	}

	_request_network.clock();
	_read_requests();
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
	_scan_node_prefetch_candidate();
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
bool UnitRTCore<NT, PT>::_try_queue_tri(uint ray_id, uint tri_id)
{
	paddr_t start = _tri_base_addr + tri_id * sizeof(PT);
	if(typeid(NT) == typeid(rtm::HECWBVH::Node))
		start = _node_base_addr + tri_id * sizeof(NT);
	paddr_t end = start + sizeof(PT);

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;
	ray_state.buffer.type = 1;
	ray_state.buffer.id = tri_id;

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

	auto& last_req = _cache_fetch_queues[ray_id % _cache_fetch_queues.size()].back();
	//last_req.flags.trigger_prefetch = 1;
	//for(uint32_t i = 0; i < 8; ++i)
	//{
	//	last_req.prefetch_offsets[0] = 0;
	//	if(i < ray_state.stack_size)
	//	{
	//		paddr_t addr = ray_state.stack[ray_state.stack_size - i - 1];
	//		if(addr / 4096 == start / 4096) last_req.prefetch_offsets[0] = (start % 4096) / 32;
	//	}
	//}

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
		log.prefetch_dropped_sectors += missing_sectors.size();
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
void UnitRTCore<NT, PT>::_invalidate_node_prefetch(uint ray_id)
{
	if(_node_prefetch_depth == 0) return;
	RayState& ray_state = _ray_states[ray_id];
	++ray_state.prefetch_epoch;
	ray_state.prefetch_scan_next = 0;
	ray_state.prefetch_scan_count = 0;
	ray_state.prefetch_first_valid = false;

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
void UnitRTCore<NT, PT>::_begin_node_prefetch_scan(uint ray_id)
{
	if(_node_prefetch_depth == 0) return;
	RayState& ray_state = _ray_states[ray_id];
	ray_state.prefetch_scan_stack_size = ray_state.stack_size;
	ray_state.prefetch_scan_next = 0;
	ray_state.prefetch_scan_count = static_cast<uint8_t>(
		std::min(_node_prefetch_depth, static_cast<uint>(ray_state.stack_size)));
	ray_state.prefetch_first_valid = false;
	if(ray_state.prefetch_scan_count && !_prefetch_stack_read_used)
	{
		// Capture the top entry at the pop, before any later node intersection can push.
		ray_state.prefetch_first_snapshot = ray_state.stack[ray_state.stack_size - 1];
		ray_state.prefetch_first_valid = true;
		_prefetch_stack_read_used = true;
	}
	if(ray_state.prefetch_scan_count && !ray_state.prefetch_scan_queued)
	{
		_prefetch_scan_queue.push(ray_id);
		ray_state.prefetch_scan_queued = true;
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_scan_node_prefetch_candidate()
{
	if(_node_prefetch_depth == 0) return;
	// A ready pop-time snapshot may be behind a scan waiting for the next cycle's port.
	const uint pending = static_cast<uint>(_prefetch_scan_queue.size());
	for(uint scanned = 0; scanned < pending; ++scanned)
	{
		const uint ray_id = _prefetch_scan_queue.front();
		_prefetch_scan_queue.pop();
		RayState& ray_state = _ray_states[ray_id];
		ray_state.prefetch_scan_queued = false;
		if(ray_state.prefetch_scan_next >= ray_state.prefetch_scan_count) continue;
		if(ray_state.stack_size != ray_state.prefetch_scan_stack_size)
		{
			_invalidate_node_prefetch(ray_id);
			continue;
		}

		StackEntry entry;
		if(ray_state.prefetch_scan_next == 0 && ray_state.prefetch_first_valid)
		{
			entry = ray_state.prefetch_first_snapshot;
			ray_state.prefetch_first_valid = false;
		}
		else
		{
			if(_prefetch_stack_read_used)
			{
				_prefetch_scan_queue.push(ray_id);
				ray_state.prefetch_scan_queued = true;
				continue;
			}
			const uint slot = ray_state.prefetch_scan_stack_size - 1 - ray_state.prefetch_scan_next;
			entry = ray_state.stack[slot];
			_prefetch_stack_read_used = true;
		}
		++ray_state.prefetch_scan_next;
		if(ray_state.prefetch_scan_next < ray_state.prefetch_scan_count)
		{
			_prefetch_scan_queue.push(ray_id);
			ray_state.prefetch_scan_queued = true;
		}

		if(entry.data.is_int && entry.t < ray_state.hit.t)
		{
			log.prefetch_candidates++;
			const paddr_t addr = _node_base_addr + entry.data.child_idx * sizeof(NT);
			_try_queue_prefetch(ray_id, addr, sizeof(NT), 0);
		}
		return; // At most one candidate is processed per global cycle.
	}
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
		_invalidate_node_prefetch(ray_id);
		std::memcpy(&ray_state.ray, request.data, sizeof(rtm::Ray));
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
		_ray_scheduling_queue.push(ray_id);

		log.rays++;

		//_ray_return_queue.push(ray_id);
	}

	//if(_free_ray_ids.empty()) _drain_phase = true;

	_stall_cycles++;
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
				uint offset = (ret.paddr - buffer.address);
				std::memcpy((uint8_t*)&buffer.data + offset, ret.data, ret.size);
				buffer.bytes_filled += ret.size;
				if(buffer.bytes_filled == sizeof(PT))
				{

					ray_state.phase = RayState::Phase::TRI_ISECT;
					_tri_isect_queue.push(ray_id);
				}
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
		uint ray_id = _ray_scheduling_queue.front();
		_ray_scheduling_queue.pop();

		RayState& ray_state = _ray_states[ray_id];
		_invalidate_node_prefetch(ray_id);

		StackEntry entry;
		if(ray_state.update_restart_trail)
		{
			uint parent_level = ray_state.restart_trail.find_parent_level(ray_state.level);
			if(parent_level == ~0u)
			{
				//Ray complete
				//stack empty or anyhit found return the hit
				ray_state.phase = RayState::Phase::HIT_RETURN;
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

		if(!_pop_culling || entry.t < ray_state.hit.t)
		{
			if(entry.data.is_int)
			{
				_try_queue_node(ray_id, entry.data.child_idx);
				ray_state.phase = RayState::Phase::NODE_FETCH;
				_begin_node_prefetch_scan(ray_id);

				log.issue_counters[(uint)IssueType::NODE_FETCH]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d NODE_FETCH: %d\n", ray_id, entry.data.child_idx);
			}
			else
			{
				_try_queue_tri(ray_id, entry.data.prim_idx);
				if(entry.data.prim_cnt > 1)
				{
					entry.data.prim_cnt--;
					entry.data.prim_idx++;
					ray_state.stack[ray_state.stack_size++] = entry;
					ray_state.update_restart_trail = false;
				}

				ray_state.phase = RayState::Phase::TRI_FETCH;
				_begin_node_prefetch_scan(ray_id);

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
		uint node_count = rtm::decompress(ray_state.buffer.node, nodes);

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
					uint drain_count = ray_state.stack_size - RayState::STACK_SIZE;
					for(uint i = 0; i < RayState::STACK_SIZE; ++i)
						ray_state.stack[i] = ray_state.stack[i + drain_count];
					ray_state.stack_size = RayState::STACK_SIZE;
				}
				_invalidate_node_prefetch(ray_id);
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
			_ray_states[ray_id].phase = RayState::Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);
			log.nodes++;
		}
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_simualte_tri_pipline()
{
	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
	{
		_stall_cycles = 0;
		uint ray_id = _tri_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

		rtm::IntersectionTriangle tris[rtm::FTB::MAX_TRIS];
		uint tri_count = rtm::decompress(buffer.prim, tris);

		_tri_issue_count += 1;
		if(_tri_issue_count >= tri_count)
		{
			rtm::Ray& ray = ray_state.ray;
			rtm::vec3& inv_d = ray_state.inv_d;
			rtm::Hit& hit = ray_state.hit;
			const float previous_hit_t = hit.t;

			for(uint i = 0; i < tri_count; ++i)
				if(rtm::intersect(tris[i].tri, ray, hit))
					hit.id = tris[i].id;
			if(hit.t < previous_hit_t) _invalidate_node_prefetch(ray_id);

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
			_ray_states[ray_id].phase = RayState::Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);
		}

	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_issue_requests()
{
	std::vector<uint8_t> demand_selected;
	if(_node_prefetch_depth) demand_selected.resize(_cache_fetch_queues.size(), 0);
	for(uint i = 0; i < _cache_fetch_queues.size(); ++i)
	{
		uint port = _cache_port + i * _cache_port_stride;
		if(!_cache_fetch_queues[i].empty() && _cache->request_port_write_valid(port))
		{
			_cache_fetch_queues[i].front().port = port;
			_cache->write_request(_cache_fetch_queues[i].front());
			_cache_fetch_queues[i].pop();
			if(_node_prefetch_depth) demand_selected[i] = 1;
		}
	}

	// Demand has selected all ports before any prefetch is considered.
	if(_node_prefetch_depth == 0 || _prefetch_queue.empty()) return;
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
			_invalidate_node_prefetch(ray_id);

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