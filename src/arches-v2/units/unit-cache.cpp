#include "unit-cache.hpp"
#include <stdexcept>

namespace Arches {namespace Units {

UnitCache::UnitCache(Configuration config) :
	UnitCacheBase(config.size, config.block_size, config.associativity, config.sector_size, config.policy),
	_request_network(config.num_ports, config.num_slices * config.num_banks, config.block_size, config.crossbar_width),
	_return_network(config.num_slices * config.num_banks, config.num_ports, config.crossbar_width),
	_mem_highers(config.mem_highers),
	_level(config.level), _block_prefetch(config.block_prefetch), _num_mshr(config.num_mshr), _num_subentries(config.num_subentries), _pf_mshr_limit(config.pf_mshr_limit), _miss_alloc(config.miss_alloc)
{
	_slices.reserve(config.num_slices);
	for(uint i = 0; i < config.num_slices; ++i)
	{
		_slices.push_back(config);
		config.mem_higher_port += config.mem_higher_port_stride;
	}
}

UnitCache::Slice::Slice(Configuration config) :
	miss_network(config.num_banks, 1, config.num_banks, 1)
{
	mem_higher_port = config.mem_higher_port;

	banks.reserve(config.num_banks);
	for(uint i = 0; i < config.num_banks; ++i)
		banks.push_back(config);
}

UnitCache::Bank::Bank(Configuration config) :
	request_pipline(config.latency), return_pipline(1), return_queue(config.return_queue_size) {}

UnitCache::~UnitCache()
{

}

void UnitCache::configure_memory_path_diagnostics(bool enabled, uint16_t diagnostic_unit,
	std::pair<paddr_t, paddr_t> global_ftb_range, uint partition, uint partitions, uint stride)
{
	if(enabled && (!diagnostic_unit || !partitions || !stride || partition >= partitions ||
		global_ftb_range.first >= global_ftb_range.second))
		throw std::runtime_error("Invalid host cache memory-path diagnostic configuration");
	_memory_path_enabled = enabled;
	_memory_path_unit = diagnostic_unit;
	_memory_path_ftb_range = global_ftb_range;
	_memory_path_partition = partition;
	_memory_path_partitions = partitions;
	_memory_path_stride = stride;
	_memory_path_sequence = _memory_path_errors = 0;
	_memory_path_requests.clear(); _memory_path_fills.clear();
	_memory_path_request_index.clear(); _memory_path_fill_index.clear();
}

uint64_t UnitCache::_memory_path_cycle() const
{
	return static_cast<uint64_t>(simulator->current_cycle) + 1;
}

paddr_t UnitCache::_memory_path_global_address(paddr_t address) const
{
	return (address / _memory_path_stride * _memory_path_partitions + _memory_path_partition) *
		_memory_path_stride + address % _memory_path_stride;
}

bool UnitCache::_memory_path_ftb(paddr_t address, uint size) const
{
	if(!_memory_path_enabled) return false;
	const paddr_t global = _memory_path_global_address(address);
	return global >= _memory_path_ftb_range.first && global < _memory_path_ftb_range.second &&
		size <= _memory_path_ftb_range.second - global;
}

void UnitCache::_memory_path_error()
{
	++_memory_path_errors;
}

UnitCache::MemoryPathRequest* UnitCache::_memory_path_request(uint64_t token)
{
	if(!_memory_path_enabled || !token) return nullptr;
	const auto found = _memory_path_request_index.find(token);
	if(found == _memory_path_request_index.end()) return nullptr;
	return &_memory_path_requests[found->second];
}

void UnitCache::_memory_path_accept(const MemoryRequest& request)
{
	if(!_memory_path_ftb(request.paddr, request.size) || request.type != MemoryRequest::Type::LOAD) return;
	if(!request.host_trace_token) { ++_memory_path_errors; return; }
	if(_memory_path_request_index.count(request.host_trace_token)) { _memory_path_error(); return; }
	MemoryPathRequest observed;
	observed.token = request.host_trace_token;
	observed.address = _memory_path_global_address(request.paddr);
	observed.local_address = request.paddr;
	observed.entry = request.host_trace_entry_cycle;
	observed.accepted = _memory_path_cycle();
	observed.size = request.size; observed.port = request.port;
	observed.prefetch_origin = request.flags.prefetch_origin;
	_memory_path_request_index.emplace(observed.token, _memory_path_requests.size());
	_memory_path_requests.push_back(observed);
}

void UnitCache::_memory_path_lookup_ready(const MemoryRequest& request)
{
	if(auto* observed = _memory_path_request(request.host_trace_token))
		if(!observed->lookup_ready) observed->lookup_ready = _memory_path_cycle();
}

void UnitCache::_memory_path_lookup(const MemoryRequest& request, bool hit)
{
	if(auto* observed = _memory_path_request(request.host_trace_token))
		if(!observed->lookup)
		{
			observed->lookup = _memory_path_cycle();
			observed->initial_lookup_hit = hit;
		}
}

void UnitCache::_memory_path_miss_enqueue(const MemoryRequest& request)
{
	if(auto* observed = _memory_path_request(request.host_trace_token))
		if(!observed->miss_enqueued) observed->miss_enqueued = _memory_path_cycle();
}

void UnitCache::_memory_path_hit(const MemoryRequest& request, const char* path)
{
	if(auto* observed = _memory_path_request(request.host_trace_token))
	{
		observed->resolve = observed->data_ready = _memory_path_cycle();
		observed->path = path;
	}
}

void UnitCache::_memory_path_attach(const MemoryRequest& request, const MSHR& mshr, const char* path)
{
	if(auto* observed = _memory_path_request(request.host_trace_token))
	{
		observed->resolve = _memory_path_cycle();
		observed->path = path;
		observed->fill_token = mshr.host_fill_token;
		observed->source_prefetch = mshr.prefetch;
		const auto found = _memory_path_fill_index.find(mshr.host_fill_token);
		if(found == _memory_path_fill_index.end()) { _memory_path_error(); return; }
		observed->fill_ready = _memory_path_fills[found->second].fill_ready;
		if(observed->fill_ready) observed->data_ready = std::max(observed->resolve, observed->fill_ready);
	}
}

void UnitCache::_memory_path_new_fill(MSHR& mshr, MemoryRequest& fill,
	const MemoryRequest* trigger, const char* source)
{
	if(!_memory_path_ftb(fill.paddr, fill.size)) return;
	uint64_t& sequence = _memory_path_sequence;
	if(sequence >= ((1ull << 48) - 1))
		throw std::runtime_error("Host cache diagnostic token sequence exhausted");
	MemoryPathFill observed;
	observed.token = (static_cast<uint64_t>(_memory_path_unit) << 48) | ++sequence;
	observed.address = _memory_path_global_address(fill.paddr);
	observed.local_address = fill.paddr;
	observed.trigger_token = trigger ? trigger->host_trace_token : 0;
	observed.allocated = _memory_path_cycle();
	observed.prefetch_origin = fill.flags.prefetch_origin;
	observed.source = source;
	mshr.host_fill_token = fill.host_trace_token = observed.token;
	_memory_path_fill_index.emplace(observed.token, _memory_path_fills.size());
	_memory_path_fills.push_back(observed);
}

void UnitCache::_memory_path_fill_ready(MSHR& mshr, const MemoryReturn& ret)
{
	if(!_memory_path_enabled || !mshr.host_fill_token) return;
	const auto found = _memory_path_fill_index.find(mshr.host_fill_token);
	if(found == _memory_path_fill_index.end()) { _memory_path_error(); return; }
	if(ret.host_trace_token != mshr.host_fill_token) _memory_path_error();
	auto& fill = _memory_path_fills[found->second];
	if(fill.fill_ready) { _memory_path_error(); return; }
	fill.fill_ready = _memory_path_cycle();
	// A fill may fan out over several cycles. Update readiness now, before the
	// original one-subentry response arbitration, without touching that queue.
	auto remaining = mshr.subentries;
	while(!remaining.empty())
	{
		if(auto* observed = _memory_path_request(remaining.front().host_trace_token))
		{
			observed->fill_ready = fill.fill_ready;
			observed->data_ready = std::max(observed->resolve, fill.fill_ready);
		}
		remaining.pop();
	}
}

void UnitCache::_memory_path_response(const MemoryRequest& request)
{
	if(auto* observed = _memory_path_request(request.host_trace_token))
	{
		if(observed->response_enqueue) _memory_path_error();
		observed->response_enqueue = _memory_path_cycle();
	}
}

void UnitCache::_memory_path_emit(const MemoryReturn& ret)
{
	if(auto* observed = _memory_path_request(ret.host_trace_token))
	{
		if(observed->return_emit) _memory_path_error();
		observed->return_emit = _memory_path_cycle();
	}
}

void UnitCache::memory_path_headers(std::ostream& requests, std::ostream& fills)
{
	requests << "unit,simulator_unit,level,partition,token,address,local_address,size,port,prefetch_origin,entry,accepted,lookup_ready,lookup,initial_lookup_hit,miss_enqueued,resolve,path,fill_token,source_prefetch,fill_ready,data_ready,response_enqueue,return_emit\n";
	fills << "unit,simulator_unit,level,partition,token,address,local_address,trigger_token,prefetch_origin,source,allocated,issue,fill_ready,retired\n";
}

UnitCache::MemoryPathTotals UnitCache::write_memory_path_diagnostics(std::ostream& requests, std::ostream& fills) const
{
	MemoryPathTotals totals;
	totals.errors = _memory_path_errors;
	for(const auto& r : _memory_path_requests)
	{
		if(!_memory_path_ftb(r.local_address, r.size)) continue;
		++totals.requests;
		const bool complete = r.entry && r.accepted && r.lookup_ready && r.lookup && r.resolve &&
			r.data_ready && r.response_enqueue && r.return_emit && !r.path.empty();
		if(!complete) ++totals.incomplete_requests;
		if(complete && !(r.entry <= r.accepted && r.accepted <= r.lookup_ready && r.lookup_ready <= r.lookup &&
			r.lookup <= r.resolve && r.resolve <= r.data_ready && r.data_ready <= r.response_enqueue &&
			r.response_enqueue <= r.return_emit)) ++totals.errors;
		requests << _memory_path_unit << ',' << unit_id << ',' << _level << ',' << _memory_path_partition << ','
			<< r.token << ',' << r.address << ',' << r.local_address << ',' << r.size << ',' << r.port << ',' << r.prefetch_origin << ','
			<< r.entry << ',' << r.accepted << ',' << r.lookup_ready << ',' << r.lookup << ',' << r.initial_lookup_hit << ','
			<< r.miss_enqueued << ',' << r.resolve << ',' << r.path << ',' << r.fill_token << ',' << r.source_prefetch << ','
			<< r.fill_ready << ',' << r.data_ready << ',' << r.response_enqueue << ',' << r.return_emit << '\n';
	}
	for(const auto& f : _memory_path_fills)
	{
		if(!_memory_path_ftb(f.local_address, _sector_size)) continue;
		++totals.fills;
		const bool complete = f.allocated && f.issue && f.fill_ready && f.retired;
		if(!complete) ++totals.incomplete_fills;
		if(complete && !(f.allocated <= f.issue && f.issue <= f.fill_ready && f.fill_ready <= f.retired)) ++totals.errors;
		fills << _memory_path_unit << ',' << unit_id << ',' << _level << ',' << _memory_path_partition << ','
			<< f.token << ',' << f.address << ',' << f.local_address << ',' << f.trigger_token << ',' << f.prefetch_origin << ','
			<< f.source << ',' << f.allocated << ',' << f.issue << ',' << f.fill_ready << ',' << f.retired << '\n';
	}
	return totals;
}

void UnitCache::_allocate_for_fill(paddr_t sector_addr)
{
	// A miss-allocated tag may have been evicted before its data returns.
	// Keep an existing tag, including valid sibling sectors, when it is present.
	log.tag_array_access++;
	uint set = _get_set_index(sector_addr);
	uint64_t tag = _get_tag(sector_addr);
	for(uint i = set * _associativity; i < (set + 1) * _associativity; ++i)
		if(_tag_array[i].tag == tag) return;

	Victim victim = _allocate_block(sector_addr);
	for(uint i = 0; i < _block_size / _sector_size; ++i)
		if((victim.valid >> i) & 1)
			if(_prefetched_sectors.erase(victim.addr + i * _sector_size))
				log.pf_unused_evicted++;
}

bool UnitCache::_queue_prefetch(Slice& slice, const MemoryRequest& request)
{
	const bool hint = request.type == MemoryRequest::Type::PREFECTH;
	paddr_t sector_addr = _get_sector_addr(request.paddr);
	// A fill can overtake an earlier lookup while this request waits for the
	// shared miss-processing opportunity. Charge the recheck and normal hit path.
	uint8_t* cached_data = _read_sector(sector_addr);
	log.tag_array_access++;
	if(cached_data)
	{
		if(hint) log.pf_cache_redundant++;
		else
		{
			Bank& bank = slice.banks[_get_bank(sector_addr)];
			if(!bank.return_queue.is_write_valid()) { log.pf_stalls++; return false; }
			_memory_path_hit(request, "RECHECK_HIT");
			bank.return_queue.write(MemoryReturn(request, cached_data + _get_sector_offset(request.paddr)));
			_memory_path_response(request);
			log.data_array_reads++;
			log.pf_lookup_hits++;
		}
		return true;
	}
	auto found = slice.mshrs.find(sector_addr);
	if(found != slice.mshrs.end())
	{
		// Hints need no response. Downstream LOADs must return to their cache.
		if(!hint)
		{
			if(found->second.subentries.size() >= _num_subentries) { log.pf_stalls++; return false; }
			found->second.subentries.push(request);
			_memory_path_attach(request, found->second, "MERGE");
		}
		log.pf_inflight_redundant++;
		return true;
	}

	// Preserve a demand slot whenever more than one MSHR exists. A committed
	// downstream LOAD cannot be dropped; in a one-MSHR cache it must progress.
	uint reserve = _num_mshr > 1 ? 1 : 0;
	uint limit = std::min(hint ? _pf_mshr_limit : std::max(1u, _pf_mshr_limit), _num_mshr - reserve);
	if(hint && _num_mshr <= 1) limit = 0;
	if(slice.prefetch_mshrs >= limit || slice.mshrs.size() >= _num_mshr - reserve ||
		slice.prefetch_request_queue.size() >= std::max(1u, _pf_mshr_limit))
	{
		if(!hint) { log.pf_stalls++; return false; }
		log.pf_drop++;
		return true;
	}

	MSHR& mshr = slice.mshrs[sector_addr];
	mshr.prefetch = true;
	if(!hint) mshr.subentries.push(request);
	++slice.prefetch_mshrs;
	MemoryRequest fill;
	fill.type = MemoryRequest::Type::LOAD;
	fill.flags.prefetch_origin = 1;
	fill.paddr = sector_addr;
	fill.size = _sector_size;
	fill.port = slice.mem_higher_port;
	_memory_path_new_fill(mshr, fill, &request, hint ? "PREFETCH_HINT" : "PREFETCH_LOAD");
	if(!hint) _memory_path_attach(request, mshr, "NEW_MISS");
	slice.prefetch_request_queue.push(fill);
	return true;
}

void UnitCache::_recive_return()
{
	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		for(UnitMemoryBase* mem_higher : _mem_highers)
		{
			if(!mem_higher->return_port_read_valid(slice.mem_higher_port)) continue;
			MemoryReturn ret = mem_higher->peek_return(slice.mem_higher_port);
			bool cached = !(ret.flags.omit_cache & (0x1 << _level));
			if(cached)
			{
				paddr_t sector_addr = _get_sector_addr(ret.paddr);
				auto found = slice.mshrs.find(sector_addr);
				_assert(found != slice.mshrs.end());
				MSHR& mshr = found->second;
				Bank& bank = slice.banks[_get_bank(ret.paddr)];
				if(!mshr.filled)
				{
					_allocate_for_fill(sector_addr);
					uint8_t* filled_data = _write_sector(sector_addr, ret.data, false);
					_assert(filled_data);
					log.data_array_writes++;
					mshr.filled = true;
					_memory_path_fill_ready(mshr, ret);
					if(mshr.prefetch)
					{
						log.pf_fill++;
						if(!mshr.demand_seen) _prefetched_sectors.insert(sector_addr);
					}
				}

				if(bank.return_pipline.is_write_valid() && !mshr.subentries.empty())
				{
					MemoryRequest& sube = mshr.subentries.front();
					bank.return_pipline.write(MemoryReturn(sube, ret.data + _get_sector_offset(sube.paddr)));
					_memory_path_response(sube);
					mshr.subentries.pop();
				}
				if(mshr.subentries.empty())
				{
					mem_higher->read_return(slice.mem_higher_port);
					if(_memory_path_enabled && mshr.host_fill_token)
					{
						const auto observed = _memory_path_fill_index.find(mshr.host_fill_token);
						if(observed != _memory_path_fill_index.end()) _memory_path_fills[observed->second].retired = _memory_path_cycle();
						else _memory_path_error();
					}
					if(mshr.prefetch) --slice.prefetch_mshrs;
					slice.mshrs.erase(found);
				}
			}
			else
			{
				Bank& bank = slice.banks[_get_bank(ret.paddr)];
				if(bank.return_pipline.is_write_valid())
				{
					ret.port = ret.dst.pop(8);
					bank.return_pipline.write(ret);
					if(auto* observed = _memory_path_request(ret.host_trace_token))
						observed->data_ready = observed->response_enqueue = _memory_path_cycle();
					mem_higher->read_return(slice.mem_higher_port);
				}
			}
		}
	}
}

void UnitCache::_recive_request()
{
	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		for(uint b = 0; b < slice.banks.size(); ++b)
		{
			Bank& bank = slice.banks[b];
			if(!bank.request_pipline.is_read_valid()) continue;
			MemoryRequest request = bank.request_pipline.peek();
			_memory_path_lookup_ready(request);
			bool hint = request.type == MemoryRequest::Type::PREFECTH;
			bool pf = hint || request.flags.prefetch_origin;
			bool cached = !(request.flags.omit_cache & (0x1 << _level));
			// The no-response hint is consumed only at its L1 target.
			if(hint && (_level != 1 || !cached))
			{
				log.pf_received++;
				log.pf_drop++;
				bank.request_pipline.read();
				continue;
			}
			if(!hint && (!bank.return_queue.is_write_valid() ||
				(!pf && !slice.miss_network.is_write_valid(b))))
			{
				if(pf) log.pf_stalls++;
				else log.mshr_stalls++;
				continue;
			}

			paddr_t sector_addr = _get_sector_addr(request.paddr);
			if(!cached)
			{
				_memory_path_lookup(request, false);
				if(auto* observed = _memory_path_request(request.host_trace_token))
				{
					observed->resolve = _memory_path_cycle();
					observed->path = "UNCACHED";
				}
				request.dst.push(request.port, 8);
				request.port = slice.mem_higher_port;
				slice.mem_higher_request_queue.push(request);
				log.uncached_requests++;
			}
			else if(request.type == MemoryRequest::Type::LOAD || hint)
			{
				uint8_t* data = _read_sector(sector_addr);
				log.tag_array_access++;
				_memory_path_lookup(request, data != nullptr);
				if(data)
				{
					if(hint) log.pf_cache_redundant++;
					else
					{
						_memory_path_hit(request, "HIT");
						bank.return_queue.write(MemoryReturn(request, data + _get_sector_offset(request.paddr)));
						_memory_path_response(request);
						log.data_array_reads++;
						if(pf) log.pf_lookup_hits++;
						else
						{
							log.hits++;
							if(_prefetched_sectors.erase(sector_addr)) log.pf_timely_useful++;
						}
					}
				}
				else if(pf)
				{
					if(slice.prefetch_miss_queue.size() >= (hint ? _pf_mshr_limit : std::max(1u, _pf_mshr_limit)))
					{
						if(!hint) { log.pf_stalls++; continue; } // A cache-to-cache LOAD needs its response.
						log.pf_drop++;
					}
					else
					{
						slice.prefetch_miss_queue.push(request);
						_memory_path_miss_enqueue(request);
					}
				}
				else
				{
					if(_miss_alloc) _allocate_for_fill(sector_addr);
					slice.miss_network.write(request, b);
					_memory_path_miss_enqueue(request);
				}
			}
			else _assert(false);
			if(pf) log.pf_received++;
			bank.request_pipline.read();
		}

		// Keep speculative misses out of the demand cascade. Share its one
		// MSHR allocation opportunity per slice with demand traffic first.
		slice.miss_network.clock();
		if(!slice.miss_network.is_read_valid(0))
		{
			if(!slice.prefetch_miss_queue.empty() &&
				_queue_prefetch(slice, slice.prefetch_miss_queue.front()))
				slice.prefetch_miss_queue.pop();
			continue;
		}
		const MemoryRequest& miss = slice.miss_network.peek(0);
		paddr_t sector_addr = _get_sector_addr(miss.paddr);
		uint8_t* cached_data = _read_sector(sector_addr);
		log.tag_array_access++;
		if(cached_data)
		{
			Bank& bank = slice.banks[_get_bank(sector_addr)];
			if(!bank.return_queue.is_write_valid()) continue;
			_memory_path_hit(miss, "RECHECK_HIT");
			bank.return_queue.write(MemoryReturn(miss, cached_data + _get_sector_offset(miss.paddr)));
			_memory_path_response(miss);
			log.data_array_reads++;
			log.hits++;
			if(_prefetched_sectors.erase(sector_addr)) log.pf_timely_useful++;
			slice.miss_network.read(0);
			continue;
		}
		bool request_sector = slice.mshrs.find(sector_addr) == slice.mshrs.end();
		if(request_sector && slice.mshrs.size() >= _num_mshr)
		{
			log.mshr_stalls++;
			continue;
		}
		MSHR& mshr = slice.mshrs[sector_addr];
		if(mshr.subentries.size() >= _num_subentries)
		{
			log.mshr_stalls++;
			continue;
		}
		if(mshr.prefetch && !mshr.demand_seen && !mshr.filled) log.pf_late++;
		mshr.demand_seen = true;
		mshr.subentries.push(miss);
		// Keep observation identity before the original cascade pop invalidates its front.
		const MemoryRequest observed_miss = miss;
		slice.miss_network.read(0);
		if(request_sector)
		{
			MemoryRequest fill;
			fill.type = MemoryRequest::Type::LOAD;
			fill.paddr = sector_addr;
			fill.size = _sector_size;
			fill.port = slice.mem_higher_port;
			_memory_path_new_fill(mshr, fill, &observed_miss, "DEMAND");
			_memory_path_attach(observed_miss, mshr, "NEW_MISS");
			slice.mem_higher_request_queue.push(fill);
			log.misses++;
		}
		else
		{
			_memory_path_attach(observed_miss, mshr, "MERGE");
			log.half_misses++;
		}

		if(_block_prefetch)
		{
			paddr_t block_address = _get_block_addr(sector_addr);
			for(uint i = 0; i < _block_size; i += _sector_size)
			{
				sector_addr = block_address + i;
				if(slice.mshrs.find(sector_addr) == slice.mshrs.end())
				{
					slice.mshrs.insert({sector_addr, MSHR()});
					MemoryRequest mshr_fill_req;
					mshr_fill_req.type = MemoryRequest::Type::LOAD;
					mshr_fill_req.paddr = sector_addr;
					mshr_fill_req.size = _sector_size;
					mshr_fill_req.port = slice.mem_higher_port;
					_memory_path_new_fill(slice.mshrs.find(sector_addr)->second, mshr_fill_req, &observed_miss, "BLOCK_PREFETCH");
					slice.mem_higher_request_queue.push(mshr_fill_req);
				}
			}
		}
	}
}

void UnitCache::_send_request()
{
	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		std::queue<MemoryRequest>* queue = &slice.mem_higher_request_queue;
		if(queue->empty()) queue = &slice.prefetch_request_queue;
		if(queue->empty()) continue;
		const MemoryRequest& request = queue->front();
		UnitMemoryBase* mem_higher = _get_mem_higher(request.paddr);
		_assert(request.port == slice.mem_higher_port);
		if(!mem_higher->request_port_write_valid(request.port)) continue;
		mem_higher->write_request(request);
		if(_memory_path_enabled && request.host_trace_token)
		{
			const auto observed = _memory_path_fill_index.find(request.host_trace_token);
			if(observed != _memory_path_fill_index.end())
			{
				auto& fill = _memory_path_fills[observed->second];
				if(fill.issue) _memory_path_error();
				fill.issue = _memory_path_cycle();
			}
		}

		if(request.flags.prefetch_origin) log.pf_issued++;
		queue->pop();
	}
}

void UnitCache::clock_rise()
{
	_request_network.clock();

	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		for(uint b = 0; b < slice.banks.size(); ++b)
		{
			Bank& bank = slice.banks[b];
			uint port = s * slice.banks.size() + b;

			if(_request_network.is_read_valid(port) && bank.request_pipline.is_write_valid())
			{
				const MemoryRequest request = _request_network.read(port);
				_memory_path_accept(request);
				bank.request_pipline.write(request);
			}
			bank.request_pipline.clock();
		}
	}

	_recive_return();
	_recive_request();
}

void UnitCache::clock_fall()
{
	_send_request();

	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		for(uint b = 0; b < slice.banks.size(); ++b)
		{
			Bank& bank = slice.banks[b];
			uint port = s * slice.banks.size() + b;

			if(bank.return_pipline.is_write_valid() && bank.return_queue.is_read_valid())
				bank.return_pipline.write(bank.return_queue.read());

			bank.return_pipline.clock();
			if(bank.return_pipline.is_read_valid() && _return_network.is_write_valid(port))
			{
				log.bytes_read += bank.return_pipline.peek().size;
				_memory_path_emit(bank.return_pipline.peek());
				_return_network.write(bank.return_pipline.read(), port);
			}
		}
	}

	_return_network.clock();
}

bool UnitCache::request_port_write_valid(uint port_index)
{
	return _request_network.is_write_valid(port_index);
}

void UnitCache::write_request(const MemoryRequest& request)
{
	if(_memory_path_enabled && request.host_trace_token && _memory_path_ftb(request.paddr, request.size))
	{
		MemoryRequest observed = request;
		observed.host_trace_entry_cycle = _memory_path_cycle();
		_request_network.write(observed, observed.port);
	}
	else _request_network.write(request, request.port);
}

bool UnitCache::return_port_read_valid(uint port_index)
{
	return _return_network.is_read_valid(port_index);
}

const MemoryReturn& UnitCache::peek_return(uint port_index)
{
	return _return_network.peek(port_index);
}

const MemoryReturn UnitCache::read_return(uint port_index)
{
	return _return_network.read(port_index);
}

}}