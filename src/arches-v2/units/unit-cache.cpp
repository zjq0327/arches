#include "unit-cache.hpp"

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
			bank.return_queue.write(MemoryReturn(request, cached_data + _get_sector_offset(request.paddr)));
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
					mshr.subentries.pop();
				}
				if(mshr.subentries.empty())
				{
					mem_higher->read_return(slice.mem_higher_port);
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
				request.dst.push(request.port, 8);
				request.port = slice.mem_higher_port;
				slice.mem_higher_request_queue.push(request);
				log.uncached_requests++;
			}
			else if(request.type == MemoryRequest::Type::LOAD || hint)
			{
				uint8_t* data = _read_sector(sector_addr);
				log.tag_array_access++;
				if(data)
				{
					if(hint) log.pf_cache_redundant++;
					else
					{
						bank.return_queue.write(MemoryReturn(request, data + _get_sector_offset(request.paddr)));
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
					else slice.prefetch_miss_queue.push(request);
				}
				else
				{
					if(_miss_alloc) _allocate_for_fill(sector_addr);
					slice.miss_network.write(request, b);
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
			bank.return_queue.write(MemoryReturn(miss, cached_data + _get_sector_offset(miss.paddr)));
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
		slice.miss_network.read(0);
		if(request_sector)
		{
			MemoryRequest fill;
			fill.type = MemoryRequest::Type::LOAD;
			fill.paddr = sector_addr;
			fill.size = _sector_size;
			fill.port = slice.mem_higher_port;
			slice.mem_higher_request_queue.push(fill);
			log.misses++;
		}
		else log.half_misses++;

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
				bank.request_pipline.write(_request_network.read(port));
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
	_request_network.write(request, request.port);
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