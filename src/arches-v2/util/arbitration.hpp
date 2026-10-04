#pragma once 
#include "stdafx.hpp"
#include "bit-manipulation.hpp"

class Arbiter
{
public:
	virtual uint size() = 0;
	virtual uint num_pending() = 0;
	virtual void add(uint index) = 0;
	virtual void remove(uint index) = 0;
	virtual uint get_index() = 0;
};

// Rotate in the mask's actual width. The 64-bit helper loses wrapped bits
// when its result is narrowed to a 16-bit or 32-bit arbiter mask.
template<typename MASK_T>
inline MASK_T arbiter_rotr(MASK_T mask, uint priority)
{
	if constexpr(sizeof(MASK_T) > sizeof(uint64_t))
		return rotr(mask, priority); // uint128_t has its own full-width overload.
	else
	{
		constexpr uint width = sizeof(MASK_T) * 8;
		uint n = priority % width;
		if(n == 0) return mask;
		uint64_t value = static_cast<uint64_t>(mask);
		return static_cast<MASK_T>((value >> n) | (value << (width - n)));
	}
}

//Uses a nbit integer and BM2 extension to implement a computational and stoarge efficent arbiter for up to 64 clients
template<typename MASK_T = uint64_t>
class RoundRobinArbiter : public Arbiter
{
protected:
	MASK_T _pending{0};
	uint8_t _priority_index{0};
	uint8_t _size;

public:
	RoundRobinArbiter(uint size = sizeof(MASK_T) * 8) : _size(size) 
	{ 
		_assert(size <= sizeof(MASK_T) * 8); 
		_assert(sizeof(MASK_T) * 8 <= 128);
	}

	uint size() override { return _size; }

	uint num_pending() override
	{
		return popcnt(_pending);
	}

	void add(uint index) override
	{
		_pending |= MASK_T(1) << index;
	}

	void remove(uint index) override
	{
		_pending &= ~(MASK_T(1) << index);

		//Advance the priority index on remove so the grant index is now lowest priority
		if(index == _priority_index)
			if(++_priority_index >= _size)
				_priority_index = 0;
	}

	uint get_index() override
	{
		if(num_pending() == 0)
			return ~0u;

		MASK_T rot_mask = arbiter_rotr(_pending, _priority_index);
		uint offset = ctz(rot_mask);
		uint grant_index = (_priority_index + offset) % (sizeof(MASK_T) * 8);
		_priority_index = grant_index; //make the grant bit the highest priority bit so that it will continue to be granted until removed
		return grant_index; 
	}
};

const static uint8_t _default_weight_table[128] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

//Uses a nbit integer and BM2 extension to implement a computational and stoarge efficent arbiter for up to 64 clients
template<typename MASK_T = uint64_t>
class WeightedRobinArbiter : public Arbiter
{
protected:
	MASK_T _pending{0};
	uint8_t _priority_index{0};
	uint8_t _grant_counter{0};
	uint8_t _size;

public:
	const uint8_t* _weight_table;

public:
	WeightedRobinArbiter(uint size = sizeof(MASK_T) * 8, const uint8_t* weight_table = _default_weight_table) : _size(size), _weight_table(weight_table)
	{ 
		_assert(size <= sizeof(MASK_T) * 8); 
	}

	uint size() override { return _size; }

	uint num_pending() override
	{
		return popcnt(_pending);
	}

	void add(uint index) override
	{
		_pending |= MASK_T(1) << index;
	}

	void remove(uint index) override
	{
		_pending &= ~(MASK_T(1) << index);

		//Advance the priority index on remove so the grant index is now lowest priority only after granting N requests
		if(index == _priority_index)
			if(++_grant_counter >= _weight_table[_priority_index])
				if(++_priority_index >= _size)
				{
					_grant_counter = 0;
					_priority_index = 0;
				}
	}

	uint get_index() override
	{
		if(num_pending() == 0)
			return ~0u;

		MASK_T rot_mask = arbiter_rotr(_pending, _priority_index);
		uint offset = ctz(rot_mask);
		uint grant_index = (_priority_index + offset) % (sizeof(MASK_T) * 8);
		if(grant_index != _priority_index) _grant_counter = 0; //if this a new grant chain reset the counter
		_priority_index = grant_index; //make the grant bit the highest priority bit so that it will continue to be granted until removed
		return grant_index;
	}
};

