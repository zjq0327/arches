#pragma once

// Offline post-build transform. Never used by BVH collapse/leaf grouping.
// Preserve the canonical local vertex table and triangle index stream exactly.
#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>
#include "bits.hpp"
#include "uvec2.hpp"
#include "vec2.hpp"
#include "vec4.hpp"
#include "ftb.hpp"

namespace rtm::ftb_prefix {

struct Table
{
	uint32_t prim_idx{}, triangles{}, vertices{}, bits{};
	std::array<std::array<uint32_t, 3>, FTB::MAX_TRIS> indices{};
	std::array<std::array<uint32_t, 3>, FTB::MAX_VRTS> vertex_bits{};
};

inline uint64_t fingerprint(const void* data, size_t bytes)
{
	uint64_t hash = 14695981039346656037ull;
	const auto* raw = static_cast<const uint8_t*>(data);
	for(size_t i = 0; i < bytes; ++i) { hash ^= raw[i]; hash *= 1099511628211ull; }
	return hash;
}

inline uint32_t effective_bits(const FTB& block)
{
	const uint32_t triangles = block.tri_cnt + 1, vertices = block.vrt_cnt + 1;
	const uint32_t nx = block.nx + 1, ny = block.ny + 1, nz = block.nz + 1;
	const uint32_t bits = 64 + 12 * triangles + (nx + ny + nz) * vertices +
		(32 - nx) + (32 - ny) + (32 - nz);
	if(block.is_int || triangles > FTB::MAX_TRIS || vertices > FTB::MAX_VRTS || bits > 1024)
		throw std::invalid_argument("FTB prefix: invalid block header/length");
	return bits;
}

inline uint32_t required_bytes(const FTB& block)
{
	return ((effective_bits(block) + 255) / 256) * 32;
}

inline Table decode_table(const FTB& block)
{
	Table table;
	table.bits = effective_bits(block);
	table.prim_idx = block.prim_idx;
	table.triangles = block.tri_cnt + 1;
	table.vertices = block.vrt_cnt + 1;
	uint32_t cursor = 0;
	for(uint32_t i = 0; i < table.triangles; ++i)
		for(uint32_t axis = 0; axis < 3; ++axis)
		{
			table.indices[i][axis] = static_cast<uint32_t>(block.data.read(cursor, 4));
			cursor += 4;
			if(table.indices[i][axis] >= table.vertices)
				throw std::invalid_argument("FTB prefix: triangle index exceeds local vertex table");
		}
	const uint32_t widths[3] = {uint32_t(block.nx) + 1, uint32_t(block.ny) + 1, uint32_t(block.nz) + 1};
	uint32_t background[3]{};
	for(uint32_t axis = 0; axis < 3; ++axis)
	{
		const uint32_t prefix = 32 - widths[axis];
		// A zero-size prefix needs no shift by32 in the uint32 representation.
		if(prefix) background[axis] = static_cast<uint32_t>(block.data.read(cursor, prefix)) << widths[axis];
		cursor += prefix;
	}
	for(uint32_t i = 0; i < table.vertices; ++i)
		for(uint32_t axis = 0; axis < 3; ++axis)
		{
			table.vertex_bits[i][axis] = background[axis] | static_cast<uint32_t>(block.data.read(cursor, widths[axis]));
			cursor += widths[axis];
		}
	if(cursor + 64 != table.bits) throw std::logic_error("FTB prefix: decoded cursor mismatch");
	return table;
}

inline void validate_round_trip(const FTB& canonical, const FTB& encoded)
{
	const Table before = decode_table(canonical), after = decode_table(encoded);
	if(before.prim_idx != after.prim_idx || before.triangles != after.triangles ||
		before.vertices != after.vertices || before.indices != after.indices || before.vertex_bits != after.vertex_bits)
		throw std::runtime_error("FTB prefix: local table/indices differ from canonical block");
	IntersectionTriangle old_triangles[FTB::MAX_TRIS], new_triangles[FTB::MAX_TRIS];
	const uint32_t old_count = ::rtm::decompress(canonical, old_triangles);
	const uint32_t new_count = ::rtm::decompress(encoded, new_triangles);
	if(old_count != new_count) throw std::runtime_error("FTB prefix: production triangle count mismatch");
	for(uint32_t i = 0; i < old_count; ++i)
	{
		if(old_triangles[i].id != new_triangles[i].id)
			throw std::runtime_error("FTB prefix: production primitive ID mismatch");
		for(uint32_t vertex = 0; vertex < 3; ++vertex)
			for(uint32_t axis = 0; axis < 3; ++axis)
				if(as_u32(old_triangles[i].tri.vrts[vertex][axis]) != as_u32(new_triangles[i].tri.vrts[vertex][axis]))
					throw std::runtime_error("FTB prefix: production FP32 vertex bits mismatch");
	}
}

inline FTB transform_block(const FTB& canonical)
{
	const Table table = decode_table(canonical);
	uint32_t prefixes[3]{}, widths[3]{};
	for(uint32_t axis = 0; axis < 3; ++axis)
	{
		uint32_t different = 0;
		for(uint32_t i = 1; i < table.vertices; ++i)
			different |= table.vertex_bits[0][axis] ^ table.vertex_bits[i][axis];
		// Header stores width-1 in5bits, so width must remain1..32.
		prefixes[axis] = std::min<uint32_t>(31, std::countl_zero(different));
		widths[axis] = 32 - prefixes[axis];
	}
	FTB encoded;
	std::memset(&encoded, 0, sizeof(encoded)); // Header padding and unused payload bits deterministic.
	encoded.prim_idx = canonical.prim_idx;
	encoded.tri_cnt = canonical.tri_cnt;
	encoded.vrt_cnt = canonical.vrt_cnt;
	encoded.nx = widths[0] - 1; encoded.ny = widths[1] - 1; encoded.nz = widths[2] - 1;
	uint32_t cursor = 0;
	for(uint32_t i = 0; i < table.triangles; ++i)
		for(uint32_t axis = 0; axis < 3; ++axis)
		{ encoded.data.write(cursor, 4, table.indices[i][axis]); cursor += 4; }
	for(uint32_t axis = 0; axis < 3; ++axis)
	{
		if(prefixes[axis]) encoded.data.write(cursor, prefixes[axis], table.vertex_bits[0][axis] >> widths[axis]);
		cursor += prefixes[axis];
	}
	for(uint32_t i = 0; i < table.vertices; ++i)
		for(uint32_t axis = 0; axis < 3; ++axis)
		{ encoded.data.write(cursor, widths[axis], table.vertex_bits[i][axis]); cursor += widths[axis]; }
	if(cursor + 64 != effective_bits(encoded) || cursor + 64 > table.bits)
		throw std::logic_error("FTB prefix: encoded cursor/size mismatch");
	validate_round_trip(canonical, encoded);
	return encoded;
}

struct Statistics
{
	uint64_t blocks{}, changed_blocks{}, encoded_size_changed_blocks{}, nonzero_prefix_blocks{};
	uint64_t old_total_bits{}, new_total_bits{}, old_sum_byte_ceil{}, new_sum_byte_ceil{};
	std::array<uint64_t, 4> old_required_hist{}, new_required_hist{};
};

struct Result
{
	std::vector<FTB> ftbs;
	Statistics stats;
	uint64_t canonical_fingerprint{}, encoded_fingerprint{};
};

inline Result transform(const std::vector<FTB>& canonical)
{
	Result result;
	result.canonical_fingerprint = fingerprint(canonical.data(), canonical.size() * sizeof(FTB));
	result.ftbs.reserve(canonical.size());
	for(const auto& block : canonical)
	{
		result.ftbs.push_back(transform_block(block));
		const auto& encoded = result.ftbs.back();
		const uint32_t old_bits = effective_bits(block), new_bits = effective_bits(encoded);
		const uint32_t old_bytes = required_bytes(block), new_bytes = required_bytes(encoded);
		auto& stats = result.stats;
		++stats.blocks;
		stats.changed_blocks += std::memcmp(&block, &encoded, sizeof(FTB)) != 0;
		stats.encoded_size_changed_blocks += old_bytes != new_bytes;
		stats.nonzero_prefix_blocks += encoded.nx != 31 || encoded.ny != 31 || encoded.nz != 31;
		stats.old_total_bits += old_bits; stats.new_total_bits += new_bits;
		stats.old_sum_byte_ceil += (old_bits + 7) / 8; stats.new_sum_byte_ceil += (new_bits + 7) / 8;
		++stats.old_required_hist[old_bytes / 32 - 1]; ++stats.new_required_hist[new_bytes / 32 - 1];
	}
	if(fingerprint(canonical.data(), canonical.size() * sizeof(FTB)) != result.canonical_fingerprint)
		throw std::runtime_error("FTB prefix: canonical array changed");
	result.encoded_fingerprint = fingerprint(result.ftbs.data(), result.ftbs.size() * sizeof(FTB));
	return result;
}

} // namespace rtm::ftb_prefix
