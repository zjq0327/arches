#pragma once

// Host-side format transform and the corresponding simulated node decoder.
// The canonical HE2/FTB arrays and their cache serialization are never changed.
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "bits.hpp"
#include "uvec2.hpp"
#include "vec2.hpp"
#include "vec4.hpp"
#include "nvcwbvh.hpp"

namespace rtm::compact_ftb {

// A compact node still occupies 64B. Its base uses low25 bits as a 64B slot.
// The size flag consumes a previously unused high bit of the transformed base.
inline constexpr uint32_t NODE_SMALL_FLAG = uint32_t{1} << 31;
// A leaf stays within BVHPtr's existing 26-bit prim_idx; prim_cnt stays one.
inline constexpr uint32_t LEAF_SMALL_FLAG = uint32_t{1} << 25;
inline constexpr uint32_t SLOT_MASK = LEAF_SMALL_FLAG - 1;
inline constexpr uint64_t MAX_PAYLOAD_BYTES = uint64_t{LEAF_SMALL_FLAG} * 64;

static_assert(sizeof(HE2CWBVH::Node) == 64, "Compact nodes must remain 64B");
static_assert(sizeof(FTB) == 128, "Canonical FTB format must remain 128B");
static_assert(sizeof(BVHPtr) == 4, "Compact leaves require the existing 32-bit pointer");

inline constexpr uint32_t leaf_slot(uint32_t encoded) noexcept
{
	return encoded & SLOT_MASK;
}

inline constexpr uint32_t leaf_bytes(uint32_t encoded) noexcept
{
	return (encoded & LEAF_SMALL_FLAG) ? 64u : 128u;
}

inline uint32_t encode_leaf(uint32_t slot, uint32_t bytes)
{
	if(bytes != 64 && bytes != 128)
		throw std::invalid_argument("compact FTB: physical size must be 64 or 128B");
	if(slot > SLOT_MASK || uint64_t{slot} * 64 + bytes > MAX_PAYLOAD_BYTES)
		throw std::out_of_range("compact FTB: leaf exceeds 25-bit slot / 2GiB range");
	if(bytes == 128 && (slot & 1))
		throw std::invalid_argument("compact FTB: 128B leaf must be 128B aligned");
	return slot | (bytes == 64 ? LEAF_SMALL_FLAG : 0u);
}

// Header bits determine the encoded size for packing, validation and statistics.
// Runtime fetches the full 64B/128B block; invalid headers must not fall back.
inline uint32_t required_bytes(const FTB& header)
{
	const uint32_t tris = header.tri_cnt + 1;
	const uint32_t vertices = header.vrt_cnt + 1;
	const uint32_t nx = header.nx + 1, ny = header.ny + 1, nz = header.nz + 1;
	const uint32_t bits = 64 + 12 * tris + (nx + ny + nz) * vertices +
		(32 - nx) + (32 - ny) + (32 - nz);
	if(header.is_int || tris > FTB::MAX_TRIS || vertices > FTB::MAX_VRTS || bits > 1024)
		throw std::invalid_argument("compact FTB: invalid canonical FTB header");
	return ((bits + 255) / 256) * 32;
}

inline bool valid_child(const HE2CWBVH::Node& node, uint32_t child) noexcept
{
	for(uint32_t axis = 0; axis < 3; ++axis)
		if(node.qaabb[child].min[axis] > node.qaabb[child].max[axis]) return false;
	return true;
}

// Preserve the canonical quantized AABB and internal-pointer decoder exactly.
// Consume the group's address stride here; the stack carries only absolute
// slot plus physical size, with no mapping LOAD and no extra stack bit.
inline uint decompress(const HE2CWBVH::Node& node, BVH::Node children[HE2CWBVH::WIDTH])
{
	::rtm::decompress(node, children);
	int32_t offset = -1;
	const bool small = (node.base_prim_index & NODE_SMALL_FLAG) != 0;
	const uint32_t stride = small ? 1 : 2;
	const uint32_t base = node.base_prim_index & SLOT_MASK;
	for(uint32_t i = 0; i < HE2CWBVH::WIDTH; ++i)
	{
		if((node.i_mask >> i) & 1) continue;
		if((node.o_mask >> i) & 1) ++offset;
		children[i].ptr.prim_cnt = 1;
		const bool valid = valid_child(node, i);
		if(!valid && (offset < 0 ||
			(node.base_prim_index & ~(NODE_SMALL_FLAG | SLOT_MASK))))
		{
			// Internal-only nodes have no usable leaf base (~0 canonically).
			// In a leaf-bearing node, padding instead retains its last-leaf
			// alias below: the legacy box test can admit planar padding.
			children[i].ptr.prim_idx = 0;
			continue;
		}
		if(offset < 0 || (node.base_prim_index & ~(NODE_SMALL_FLAG | SLOT_MASK)))
			throw std::invalid_argument("compact FTB: malformed node leaf base or o_mask");
		const uint64_t slot = uint64_t{base} + uint64_t{static_cast<uint32_t>(offset)} * stride;
		if(slot > SLOT_MASK)
		{
			if(!valid) { children[i].ptr.prim_idx = 0; continue; }
			throw std::out_of_range("compact FTB: decoded leaf slot exceeds 25 bits");
		}
		children[i].ptr.prim_idx = encode_leaf(static_cast<uint32_t>(slot), small ? 64 : 128);
	}
	return HE2CWBVH::WIDTH;
}

struct Statistics
{
	uint64_t canonical_bytes{0};
	uint64_t occupied_bytes{0}; // Payload span, including all deterministic holes.
	uint64_t allocated_block_bytes{0}; // Sum of actual per-block 64/128B sizes.
	uint64_t alignment_padding_bytes{0}; // Gaps before 128B blocks.
	uint64_t small_blocks{0}, large_blocks{0};
	uint64_t leaf_parent_groups{0}, all_small_parent_groups{0}, small_parent_groups{0};
	uint64_t alias_ftbs{0}, within_parent_alias_slots{0}, unreferenced_ftbs{0};
	std::array<uint64_t, 4> required_byte_hist{}; // Encoded 32/64/96/128B, not allocation.
};

struct Layout
{
	std::vector<HE2CWBVH::Node> nodes;
	std::vector<uint8_t> payload;
	std::vector<uint32_t> old_id_to_leaf;
	Statistics stats;
};

namespace detail {

struct DisjointSet
{
	std::vector<uint32_t> parent, size;
	explicit DisjointSet(size_t count) : parent(count), size(count, 1)
	{
		std::iota(parent.begin(), parent.end(), uint32_t{0});
	}
	uint32_t find(uint32_t id)
	{
		while(parent[id] != id) { parent[id] = parent[parent[id]]; id = parent[id]; }
		return id;
	}
	void join(uint32_t a, uint32_t b)
	{
		a = find(a); b = find(b);
		if(a == b) return;
		if(size[a] < size[b]) std::swap(a, b);
		parent[b] = a; size[a] += size[b];
	}
};

struct Group
{
	uint32_t node, base;
	std::vector<uint32_t> ids;
	bool all_small{true};
};

// Check untruncated arithmetic before using canonical BVHPtr's 26-bit result.
// A malformed large base must not wrap back to a valid array entry.
inline std::vector<Group> groups(const std::vector<HE2CWBVH::Node>& nodes,
	size_t ftb_count, std::vector<uint32_t>& owners, Statistics* stats)
{
	std::vector<Group> result;
	for(size_t ni = 0; ni < nodes.size(); ++ni)
	{
		const auto& node = nodes[ni];
		Group group{static_cast<uint32_t>(ni), node.base_prim_index, {}, true};
		int32_t internal_offset = -1, leaf_offset = -1;
		for(uint32_t i = 0; i < HE2CWBVH::WIDTH; ++i)
		{
			const bool internal = (node.i_mask >> i) & 1;
			int32_t& offset = internal ? internal_offset : leaf_offset;
			if((node.o_mask >> i) & 1) ++offset;
			if(!valid_child(node, i))
			{
				if(((node.i_mask | node.o_mask) >> i) & 1)
					throw std::invalid_argument("compact FTB: padding has nonzero pointer masks");
				continue;
			}
			const uint64_t base = internal ? node.base_child_index : node.base_prim_index;
			if(offset < 0)
				throw std::invalid_argument("compact FTB: first valid child lacks o_mask event");
			const uint64_t id = base + static_cast<uint32_t>(offset);
			if(id >= (uint64_t{1} << 26) || (internal ? id >= nodes.size() : id >= ftb_count))
				throw std::out_of_range("compact FTB: invalid canonical child/leaf index");
			if(internal) continue;
			const auto leaf = static_cast<uint32_t>(id);
			if(std::find(group.ids.begin(), group.ids.end(), leaf) != group.ids.end())
			{
				if(stats) ++stats->within_parent_alias_slots;
			}
			else group.ids.push_back(leaf);
		}
		if(group.ids.empty()) continue;
		std::sort(group.ids.begin(), group.ids.end());
		if(group.base != group.ids.front() ||
			uint64_t{group.ids.back()} - group.ids.front() + 1 != group.ids.size())
			throw std::invalid_argument("compact FTB: noncontiguous canonical parent leaf group");
		for(uint32_t id : group.ids) ++owners[id];
		result.push_back(std::move(group));
	}
	return result;
}

inline bool zero_range(const std::vector<uint8_t>& payload, uint64_t begin, uint64_t end)
{
	return std::all_of(payload.begin() + static_cast<size_t>(begin),
		payload.begin() + static_cast<size_t>(end), [](uint8_t byte) { return byte == 0; });
}

} // namespace detail

// Throws on any mismatch. This also works as an independent post-transform
// validation entry point for tests/callers; canonical arrays remain read-only.
inline void validate_layout(const Layout& layout,
	const std::vector<HE2CWBVH::Node>& canonical_nodes,
	const std::vector<FTB>& canonical_ftbs)
{
	if(layout.nodes.size() != canonical_nodes.size() ||
		layout.old_id_to_leaf.size() != canonical_ftbs.size() ||
		layout.payload.size() > uint64_t{canonical_ftbs.size()} * 128 ||
		layout.payload.size() > MAX_PAYLOAD_BYTES)
		throw std::invalid_argument("compact FTB: inconsistent layout arrays or span");
	// Canonical IDs stay in order. Check every copied block, overlap and
	// alignment hole independently of the builder's allocation counters.
	uint64_t cursor = 0;
	for(size_t id = 0; id < canonical_ftbs.size(); ++id)
	{
		const uint32_t encoded = layout.old_id_to_leaf[id];
		const uint32_t bytes = leaf_bytes(encoded), slot = leaf_slot(encoded);
		if(encode_leaf(slot, bytes) != encoded || required_bytes(canonical_ftbs[id]) > bytes)
			throw std::invalid_argument("compact FTB: invalid mapped leaf descriptor or size");
		const uint64_t expected = bytes == 128 ? (cursor + 127) & ~uint64_t{127} : cursor;
		const uint64_t address = uint64_t{slot} * 64;
		if(address != expected || address + bytes > layout.payload.size())
			throw std::out_of_range("compact FTB: payload address/order mismatch");
		if(!detail::zero_range(layout.payload, cursor, address) ||
			std::memcmp(layout.payload.data() + static_cast<size_t>(address), &canonical_ftbs[id], bytes) != 0)
			throw std::invalid_argument("compact FTB: payload changed or padding is not zero");
		cursor = address + bytes;
	}
	if(layout.payload.size() != cursor)
		throw std::invalid_argument("compact FTB: payload end/padding mismatch");
	std::vector<uint32_t> owners(canonical_ftbs.size(), 0);
	const auto parent_groups = detail::groups(canonical_nodes, canonical_ftbs.size(), owners, nullptr);
	for(const auto& group : parent_groups)
	{
		const uint32_t encoded = layout.old_id_to_leaf[group.base];
		const uint64_t base = uint64_t{leaf_slot(encoded)} * 64;
		const uint32_t bytes = leaf_bytes(encoded), stride = bytes;
		for(uint32_t id : group.ids)
			if(leaf_bytes(layout.old_id_to_leaf[id]) != bytes ||
				uint64_t{leaf_slot(layout.old_id_to_leaf[id])} * 64 != base + uint64_t{id - group.base} * stride)
				throw std::invalid_argument("compact FTB: parent/alias component physical stride changed");
	}
	for(size_t ni = 0; ni < canonical_nodes.size(); ++ni)
	{
		auto expected = canonical_nodes[ni];
		expected.base_prim_index = layout.nodes[ni].base_prim_index;
		if(std::memcmp(&expected, &layout.nodes[ni], sizeof(expected)) != 0)
			throw std::invalid_argument("compact FTB: node geometry/internal fields changed");
		BVH::Node old_children[HE2CWBVH::WIDTH], new_children[HE2CWBVH::WIDTH];
		::rtm::decompress(canonical_nodes[ni], old_children);
		compact_ftb::decompress(layout.nodes[ni], new_children);
		bool has_leaf = false;
		for(uint32_t i = 0; i < HE2CWBVH::WIDTH; ++i)
		{
			const auto& before = old_children[i].ptr;
			const auto& after = new_children[i].ptr;
			if(!valid_child(canonical_nodes[ni], i))
			{
				if(!before.is_int && before.prim_idx < canonical_ftbs.size() &&
					(after.is_int || after.prim_cnt != 1 || after.prim_idx != layout.old_id_to_leaf[before.prim_idx]))
					throw std::invalid_argument("compact FTB: padding leaf alias changed");
				continue;
			}
			if(before.is_int)
			{
				if(after.raw != before.raw)
					throw std::invalid_argument("compact FTB: internal decoded pointer changed");
			}
			else
			{
				has_leaf = true;
				if(after.is_int || after.prim_cnt != 1 || after.prim_idx != layout.old_id_to_leaf[before.prim_idx])
					throw std::invalid_argument("compact FTB: parent stride/alias address equation failed");
			}
		}
		if(!has_leaf && layout.nodes[ni].base_prim_index != canonical_nodes[ni].base_prim_index)
			throw std::invalid_argument("compact FTB: unused node base changed");
	}
}

// Compact canonical ID order, retaining explicit 128B alignment padding.
// Overlapping parent leaf ranges form alias components whose members retain
// a common physical stride, even when some individual blocks could be small.
inline Layout build(const std::vector<HE2CWBVH::Node>& canonical_nodes,
	const std::vector<FTB>& canonical_ftbs)
{
	if(canonical_nodes.size() > (uint64_t{1} << 26) || canonical_ftbs.size() > uint64_t{LEAF_SMALL_FLAG})
		throw std::out_of_range("compact FTB: canonical arrays exceed pointer capacity");
	Layout layout;
	layout.nodes = canonical_nodes;
	layout.stats.canonical_bytes = uint64_t{canonical_ftbs.size()} * 128;
	std::vector<uint32_t> owners(canonical_ftbs.size(), 0);
	auto parent_groups = detail::groups(canonical_nodes, canonical_ftbs.size(), owners, &layout.stats);
	layout.stats.leaf_parent_groups = parent_groups.size();
	std::vector<uint32_t> encoded_bytes(canonical_ftbs.size());
	for(size_t id = 0; id < canonical_ftbs.size(); ++id)
	{
		encoded_bytes[id] = required_bytes(canonical_ftbs[id]);
		++layout.stats.required_byte_hist[encoded_bytes[id] / 32 - 1];
	}
	detail::DisjointSet components(canonical_ftbs.size());
	for(auto& group : parent_groups)
	{
		for(uint32_t id : group.ids)
		{
			components.join(group.base, id);
			group.all_small &= encoded_bytes[id] <= 64;
		}
		layout.stats.all_small_parent_groups += group.all_small;
	}
	std::vector<uint8_t> component_large(canonical_ftbs.size(), 0), small(canonical_ftbs.size(), 0);
	for(uint32_t id = 0; id < canonical_ftbs.size(); ++id)
		if(encoded_bytes[id] > 64) component_large[components.find(id)] = 1;
	for(uint32_t id = 0; id < canonical_ftbs.size(); ++id)
	{
		small[id] = owners[id] != 0 && !component_large[components.find(id)];
		layout.stats.small_blocks += small[id] != 0;
		layout.stats.large_blocks += small[id] == 0;
		layout.stats.alias_ftbs += owners[id] > 1;
		layout.stats.unreferenced_ftbs += owners[id] == 0;
	}
	layout.old_id_to_leaf.resize(canonical_ftbs.size());
	uint64_t cursor = 0;
	for(size_t id = 0; id < canonical_ftbs.size(); ++id)
	{
		const uint32_t bytes = small[id] ? 64 : 128;
		const uint64_t address = bytes == 128 ? (cursor + 127) & ~uint64_t{127} : cursor;
		if(address + bytes > MAX_PAYLOAD_BYTES)
			throw std::out_of_range("compact FTB: packed arena exceeds 2GiB range");
		layout.stats.alignment_padding_bytes += address - cursor;
		layout.old_id_to_leaf[id] = encode_leaf(static_cast<uint32_t>(address / 64), bytes);
		layout.stats.allocated_block_bytes += bytes;
		cursor = address + bytes;
	}
	layout.stats.occupied_bytes = cursor;
	if(layout.stats.occupied_bytes > MAX_PAYLOAD_BYTES || layout.stats.occupied_bytes > layout.stats.canonical_bytes)
		throw std::out_of_range("compact FTB: payload span exceeds capacity or original arena");
	layout.payload.resize(static_cast<size_t>(layout.stats.occupied_bytes), 0);
	for(size_t id = 0; id < canonical_ftbs.size(); ++id)
	{
		const uint32_t encoded = layout.old_id_to_leaf[id];
		std::memcpy(layout.payload.data() + uint64_t{leaf_slot(encoded)} * 64, &canonical_ftbs[id], leaf_bytes(encoded));
	}
	for(const auto& group : parent_groups)
	{
		const uint32_t encoded = layout.old_id_to_leaf[group.base];
		const bool group_small = leaf_bytes(encoded) == 64;
		layout.stats.small_parent_groups += group_small;
		const uint64_t base = uint64_t{leaf_slot(encoded)} * 64;
		const uint32_t stride = group_small ? 64 : 128;
		for(uint32_t id : group.ids)
			if(leaf_bytes(layout.old_id_to_leaf[id]) != leaf_bytes(encoded) ||
				uint64_t{leaf_slot(layout.old_id_to_leaf[id])} * 64 != base + uint64_t{id - group.base} * stride)
				throw std::invalid_argument("compact FTB: alias closure cannot preserve parent stride");
		layout.nodes[group.node].base_prim_index = leaf_slot(encoded) |
			(group_small ? NODE_SMALL_FLAG : 0u);
	}
	validate_layout(layout, canonical_nodes, canonical_ftbs);
	return layout;
}

} // namespace rtm::compact_ftb
