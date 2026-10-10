#pragma once

// Build-time HE2 leaf partition search. No device format or decoder changes.
#ifndef __riscv
#include <chrono>
#include <cmath>
#include <limits>
#include "compact-ftb.hpp"
#include "ftb-prefix.hpp"

namespace rtm::leaf_grouping {

struct Costs
{
	// Effective geometric weights. Overlapped ray waits and pipeline latency
	// cannot be added to predict frame cycles; sector requests can share a
	// cache line. Keep measured cycles/correctness as the acceptance criteria.
	// Two triangle issue ticks per global cycle; terminal FIFO latency plus
	// decode is approximately 12.5 + 0.5*n cycles per isolated leaf. Sector
	// delivery is serialized on the leaf's port, but waits can overlap.
	double triangle{0.5}, sector{1}, leaf{12.5};
	bool preserve_stride{false}, pareto_filter{false};
	double block(uint32_t triangles, uint32_t sectors) const
	{ return triangle * triangles + sector * sectors + leaf; }
	void validate() const
	{
		if(!std::isfinite(triangle) || !std::isfinite(sector) || !std::isfinite(leaf) ||
		   triangle < 0 || sector < 0 || leaf < 0 || triangle + sector + leaf <= 0)
			throw std::invalid_argument("Leaf grouping costs must be finite, nonnegative, and not all zero");
	}
};

struct Statistics
{
	uint64_t eligible{0}, searchable{0}, changed_parents{0}, changed_blocks{0};
	uint64_t skipped_shared{0}, skipped_mixed{0}, skipped_interval{0};
	uint64_t rejected_candidates{0}, checked_primitives{0}, elapsed_us{0};
	double baseline_cost{0}, selected_cost{0};
	double baseline_triangles{0}, selected_triangles{0}, baseline_sectors{0}, selected_sectors{0};
	uint64_t filtered_plans{0};
};

namespace detail {
inline AABB bounds(const HE2CWBVH::Node& node, const QAABB8& q)
{
	const vec3 p(as_f32(node.exts[0].anchor << 8), as_f32(node.exts[1].anchor << 8), as_f32(node.exts[2].anchor << 8));
	const vec3 e(as_f32(node.exts[0].exp << 23), as_f32(node.exts[1].exp << 23), as_f32(node.exts[2].exp << 23));
	AABB box;
	box.min = vec3(q.min[0], q.min[1], q.min[2]) * e + p;
	box.max = vec3(q.max[0], q.max[1], q.max[2]) * e + p;
	return box;
}

inline bool contains(const AABB& outer, const AABB& inner)
{
	for(uint32_t a = 0; a < 3; ++a)
		if(!std::isfinite(outer.min[a]) || !std::isfinite(outer.max[a]) ||
		   outer.min[a] > inner.min[a] || outer.max[a] < inner.max[a]) return false;
	return true;
}

inline bool quantize(const HE2CWBVH::Node& node, const AABB& box, QAABB8& q)
{
	for(uint32_t a = 0; a < 3; ++a)
	{
		const float p = as_f32(node.exts[a].anchor << 8), e = as_f32(node.exts[a].exp << 23);
		if(e == 0.0f)
		{
			if(box.min[a] != p || box.max[a] != p) return false;
			q.min[a] = q.max[a] = 0;
			continue;
		}
		const float lo = std::floor((box.min[a] - p) * (1.0f / e));
		const float hi = std::ceil((box.max[a] - p) * (1.0f / e));
		if(!std::isfinite(lo) || !std::isfinite(hi) || lo < 0 || hi > 255 || lo > hi) return false;
		q.min[a] = static_cast<uint8_t>(lo);
		q.max[a] = static_cast<uint8_t>(hi);
	}
	// Match the production FP32 dequantization, including its rounding.
	// Move a lattice endpoint outwards if reciprocal rounding made it narrow.
	for(uint32_t a = 0; a < 3; ++a)
	{
		while(bounds(node, q).min[a] > box.min[a])
		{
			if(q.min[a] == 0) return false;
			--q.min[a];
		}
		while(bounds(node, q).max[a] < box.max[a])
		{
			if(q.max[a] == 255) return false;
			++q.max[a];
		}
	}
	return contains(bounds(node, q), box);
}

inline bool geometry_matches(const FTB& block, const Mesh& mesh)
{
	const uint32_t count = block.tri_cnt + 1;
	if(block.is_int || count > 3 || uint64_t(block.prim_idx) + count > mesh.vertex_indices.size()) return false;
	IntersectionTriangle triangles[FTB::MAX_TRIS];
	if(rtm::decompress(block, triangles) != count) return false;
	for(uint32_t i = 0; i < count; ++i)
	{
		if(triangles[i].id != block.prim_idx + i) return false;
		const Triangle source = mesh.get_triangle(triangles[i].id);
		for(uint32_t v = 0; v < 3; ++v)
			for(uint32_t a = 0; a < 3; ++a)
				if(as_u32(source.vrts[v][a]) != as_u32(triangles[i].tri.vrts[v][a])) return false;
	}
	return true;
}

struct Candidate
{
	FTB block{};
	QAABB8 box{};
	double area{0};
	uint32_t bytes{0};
	bool valid{false};
};

struct Plan
{
	std::array<uint32_t, HE2CWBVH::WIDTH> lengths{};
	double cost{std::numeric_limits<double>::infinity()};
	double triangles{0}, sectors{0};
};

// Parent stride is uniform. Search exact two/four-sector plans separately;
// the large-block state makes four-sector plans contain at least one >64B block.
inline Plan search(const std::vector<std::array<Candidate, 4>>& candidates,
	uint32_t n, uint32_t k, uint32_t sectors, const Costs& weights = {})
{
	if(k > HE2CWBVH::WIDTH || n < k || n > 3 * k || candidates.size() <= n ||
	   (sectors != 2 && sectors != 4)) return {};
	using Row = std::array<std::array<double, 2>, 3 * HE2CWBVH::WIDTH + 1>;
	std::array<Row, HE2CWBVH::WIDTH + 1> cost;
	struct Step { uint32_t len{0}, large{0}; };
	std::array<std::array<std::array<Step, 2>, 3 * HE2CWBVH::WIDTH + 1>, HE2CWBVH::WIDTH + 1> previous{};
	for(auto& row : cost) for(auto& entry : row) entry.fill(std::numeric_limits<double>::infinity());
	cost[0][0][0] = 0;
	for(uint32_t g = 1; g <= k; ++g)
		for(uint32_t end = g; end <= n && end <= 3 * g; ++end)
			for(uint32_t len = 1; len <= 3 && len <= end; ++len)
			{
				const auto& candidate = candidates[end - len][len];
				if(!candidate.valid || candidate.bytes > sectors * 32) continue;
				for(uint32_t large = 0; large < 2; ++large)
				{
					const uint32_t next_large = large || candidate.bytes > 64;
					const double next = cost[g - 1][end - len][large] + candidate.area * weights.block(len, sectors);
					if(next < cost[g][end][next_large])
					{ cost[g][end][next_large] = next; previous[g][end][next_large] = {len, large}; }
				}
			}
	Plan plan;
	uint32_t large = sectors == 4, end = n;
	if(!std::isfinite(cost[k][n][large])) return plan;
	for(uint32_t g = k; g > 0; --g)
	{
		const auto step = previous[g][end][large];
		plan.lengths[g - 1] = step.len;
		large = step.large;
		end -= plan.lengths[g - 1];
	}
	plan.cost = 0;
	uint32_t cursor = 0;
	for(uint32_t i = 0; i < k; ++i)
	{
		const double area = candidates[cursor][plan.lengths[i]].area;
		plan.cost += area * weights.block(plan.lengths[i], sectors);
		plan.triangles += area * plan.lengths[i];
		plan.sectors += area * sectors;
		cursor += plan.lengths[i];
	}
	return plan;
}
} // namespace detail

inline Statistics apply(std::vector<HE2CWBVH::Node>& nodes, std::vector<FTB>& ftbs, const Mesh& mesh, const Costs& weights = {})
{
	weights.validate();
	const auto start_time = std::chrono::steady_clock::now();
	Statistics stats;
	std::vector<uint32_t> owners(ftbs.size(), 0);
	const auto groups = compact_ftb::detail::groups(nodes, ftbs.size(), owners, nullptr);
	for(const auto& group : groups)
	{
		auto& node = nodes[group.node];
		std::vector<uint32_t> slots;
		bool mixed = false;
		for(uint32_t s = 0; s < HE2CWBVH::WIDTH; ++s)
			if(compact_ftb::valid_child(node, s))
			{
				if((node.i_mask >> s) & 1) mixed = true;
				else slots.push_back(s);
			}
		if(mixed) { ++stats.skipped_mixed; continue; }
		if(slots.size() != group.ids.size() ||
		   std::any_of(group.ids.begin(), group.ids.end(), [&](uint32_t id) { return owners[id] != 1; }))
		{ ++stats.skipped_shared; continue; }
		const uint32_t k = static_cast<uint32_t>(slots.size());
		if(k == 0) continue;
		const uint32_t first = ftbs[group.ids.front()].prim_idx;
		uint32_t n = 0, baseline_sectors = 2;
		detail::Plan baseline;
		bool continuous = true;
		AABB parent_box;
		for(uint32_t i = 0; i < k; ++i)
		{
			const auto& block = ftbs[group.ids[i]];
			const uint32_t len = block.tri_cnt + 1;
			if(block.prim_idx != uint64_t(first) + n || len > 3) continuous = false;
			baseline.lengths[i] = len;
			n += len;
			if(ftb_prefix::required_bytes(ftb_prefix::transform_block(block)) > 64) baseline_sectors = 4;
			parent_box.add(detail::bounds(node, node.qaabb[slots[i]]));
		}
		if(!continuous || n > 3 * k || uint64_t(first) + n > mesh.vertex_indices.size())
		{ ++stats.skipped_interval; continue; }
		++stats.eligible;
		// A positive common denominator changes only the reported cost scale.
		const double parent_area = parent_box.surface_area();
		if(parent_area <= 0 || !std::isfinite(parent_area)) continue;
		baseline.cost = 0;
		for(uint32_t i = 0; i < k; ++i)
		{
			const double area = detail::bounds(node, node.qaabb[slots[i]]).surface_area() / parent_area;
			baseline.cost += area * weights.block(baseline.lengths[i], baseline_sectors);
			baseline.triangles += area * baseline.lengths[i];
			baseline.sectors += area * baseline_sectors;
		}
		std::vector<std::array<detail::Candidate, 4>> candidates(n + 1);
		for(uint32_t offset = 0; offset < n; ++offset)
		{
			AABB exact_box;
			for(uint32_t len = 1; len <= 3 && offset + len <= n; ++len)
			{
				const auto triangle = mesh.get_triangle(first + offset + len - 1);
				for(uint32_t v = 0; v < 3; ++v) exact_box.add(triangle.vrts[v]);
				auto& candidate = candidates[offset][len];
				if(!rtm::compress(first + offset, len, mesh, &candidate.block) ||
				   !detail::geometry_matches(candidate.block, mesh) || !detail::quantize(node, exact_box, candidate.box))
				{ ++stats.rejected_candidates; continue; }
				candidate.bytes = ftb_prefix::required_bytes(ftb_prefix::transform_block(candidate.block));
				candidate.area = detail::bounds(node, candidate.box).surface_area() / parent_area;
				candidate.valid = candidate.bytes <= 128 && std::isfinite(candidate.area);
			}
		}
		// Include original intervals/boxes verbatim, so rounding or equal costs
		// never force a change to the verified baseline.
		uint32_t cursor = 0;
		for(uint32_t i = 0; i < k; ++i)
		{
			auto& candidate = candidates[cursor][baseline.lengths[i]];
			candidate.block = ftbs[group.ids[i]];
			candidate.box = node.qaabb[slots[i]];
			candidate.area = detail::bounds(node, candidate.box).surface_area() / parent_area;
			candidate.bytes = ftb_prefix::required_bytes(ftb_prefix::transform_block(candidate.block));
			candidate.valid = true;
			cursor += baseline.lengths[i];
		}
		if(n > k && n < 3 * k && k > 1) ++stats.searchable;
		detail::Plan selected = baseline;
		for(uint32_t sectors : {2u, 4u})
		{
			if(weights.preserve_stride && sectors != baseline_sectors) continue;
			const auto plan = detail::search(candidates, n, k, sectors, weights);
			// Conservative selection filter on the weighted optimum, not a
			// constrained Pareto DP or a guarantee on measured ray work/cycles.
			if(weights.pareto_filter &&
			   (plan.triangles > baseline.triangles + 1e-12 * std::max(1.0, baseline.triangles) ||
			    plan.sectors > baseline.sectors + 1e-12 * std::max(1.0, baseline.sectors)))
			{ ++stats.filtered_plans; continue; }
			if(plan.cost < selected.cost - 1e-12 * std::max(1.0, baseline.cost)) selected = plan;
		}
		stats.baseline_cost += baseline.cost;
		stats.selected_cost += selected.cost;
		stats.baseline_triangles += baseline.triangles;
		stats.selected_triangles += selected.triangles;
		stats.baseline_sectors += baseline.sectors;
		stats.selected_sectors += selected.sectors;
		if(selected.lengths == baseline.lengths) continue;
		++stats.changed_parents; cursor = 0;
		for(uint32_t i = 0; i < k; ++i)
		{
			const auto& candidate = candidates[cursor][selected.lengths[i]];
			if(std::memcmp(&ftbs[group.ids[i]], &candidate.block, sizeof(FTB)) != 0) ++stats.changed_blocks;
			ftbs[group.ids[i]] = candidate.block;
			node.qaabb[slots[i]] = candidate.box;
			cursor += selected.lengths[i];
		}
	}
	// Check the full primitive cover and FP32 source geometry after regrouping.
	// This also catches signed-zero vertex coalescing in the canonical codec.
	std::vector<uint32_t> coverage(mesh.vertex_indices.size(), 0);
	for(const auto& block : ftbs)
	{
		if(!detail::geometry_matches(block, mesh)) throw std::runtime_error("Leaf grouping changed primitive ID/FP32 geometry");
		for(uint32_t t = 0; t <= block.tri_cnt; ++t) ++coverage[block.prim_idx + t];
	}
	if(std::any_of(coverage.begin(), coverage.end(), [](uint32_t count) { return count != 1; }))
		throw std::runtime_error("Leaf grouping changed primitive coverage");
	for(const auto& group : groups)
	{
		const auto& node = nodes[group.node];
		BVH::Node children[HE2CWBVH::WIDTH]; rtm::decompress(node, children);
		for(uint32_t s = 0; s < HE2CWBVH::WIDTH; ++s)
			if(compact_ftb::valid_child(node, s) && !children[s].ptr.is_int)
			{
				const auto& block = ftbs[children[s].ptr.prim_idx];
				AABB exact;
				for(uint32_t t = 0; t <= block.tri_cnt; ++t)
					for(const auto& vertex : mesh.get_triangle(block.prim_idx + t).vrts) exact.add(vertex);
				if(!detail::contains(children[s].aabb, exact)) throw std::runtime_error("Leaf grouping box is not conservative");
			}
	}
	stats.checked_primitives = coverage.size();
	stats.elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count();
	return stats;
}
} // namespace rtm::leaf_grouping
#endif
