#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

// ray.hpp uses vec2 in Hit as well as vec3 in Ray.
#include "rtm/vec2.hpp"
#include "rtm/ray.hpp"

namespace ray_input {

// Records stay in file order. The original ID identifies the framebuffer slot;
// loading a differently ordered file must not implicitly reorder submissions.
struct FrozenRays
{
	uint32_t width = 0;
	uint32_t height = 0;
	std::vector<rtm::Ray> rays;
	std::vector<uint64_t> original_ray_ids;
};

namespace detail {

// All integers are little endian. Header (40 bytes): magic[8], version:u32,
// header_bytes:u32, width:u32, height:u32, count:u64, record_bytes:u32,
// reserved:u32. Each record (40 bytes): original_ray_id:u64 followed by raw
// IEEE binary32 bits for ox, oy, oz, t_min, dx, dy, dz, t_max.
inline constexpr std::array<uint8_t, 8> magic = {'A', 'R', 'C', 'H', 'R', 'A', 'Y', 0};
inline constexpr uint32_t version = 1;
inline constexpr uint32_t header_bytes = 40;
inline constexpr uint32_t record_bytes = 40;
static_assert(sizeof(float) == sizeof(uint32_t) && std::numeric_limits<float>::is_iec559,
	"Frozen rays require IEEE binary32 floats");

[[noreturn]] inline void fail(const std::string& path, const std::string& reason)
{
	throw std::runtime_error("Ray file '" + path + "': " + reason);
}

inline uint64_t checked_count(const std::string& path, uint32_t width, uint32_t height)
{
	if(width == 0 || height == 0) fail(path, "width and height must be nonzero");
	const uint64_t count = uint64_t(width) * height;
	if(count > (std::numeric_limits<uint64_t>::max() - header_bytes) / record_bytes)
		fail(path, "resolution exceeds the file format size limit");
	return count;
}

inline void put_u32(uint8_t* dst, uint32_t value)
{
	for(unsigned i = 0; i < 4; ++i) dst[i] = uint8_t(value >> (8 * i));
}

inline void put_u64(uint8_t* dst, uint64_t value)
{
	for(unsigned i = 0; i < 8; ++i) dst[i] = uint8_t(value >> (8 * i));
}

inline uint32_t get_u32(const uint8_t* src)
{
	uint32_t value = 0;
	for(unsigned i = 0; i < 4; ++i) value |= uint32_t(src[i]) << (8 * i);
	return value;
}

inline uint64_t get_u64(const uint8_t* src)
{
	uint64_t value = 0;
	for(unsigned i = 0; i < 8; ++i) value |= uint64_t(src[i]) << (8 * i);
	return value;
}

inline std::array<const float*, 8> fields(const rtm::Ray& ray)
{
	return {&ray.o.x, &ray.o.y, &ray.o.z, &ray.t_min,
		&ray.d.x, &ray.d.y, &ray.d.z, &ray.t_max};
}

inline std::array<float*, 8> fields(rtm::Ray& ray)
{
	return {&ray.o.x, &ray.o.y, &ray.o.z, &ray.t_min,
		&ray.d.x, &ray.d.y, &ray.d.z, &ray.t_max};
}

inline void validate_ids(const std::string& path, uint64_t count,
	const std::vector<uint64_t>& ids)
{
	if(ids.size() != count) fail(path, "original ID count differs from the ray count");
	std::vector<bool> seen(ids.size(), false);
	for(uint64_t id : ids)
	{
		if(id >= count) fail(path, "original ray ID " + std::to_string(id) + " is out of range");
		if(seen[size_t(id)]) fail(path, "duplicate original ray ID " + std::to_string(id));
		seen[size_t(id)] = true;
	}
}

inline void write_file(const std::string& path, uint32_t width, uint32_t height,
	const std::vector<rtm::Ray>& rays, const std::vector<uint64_t>* ids)
{
	const uint64_t count = checked_count(path, width, height);
	if(rays.size() != count) fail(path, "ray count differs from width * height");
	if(ids) validate_ids(path, count, *ids);
	std::ofstream output(path, std::ios::binary | std::ios::trunc);
	if(!output) fail(path, "cannot open for writing");
	std::array<uint8_t, header_bytes> header{};
	std::memcpy(header.data(), magic.data(), magic.size());
	put_u32(header.data() + 8, version);
	put_u32(header.data() + 12, header_bytes);
	put_u32(header.data() + 16, width);
	put_u32(header.data() + 20, height);
	put_u64(header.data() + 24, count);
	put_u32(header.data() + 32, record_bytes);
	output.write(reinterpret_cast<const char*>(header.data()), header.size());
	for(size_t i = 0; i < rays.size(); ++i)
	{
		std::array<uint8_t, record_bytes> record{};
		put_u64(record.data(), ids ? (*ids)[i] : uint64_t(i));
		const auto values = fields(rays[i]);
		for(size_t j = 0; j < values.size(); ++j)
		{
			uint32_t bits;
			std::memcpy(&bits, values[j], sizeof(bits));
			put_u32(record.data() + 8 + 4 * j, bits);
		}
		output.write(reinterpret_cast<const char*>(record.data()), record.size());
		if(!output) fail(path, "failed while writing ray record " + std::to_string(i));
	}
	output.close();
	if(!output) fail(path, "failed to finish writing");
}

} // namespace detail

// Canonical export: ray i retains original framebuffer ID i, including duplicate
// rays and inactive entries. No filtering or floating point operations occur.
inline void write_file(const std::string& path, uint32_t width, uint32_t height,
	const std::vector<rtm::Ray>& rays)
{
	detail::write_file(path, width, height, rays, nullptr);
}

inline void write_file(const std::string& path, const FrozenRays& rays)
{
	detail::write_file(path, rays.width, rays.height, rays.rays, &rays.original_ray_ids);
}

inline FrozenRays read_file(const std::string& path, uint32_t expected_width, uint32_t expected_height)
{
	std::ifstream input(path, std::ios::binary | std::ios::ate);
	if(!input) detail::fail(path, "cannot open for reading");
	const std::streampos end = input.tellg();
	if(end < std::streampos(0)) detail::fail(path, "cannot determine file size");
	const uint64_t file_bytes = uint64_t(std::streamoff(end));
	if(file_bytes < detail::header_bytes) detail::fail(path, "truncated header");
	input.seekg(0);
	std::array<uint8_t, detail::header_bytes> header{};
	input.read(reinterpret_cast<char*>(header.data()), header.size());
	if(!input) detail::fail(path, "cannot read header");
	if(std::memcmp(header.data(), detail::magic.data(), detail::magic.size()) != 0)
		detail::fail(path, "invalid magic; expected ARCHRAY");
	const uint32_t version = detail::get_u32(header.data() + 8);
	if(version != detail::version)
		detail::fail(path, "unsupported version " + std::to_string(version));
	if(detail::get_u32(header.data() + 12) != detail::header_bytes ||
		detail::get_u32(header.data() + 32) != detail::record_bytes ||
		detail::get_u32(header.data() + 36) != 0)
		detail::fail(path, "invalid header size, record size, or reserved field");
	FrozenRays result;
	result.width = detail::get_u32(header.data() + 16);
	result.height = detail::get_u32(header.data() + 20);
	const uint64_t count = detail::checked_count(path, result.width, result.height);
	if(result.width != expected_width || result.height != expected_height)
		detail::fail(path, "resolution " + std::to_string(result.width) + "x" +
			std::to_string(result.height) + " differs from requested " +
			std::to_string(expected_width) + "x" + std::to_string(expected_height));
	if(detail::get_u64(header.data() + 24) != count)
		detail::fail(path, "header ray count differs from width * height");
	const uint64_t expected_bytes = detail::header_bytes + count * detail::record_bytes;
	if(file_bytes != expected_bytes)
		detail::fail(path, file_bytes < expected_bytes ? "truncated ray records" : "unexpected trailing bytes");
	if(count > result.rays.max_size() || count > result.original_ray_ids.max_size())
		detail::fail(path, "ray count exceeds host container limits");
	result.rays.resize(size_t(count));
	result.original_ray_ids.resize(size_t(count));
	std::vector<bool> seen(size_t(count), false);
	for(size_t i = 0; i < result.rays.size(); ++i)
	{
		std::array<uint8_t, detail::record_bytes> record{};
		input.read(reinterpret_cast<char*>(record.data()), record.size());
		if(!input) detail::fail(path, "cannot read ray record " + std::to_string(i));
		const uint64_t id = detail::get_u64(record.data());
		if(id >= count) detail::fail(path, "original ray ID " + std::to_string(id) + " is out of range");
		if(seen[size_t(id)]) detail::fail(path, "duplicate original ray ID " + std::to_string(id));
		seen[size_t(id)] = true;
		result.original_ray_ids[i] = id;
		const auto values = detail::fields(result.rays[i]);
		for(size_t j = 0; j < values.size(); ++j)
		{
			const uint32_t bits = detail::get_u32(record.data() + 8 + 4 * j);
			std::memcpy(values[j], &bits, sizeof(bits));
		}
	}
	// Also detect a file that grew after the initial size check.
	if(input.peek() != std::char_traits<char>::eof()) detail::fail(path, "unexpected trailing bytes");
	return result;
}

} // namespace ray_input
