// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef FAR_PLANNER_POINT_CLOUD_VALIDATION_H
#define FAR_PLANNER_POINT_CLOUD_VALIDATION_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace far_planner {

// PCL copies PointCloud2 fields directly into PointXYZI without checking the
// input buffer or converting byte order. Validate the layout before conversion.
template <typename PointCloud2>
bool ValidPointCloudLayout(const PointCloud2& cloud, bool require_intensity) {
    if (cloud.width == 0 || cloud.height == 0 || cloud.point_step == 0) return false;
    const std::uint64_t row_bytes = static_cast<std::uint64_t>(cloud.width) * cloud.point_step;
    if (row_bytes > cloud.row_step ||
        static_cast<std::uint64_t>(cloud.row_step) * cloud.height != cloud.data.size() ||
        static_cast<std::uint64_t>(cloud.width) * cloud.height >
            std::numeric_limits<std::uint32_t>::max()) return false;

    const std::uint16_t one = 1;
    const bool native_big_endian = *reinterpret_cast<const unsigned char*>(&one) == 0;
    if (cloud.is_bigendian != native_big_endian) return false;

    using Field = std::decay_t<decltype(cloud.fields.front())>;
    std::array<bool, 4> seen{};
    std::array<std::uint32_t, 4> offsets{};
    for (const auto& field : cloud.fields) {
        std::size_t index;
        if (field.name == "x") index = 0;
        else if (field.name == "y") index = 1;
        else if (field.name == "z") index = 2;
        else if (field.name == "intensity") index = 3;
        else continue;
        if (field.datatype != Field::FLOAT32 || field.count != 1 ||
            static_cast<std::uint64_t>(field.offset) + sizeof(float) > cloud.point_step ||
            seen[index]) {
            return false;
        }
        for (std::size_t i = 0; i < seen.size(); ++i) {
            if (seen[i] && field.offset < static_cast<std::uint64_t>(offsets[i]) + sizeof(float) &&
                offsets[i] < static_cast<std::uint64_t>(field.offset) + sizeof(float)) return false;
        }
        seen[index] = true;
        offsets[index] = field.offset;
    }
    return seen[0] && seen[1] && seen[2] && (!require_intensity || seen[3]);
}

}  // namespace far_planner
#endif
