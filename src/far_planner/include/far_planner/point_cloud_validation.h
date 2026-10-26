// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef FAR_PLANNER_POINT_CLOUD_VALIDATION_H
#define FAR_PLANNER_POINT_CLOUD_VALIDATION_H

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
    bool has_x = false, has_y = false, has_z = false, has_intensity = false;
    for (const auto& field : cloud.fields) {
        if (field.name != "x" && field.name != "y" && field.name != "z" &&
            field.name != "intensity") continue;
        if (field.datatype != Field::FLOAT32 || field.count != 1 ||
            static_cast<std::uint64_t>(field.offset) + sizeof(float) > cloud.point_step) {
            return false;
        }
        if (field.name == "x") has_x = true;
        else if (field.name == "y") has_y = true;
        else if (field.name == "z") has_z = true;
        else has_intensity = true;
    }
    return has_x && has_y && has_z && (!require_intensity || has_intensity);
}

}  // namespace far_planner
#endif
