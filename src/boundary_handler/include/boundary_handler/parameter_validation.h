// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef BOUNDARY_HANDLER_PARAMETER_VALIDATION_H
#define BOUNDARY_HANDLER_PARAMETER_VALIDATION_H

#include <cmath>
#include <stdexcept>
#include <string>

namespace boundary_handler {

inline std::string NormalizeWorldFrame(std::string frame) {
    const auto first = frame.find_first_not_of('/');
    if (first == std::string::npos) {
        throw std::invalid_argument("world_frame must name a nonempty frame");
    }
    frame.erase(0, first);
    if (frame.find_first_of(" \t\n\r") != std::string::npos) {
        throw std::invalid_argument("world_frame must not contain whitespace");
    }
    return frame;
}

inline void ValidateGeometryParameters(float visual_scale_ratio, float height_tolz) {
    if (!std::isfinite(visual_scale_ratio) || visual_scale_ratio <= 0.0f) {
        throw std::invalid_argument("visual_scale_ratio must be finite and positive");
    }
    if (!std::isfinite(height_tolz) || height_tolz < 0.0f) {
        throw std::invalid_argument("height_tolz must be finite and nonnegative");
    }
}

inline std::string ResolveFilePath(const std::string& folder, const std::string& filename) {
    if (filename.empty()) throw std::invalid_argument("file name must be nonempty");
    if (filename.front() == '/') return filename;
    if (folder.empty()) throw std::invalid_argument("folder_path must be nonempty for relative files");
    return folder + (folder.back() == '/' ? "" : "/") + filename;
}

}  // namespace boundary_handler

#endif
