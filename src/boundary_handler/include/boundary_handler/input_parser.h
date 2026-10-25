// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef BOUNDARY_HANDLER_INPUT_PARSER_H
#define BOUNDARY_HANDLER_INPUT_PARSER_H

#include <array>
#include <cmath>
#include <cstddef>
#include <istream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace boundary_handler {

struct BoundaryVertex {
    float x, y, z, polygon_index;
};

inline std::vector<BoundaryVertex> ReadAsciiPly(std::istream& input) {
    std::string line;
    if (!std::getline(input, line) || line != "ply") {
        throw std::runtime_error("Boundary must start with a PLY header");
    }
    bool ascii = false, have_count = false, ended = false;
    std::size_t count = 0;
    for (std::size_t header_lines = 0; header_lines < 256 && std::getline(input, line); ++header_lines) {
        if (line.size() > 4096) throw std::runtime_error("Boundary header line too long");
        if (line == "end_header") {
            ended = true;
            break;
        }
        std::istringstream fields(line);
        std::string first, second, third;
        fields >> first;
        if (first == "format") {
            fields >> second;
            ascii = second == "ascii";
        } else if (first == "element") {
            fields >> second;
            if (second == "vertex") {
                fields >> third;
                if (third.empty() || third.find_first_not_of("0123456789") != std::string::npos) {
                    throw std::runtime_error("Invalid boundary vertex count");
                }
                const auto value = std::stoull(third);
                if (value == 0 || value > 1000000ULL) {
                    throw std::runtime_error("Boundary vertex count must be in [1, 1000000]");
                }
                count = static_cast<std::size_t>(value);
                have_count = true;
            }
        }
    }
    if (!ascii || !have_count || !ended) {
        throw std::runtime_error("Boundary requires ASCII PLY, a vertex count, and end_header");
    }
    std::vector<BoundaryVertex> vertices;
    vertices.reserve(count);
    while (vertices.size() < count && std::getline(input, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        std::istringstream fields(line);
        BoundaryVertex vertex{};
        if (!(fields >> vertex.x >> vertex.y >> vertex.z >> vertex.polygon_index) ||
            !std::isfinite(vertex.x) || !std::isfinite(vertex.y) ||
            !std::isfinite(vertex.z) || !std::isfinite(vertex.polygon_index) ||
            vertex.polygon_index < static_cast<float>(std::numeric_limits<int>::min()) ||
            vertex.polygon_index >= static_cast<float>(std::numeric_limits<int>::max())) {
            throw std::runtime_error("Invalid boundary vertex values");
        }
        vertices.push_back(vertex);
    }
    if (vertices.size() != count) throw std::runtime_error("Truncated boundary vertex data");
    return vertices;
}

inline std::array<float, 3> ReadTrajectory(std::istream& input) {
    std::array<float, 3> position{};
    if (!(input >> position[0] >> position[1] >> position[2]) ||
        !std::isfinite(position[0]) || !std::isfinite(position[1]) ||
        !std::isfinite(position[2])) {
        throw std::runtime_error("Trajectory requires finite X, Y, and Z values");
    }
    return position;
}

}  // namespace boundary_handler
#endif
