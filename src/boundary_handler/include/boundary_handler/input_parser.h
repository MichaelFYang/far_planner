// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef BOUNDARY_HANDLER_INPUT_PARSER_H
#define BOUNDARY_HANDLER_INPUT_PARSER_H

#include <algorithm>
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
    float x, y, z;
    int polygon_index;
};

inline std::vector<BoundaryVertex> ReadAsciiPly(std::istream& input) {
    std::string line;
    if (!std::getline(input, line) || line != "ply") {
        throw std::runtime_error("Boundary must start with a PLY header");
    }
    bool ascii = false, have_count = false, ended = false;
    bool have_element = false, in_vertex_element = false;
    std::size_t count = 0;
    constexpr std::size_t missing = std::numeric_limits<std::size_t>::max();
    const std::array<std::string, 4> required{{"x", "y", "z", "poly_index"}};
    std::array<std::size_t, 4> columns{{missing, missing, missing, missing}};
    std::vector<int> property_roles;
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
            if (!have_element && second != "vertex") {
                throw std::runtime_error("Boundary vertex must be the first PLY element");
            }
            have_element = true;
            in_vertex_element = second == "vertex";
            if (in_vertex_element) {
                if (have_count) throw std::runtime_error("Duplicate boundary vertex element");
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
        } else if (first == "property" && in_vertex_element) {
            fields >> second >> third;
            if (second == "list" || third.empty()) {
                throw std::runtime_error("Boundary vertex properties must be scalar");
            }
            int role = -1;
            for (std::size_t i = 0; i < required.size(); ++i) {
                if (third == required[i]) {
                    if (columns[i] != missing) throw std::runtime_error("Duplicate boundary vertex property");
                    columns[i] = property_roles.size();
                    role = static_cast<int>(i);
                }
            }
            property_roles.push_back(role);
        }
    }
    if (!ascii || !have_count || !ended ||
        std::any_of(columns.begin(), columns.end(), [missing](std::size_t index) { return index == missing; })) {
        throw std::runtime_error("Boundary requires ASCII PLY with x, y, z, and poly_index vertex properties");
    }
    std::vector<BoundaryVertex> vertices;
    vertices.reserve(count);
    while (vertices.size() < count && std::getline(input, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        std::istringstream fields(line);
        BoundaryVertex vertex{};
        std::array<double, 4> values{};
        for (std::size_t column = 0; column < property_roles.size(); ++column) {
            double value = 0.0;
            if (!(fields >> value)) throw std::runtime_error("Invalid boundary vertex values");
            if (property_roles[column] >= 0) values[static_cast<std::size_t>(property_roles[column])] = value;
        }
        std::string extra;
        if (fields >> extra) throw std::runtime_error("Too many boundary vertex values");
        vertex.x = static_cast<float>(values[0]);
        vertex.y = static_cast<float>(values[1]);
        vertex.z = static_cast<float>(values[2]);
        const double polygon_index = values[3];
        if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) ||
            !std::isfinite(vertex.z) || !std::isfinite(polygon_index) ||
            std::trunc(polygon_index) != polygon_index ||
            polygon_index < static_cast<double>(std::numeric_limits<int>::min()) ||
            polygon_index > static_cast<double>(std::numeric_limits<int>::max())) {
            throw std::runtime_error("Invalid boundary vertex values");
        }
        vertex.polygon_index = static_cast<int>(polygon_index);
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
