// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef VISIBILITY_GRAPH_MSG_GRAPH_VALIDATION_H
#define VISIBILITY_GRAPH_MSG_GRAPH_VALIDATION_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_set>

namespace graph_message_validation {

inline bool SameFrame(const std::string& first, const std::string& second) {
    const auto first_begin = first.find_first_not_of('/');
    const auto second_begin = second.find_first_not_of('/');
    return first_begin != std::string::npos && second_begin != std::string::npos &&
           first.compare(first_begin, std::string::npos, second, second_begin,
                         std::string::npos) == 0;
}

// The ROS 1 and ROS 2 Node messages differ only in the spelling of FreeType.
// Callers supply an accessor so both consumers share the same validation rules.
template <typename Graph, typename FreeType>
bool ValidateGraph(const Graph& graph, const std::string& expected_frame,
                   FreeType free_type, std::string& error) {
    error.clear();
    if (!SameFrame(graph.header.frame_id, expected_frame)) {
        error = "graph frame does not match the configured world frame";
        return false;
    }
    if (graph.size != graph.nodes.size()) {
        error = "graph size does not match node count";
        return false;
    }
    std::unordered_set<std::uint32_t> ids;
    ids.reserve(graph.nodes.size());
    const auto finite_point = [](const auto& point) {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    };
    for (const auto& node : graph.nodes) {
        if (!SameFrame(node.header.frame_id, graph.header.frame_id)) {
            error = "graph node frame does not match graph frame";
            return false;
        }
        if (!ids.insert(node.id).second) {
            error = "duplicate graph node ID";
            return false;
        }
        if (node.surface_dirs.size() != 2 || free_type(node) > 3 ||
            !finite_point(node.position) ||
            !std::all_of(node.surface_dirs.begin(), node.surface_dirs.end(), finite_point)) {
            error = "invalid graph node geometry or type";
            return false;
        }
    }
    for (const auto& node : graph.nodes) {
        const auto known = [&ids](const auto& connections) {
            return std::all_of(connections.begin(), connections.end(),
                               [&ids](const auto id) { return ids.count(id) != 0; });
        };
        if (!known(node.connect_nodes) || !known(node.poly_connects) ||
            !known(node.contour_connects) || !known(node.trajectory_connects)) {
            error = "graph connection refers to an absent node";
            return false;
        }
    }
    return true;
}

}  // namespace graph_message_validation
#endif
