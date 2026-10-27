#ifndef GRAPH_DECODER_GRAPH_FILE_PARSER_H
#define GRAPH_DECODER_GRAPH_FILE_PARSER_H

#include <array>
#include <cmath>
#include <cstdint>
#include <istream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace graph_file {

struct NodeRecord {
    std::uint32_t id;
    unsigned int free_type;
    std::array<float, 9> coordinates;
    std::array<bool, 4> flags;
    std::array<std::vector<std::uint32_t>, 4> connections;
};

inline std::uint32_t ParseId(const std::string& token) {
    if (token.empty() || token[0] == '-') throw std::runtime_error("invalid node ID");
    std::size_t end = 0;
    const auto value = std::stoull(token, &end);
    if (end != token.size() || value > std::numeric_limits<std::uint32_t>::max())
        throw std::runtime_error("node ID outside uint32 range");
    return static_cast<std::uint32_t>(value);
}

inline unsigned int ParseSmallInteger(const std::string& token, unsigned int limit) {
    const auto value = ParseId(token);
    if (value > limit) throw std::runtime_error("invalid enum or flag");
    return value;
}

inline float ParseCoordinate(const std::string& token) {
    std::size_t end = 0;
    const float value = std::stof(token, &end);
    if (end != token.size() || !std::isfinite(value))
        throw std::runtime_error("invalid coordinate");
    return value;
}

inline NodeRecord ParseLine(const std::string& line) {
    std::istringstream input(line);
    std::vector<std::string> tokens;
    std::string token;
    while (input >> token) tokens.push_back(token);
    if (tokens.size() < 18) throw std::runtime_error("incomplete graph node");

    NodeRecord node{};
    node.id = ParseId(tokens[0]);
    node.free_type = ParseSmallInteger(tokens[1], 3);
    for (std::size_t i = 0; i < node.coordinates.size(); ++i)
        node.coordinates[i] = ParseCoordinate(tokens[i + 2]);
    for (std::size_t i = 0; i < node.flags.size(); ++i)
        node.flags[i] = ParseSmallInteger(tokens[i + 11], 1) == 1;

    std::size_t section = 0;
    for (std::size_t i = 15; i < tokens.size(); ++i) {
        if (tokens[i] == "|") {
            if (++section > 3) throw std::runtime_error("too many graph sections");
        } else {
            node.connections[section].push_back(ParseId(tokens[i]));
        }
    }
    if (section != 3) throw std::runtime_error("missing graph section");
    return node;
}

inline std::vector<NodeRecord> ParseGraph(std::istream& input) {
    std::vector<NodeRecord> nodes;
    std::unordered_set<std::uint32_t> ids;
    std::string line;
    while (std::getline(input, line)) {
        if (line.size() > 65536 || nodes.size() >= 1000000)
            throw std::runtime_error("graph file exceeds size limit");
        const auto node = ParseLine(line);
        if (!ids.insert(node.id).second) throw std::runtime_error("duplicate graph node ID");
        nodes.push_back(node);
    }
    if (!input.eof() || nodes.empty()) throw std::runtime_error("empty or unreadable graph file");
    for (const auto& node : nodes)
        for (const auto& section : node.connections)
            for (const auto id : section)
                if (ids.count(id) == 0) throw std::runtime_error("unknown graph connection ID");
    return nodes;
}

}  // namespace graph_file

#endif
