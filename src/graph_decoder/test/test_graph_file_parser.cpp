// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <sys/stat.h>
#include <unistd.h>

#include "graph_decoder/atomic_file.h"
#include "graph_decoder/graph_file_parser.h"

namespace {
const char* kNode = "4294967295 2 1 2 3 0 1 0 1 0 0 1 0 1 0 | | | 4294967295";
}

TEST(GraphFileParser, KeepsFinalTokenAndFullUint32Id) {
    const auto node = graph_file::ParseLine(kNode);
    EXPECT_EQ(node.id, 4294967295u);
    ASSERT_EQ(node.connections[3].size(), 1u);
    EXPECT_EQ(node.connections[3][0], node.id);
    std::istringstream file(std::string(kNode) + "\n");
    EXPECT_EQ(graph_file::ParseGraph(file).size(), 1u);
}

TEST(GraphFileParser, RejectsMalformedAndNonfiniteFields) {
    EXPECT_THROW(graph_file::ParseLine("1 2 3"), std::exception);
    EXPECT_THROW(graph_file::ParseLine("-1 2 1 2 3 0 1 0 1 0 0 1 0 1 0 | | |"), std::exception);
    EXPECT_THROW(graph_file::ParseLine("4294967296 2 1 2 3 0 1 0 1 0 0 1 0 1 0 | | |"), std::exception);
    EXPECT_THROW(graph_file::ParseLine("1 2 nan 2 3 0 1 0 1 0 0 1 0 1 0 | | |"), std::exception);
    EXPECT_THROW(graph_file::ParseLine("1 2 1 2 3 0 1 0 1 0 0 2 0 1 0 | | |"), std::exception);
    EXPECT_THROW(graph_file::ParseLine("1 2 1 2 3 0 1 0 1 0 0 1 0 1 0 | |"), std::exception);
}

TEST(GraphFileParser, RejectsDuplicateAndUnknownIds) {
    std::istringstream duplicate(std::string(kNode) + "\n" + kNode + "\n");
    EXPECT_THROW(graph_file::ParseGraph(duplicate), std::exception);
    std::istringstream unknown("1 2 1 2 3 0 1 0 1 0 0 1 0 1 0 2 | | |\n");
    EXPECT_THROW(graph_file::ParseGraph(unknown), std::exception);
}

TEST(GraphFileParser, ReadsSavedExamples) {
    std::ifstream indoor(std::string(FAR_GRAPH_DATA_DIR) + "/indoor.vgh");
    std::ifstream boundary(std::string(FAR_GRAPH_DATA_DIR) + "/boundary_graph.vgh");
    ASSERT_TRUE(indoor.good());
    ASSERT_TRUE(boundary.good());
    EXPECT_EQ(graph_file::ParseGraph(indoor).size(), 756u);
    EXPECT_EQ(graph_file::ParseGraph(boundary).size(), 25u);
}

TEST(GraphFileWriter, ReplacesCompleteFilesAndPreservesOldFileOnFailure) {
    char directory_template[] = "/tmp/far-graph-atomic-XXXXXX";
    const char* directory = ::mkdtemp(directory_template);
    ASSERT_NE(directory, nullptr);
    const std::string path = std::string(directory) + "/saved.vgh";
    {
        std::ofstream initial(path);
        initial << "old graph\n";
    }
    ASSERT_EQ(::chmod(path.c_str(), 0640), 0);

    EXPECT_THROW(graph_file::AtomicWrite(path, [](std::FILE* output) {
        std::fputs("partial graph", output);
        throw std::runtime_error("injected write failure");
    }), std::runtime_error);
    {
        std::ifstream unchanged(path);
        std::string line;
        std::getline(unchanged, line);
        EXPECT_EQ(line, "old graph");
    }

    graph_file::AtomicWrite(path, [](std::FILE* output) {
        std::fputs("complete graph\n", output);
    });
    {
        std::ifstream replaced(path);
        std::string line;
        std::getline(replaced, line);
        EXPECT_EQ(line, "complete graph");
    }
    struct stat saved_mode{};
    ASSERT_EQ(::stat(path.c_str(), &saved_mode), 0);
    EXPECT_EQ(saved_mode.st_mode & 0777, 0640);
    DIR* entries = ::opendir(directory);
    ASSERT_NE(entries, nullptr);
    int files = 0;
    while (const dirent* entry = ::readdir(entries)) {
        if (entry->d_name[0] != '.') ++files;
    }
    ::closedir(entries);
    EXPECT_EQ(files, 1);  // No temporary file remains after either path.
    std::remove(path.c_str());
    ::rmdir(directory);
}

#ifdef FAR_PLANNER_ROS1
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#endif
