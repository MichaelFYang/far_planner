#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

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

#ifdef FAR_PLANNER_ROS1
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#endif
