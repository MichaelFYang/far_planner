// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <unistd.h>
#include "boundary_handler/input_parser.h"
#include <visibility_graph_msg/graph_file_writer.h>

TEST(InputParser, ParsesBoundaryAndTrajectory)
{
    std::istringstream boundary("ply\nformat ascii 1.0\nelement vertex 2\n"
                                "property float x\nend_header\n1 2 3 4\n5 6 7 16777217\n");
    const auto vertices = boundary_handler::ReadAsciiPly(boundary);
    ASSERT_EQ(vertices.size(), 2U);
    EXPECT_FLOAT_EQ(vertices[0].x, 1.0f);
    EXPECT_EQ(vertices[1].polygon_index, 16777217);
    std::istringstream trajectory("0 1 2 0 0 0 0");
    const auto point = boundary_handler::ReadTrajectory(trajectory);
    EXPECT_EQ(point, (std::array<float, 3>{{0.0f, 1.0f, 2.0f}}));
}

TEST(InputParser, RejectsMissingOrMalformedHeader)
{
    for (const char* content : {
             "ply\nformat ascii 1.0\nend_header\n1 2 3 4\n",
             "ply\nformat ascii 1.0\nelement vertex -1\nend_header\n",
             "ply\nformat binary_little_endian 1.0\nelement vertex 1\nend_header\n",
             "ply\nformat ascii 1.0\nelement vertex 1\n1 2 3 4\n",
             "ply\nformat ascii 1.0\nelement vertex 1000001\nend_header\n"}) {
        std::istringstream input(content);
        EXPECT_THROW(boundary_handler::ReadAsciiPly(input), std::runtime_error);
    }
}

TEST(InputParser, RejectsTruncatedAndNonfiniteCoordinates)
{
    for (const char* row : {"1 2 3", "1 2 nan 4", "1 inf 3 4", "1 2 3 2147483648",
                            "1 2 3 1.5", "1 2 3 -1.5"}) {
        std::istringstream input(std::string("ply\nformat ascii 1.0\nelement vertex 1\nend_header\n") + row);
        EXPECT_THROW(boundary_handler::ReadAsciiPly(input), std::runtime_error);
    }
    std::istringstream long_header(std::string("ply\nformat ascii 1.0\n") +
        std::string(5000, 'X') + "\nelement vertex 1\nend_header\n1 2 3 4\n");
    EXPECT_THROW(boundary_handler::ReadAsciiPly(long_header), std::runtime_error);
    for (const char* content : {"1", "1 2", "1 2 nan", "1 inf 3"}) {
        std::istringstream input(content);
        EXPECT_THROW(boundary_handler::ReadTrajectory(input), std::runtime_error);
    }
}

TEST(GraphFileWriter, PreservesBoundaryVertexFloatPrecision)
{
    struct Point { float x, y, z; };
    struct Node {
        std::size_t id = 7;
        int free_direct = 1;
        Point position{1.0f, 2.0f, 1.23456789f};
        std::pair<Point, Point> surf_dirs{{0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
        bool is_covered = true, is_frontier = false, is_navpoint = false, is_boundary = true;
        std::vector<std::size_t> connect_idxs{7}, poly_idxs, contour_idxs, traj_idxs;
    };
    char directory_template[] = "/tmp/far-boundary-writer-XXXXXX";
    const char* directory = ::mkdtemp(directory_template);
    ASSERT_NE(directory, nullptr);
    const std::string path = std::string(directory) + "/boundary_graph.vgh";
    const auto node = std::make_shared<Node>();
    graph_file::WriteGraphFile(path, std::vector<std::shared_ptr<Node>>{node});
    std::ifstream input(path);
    ASSERT_TRUE(input.good());
    std::string id, type, x, y, z;
    input >> id >> type >> x >> y >> z;
    EXPECT_EQ(id, "7");
    EXPECT_EQ(std::stof(z), node->position.z);
    input.close();
    std::remove(path.c_str());
    ::rmdir(directory);
}

#ifdef FAR_PLANNER_ROS1
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#endif
