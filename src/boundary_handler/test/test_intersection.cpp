// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include "boundary_handler/intersection.h"

TEST(Intersection, CrossingAndSeparatedSegments) {
    EXPECT_TRUE(POLYOPS::doIntersect({0, 0}, {2, 2}, {0, 2}, {2, 0}));
    EXPECT_FALSE(POLYOPS::doIntersect({0, 0}, {1, 0}, {0, 1}, {1, 1}));
}

TEST(Intersection, EndpointsAndCollinearSegments) {
    EXPECT_TRUE(POLYOPS::doIntersect({0, 0}, {2, 0}, {2, 0}, {3, 1}));
    EXPECT_TRUE(POLYOPS::doIntersect({0, 0}, {3, 0}, {1, 0}, {2, 0}));
    EXPECT_FALSE(POLYOPS::doIntersect({0, 0}, {1, 0}, {2, 0}, {3, 0}));
}
