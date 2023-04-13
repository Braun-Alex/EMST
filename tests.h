#pragma once

#include "functions.h"
#include <gtest/gtest.h>
#include <gmock/gmock-more-matchers.h>

class DelaunayTriangulationOnTwoPointsFixture: public testing::TestWithParam<std::pair<std::vector<Point>,
        std::vector<std::pair<Point, Point>>>> {
protected:
    void SetUp() override;
    std::vector<Point> points;
    std::vector<std::pair<Point, Point>> expectedResult;
};
