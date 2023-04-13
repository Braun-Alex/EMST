#include "tests.h"
#include <stdexcept>

void DelaunayTriangulationFixture::SetUp() {
    std::pair<std::vector<Point>, std::vector<std::pair<Point, Point>>> params = GetParam();
    points = params.first;
    expectedResult = params.second;
}

TEST(DelaunayTriangulation, NotValidTriangulation) {
    EXPECT_THROW(triangulateDelaunay(std::vector<Point> {Point(3, 3)}),
                 std::invalid_argument);
}

TEST_P(DelaunayTriangulationFixture, TriangulationOnTwoPoints) {
    std::vector<std::pair<Point, Point>> actualResult = triangulateDelaunay(points);
    EXPECT_EQ(actualResult, expectedResult);
}

INSTANTIATE_TEST_SUITE_P(Default, DelaunayTriangulationFixture, testing::Values(
            std::make_pair(
                    std::vector<Point> {
                        Point(3, 5), Point(5, 3)
                    },
                    std::vector<std::pair<Point, Point>> {
                        std::make_pair(Point(3, 5), Point(5, 3))
                    }),
            std::make_pair(
                    std::vector<Point> {
                            Point(5, 3), Point(3, 5)
                    },
                    std::vector<std::pair<Point, Point>> {
                            std::make_pair(Point(3, 5), Point(5, 3))
                    })
        ));