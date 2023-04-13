#include "tests.h"
#include <stdexcept>

void DelaunayTriangulationOnTwoPointsFixture::SetUp() {
    std::pair<std::vector<Point>, std::vector<std::pair<Point, Point>>> params = GetParam();
    points = params.first;
    expectedResult = params.second;
}

TEST(DelaunayTriangulation, NotValidTriangulation) {
    EXPECT_THAT(triangulateDelaunay(std::vector<Point> {}),
                testing::IsEmpty());
    EXPECT_THAT(triangulateDelaunay(std::vector<Point> {Point(3, 3)}),
                testing::IsEmpty());
    EXPECT_THAT(triangulateDelaunay(std::vector<Point> {Point(3, 3), Point(3, 3)}),
                testing::IsEmpty());
    EXPECT_THAT(triangulateDelaunay(std::vector<Point> {Point(3, 3), Point(3, 3), Point(3, 3)}),
                testing::IsEmpty());
    EXPECT_THAT(triangulateDelaunay(std::vector<Point> (100, Point(3, 3))),
                testing::IsEmpty());
}

TEST_P(DelaunayTriangulationOnTwoPointsFixture, TriangulationOnTwoPoints) {
    std::vector<std::pair<Point, Point>> actualResult = triangulateDelaunay(points);
    EXPECT_THAT(actualResult, testing::AnyOf(testing::Contains(
            testing::Pair(expectedResult.back().first,
                          expectedResult.back().second)),
                              testing::Contains(
            testing::Pair(expectedResult.back().second,
                          expectedResult.back().first))));
}

INSTANTIATE_TEST_SUITE_P(Default, DelaunayTriangulationOnTwoPointsFixture, testing::Values(
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
                    }),
            std::make_pair(
                    std::vector<Point> {
                            Point(-3, -3), Point(3, 3)
                    },
                    std::vector<std::pair<Point, Point>> {
                            std::make_pair(Point(-3, -3), Point(3, 3))
                    }),
            std::make_pair(
                    std::vector<Point> {
                            Point(0.5, 0.3), Point(0.3, 0.5)
                    },
                    std::vector<std::pair<Point, Point>> {
                            std::make_pair(Point(0.5, 0.3), Point(0.3, 0.5))
                    }),
            std::make_pair(
                    std::vector<Point> {
                            Point(0, 0), Point(0, 3)
                    },
                    std::vector<std::pair<Point, Point>> {
                            std::make_pair(Point(0, 0), Point(0, 3))
                    }),
            std::make_pair(
                    std::vector<Point> {
                            Point(0, 0), Point(0, 0), Point(0, 0),
                            Point(5, 33), Point(5, 33), Point(5, 33)
                    },
                    std::vector<std::pair<Point, Point>> {
                            std::make_pair(Point(0, 0), Point(5, 33))
                    }),
            std::make_pair(
                    std::vector<Point> {
                            Point(5, 33), Point(5, 33), Point(5, 33),
                            Point(0, 0), Point(0, 0), Point(0, 0)
                    },
                    std::vector<std::pair<Point, Point>> {
                            std::make_pair(Point(0, 0), Point(5, 33))
                    }),
            std::make_pair(
                    std::vector<Point> {
                            Point(33, -33), Point(-33, 33)
                    },
                    std::vector<std::pair<Point, Point>> {
                            std::make_pair(Point(33, -33), Point(-33, 33))
                    }),
            std::make_pair(
                    std::vector<Point> {
                            Point(-33, -33), Point(33, 33)
                    },
                    std::vector<std::pair<Point, Point>> {
                            std::make_pair(Point(-33, -33), Point(33, 33))
                    })
        ));