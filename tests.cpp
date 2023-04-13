#include "tests.h"
#include <stdexcept>

void DelaunayTriangulationOnTwoPointsFixture::SetUp() {
    std::pair<std::vector<Point>, std::vector<std::pair<Point, Point>>> params = GetParam();
    points = params.first;
    expectedResult = params.second;
}

void DelaunayTriangulationFixture::SetUp() {
    std::pair<std::vector<Point>, std::vector<std::pair<Point, Point>>> params = GetParam();
    points = params.first;
    expectedResult = params.second;
    expectedWeights = getWeights(expectedResult);
    std::sort(expectedWeights.begin(), expectedWeights.end());
}

void EuclideanMinimumSpanningTreeFixture::SetUp() {
    std::pair<std::vector<std::pair<Point, Point>>, std::vector<double>> params = GetParam();
    edges = params.first;
    expectedResult = params.second;
    std::vector<std::tuple<Point, Point, double>> euclideanMST = computeEMST(edges);
    actualResult.reserve(euclideanMST.size());
    for (const auto& edge: euclideanMST) {
        actualResult.push_back(get<2>(edge));
    }
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

TEST_P(DelaunayTriangulationFixture, TriangulationOnThreePoints) {
    std::vector<std::pair<Point, Point>> actualResult = triangulateDelaunay(points);
    std::vector<double> actualWeights = getWeights(actualResult);
    std::sort(actualWeights.begin(), actualWeights.end());
    EXPECT_EQ(actualWeights, expectedWeights);
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

INSTANTIATE_TEST_SUITE_P(Default, DelaunayTriangulationFixture, testing::Values(
        std::make_pair(
                std::vector<Point> {
                        Point(5, 3), Point(3, 5), Point(9, 5)
                },
                std::vector<std::pair<Point, Point>> {
                        std::make_pair(Point(3, 5), Point(5, 3)),
                        std::make_pair(Point(9, 5), Point(5, 3)),
                        std::make_pair(Point(3, 5), Point(9, 5))
                }),
        std::make_pair(
                std::vector<Point> {
                        Point(-3, -3), Point(0, 0), Point(3, 3)
                },
                std::vector<std::pair<Point, Point>> {
                        std::make_pair(Point(-3, -3), Point(0, 0)),
                        std::make_pair(Point(0, 0), Point(3, 3))
                }),
        std::make_pair(
                std::vector<Point> {
                        Point(1, 1), Point(2, 2), Point(3, 3)
                },
                std::vector<std::pair<Point, Point>> {
                        std::make_pair(Point(1, 1), Point(2, 2)),
                        std::make_pair(Point(2, 2), Point(3, 3))
                })
        ));

TEST_P(EuclideanMinimumSpanningTreeFixture, EMST) {
    EXPECT_EQ(actualResult, expectedResult);
}

INSTANTIATE_TEST_SUITE_P(Default, EuclideanMinimumSpanningTreeFixture, testing::Values(
        std::make_pair(
                triangulateDelaunay(std::vector<Point> {
                        Point(5, 3), Point(3, 5), Point(9, 5)
                }),
                std::vector<double> {
                    std::sqrt(8), std::sqrt(20)
                })
        ));