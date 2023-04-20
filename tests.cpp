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
    EXPECT_THROW(triangulateDelaunay(std::vector<Point> {}),
                std::invalid_argument);
    EXPECT_THROW(triangulateDelaunay(std::vector<Point> {Point(3, 3)}),
                std::invalid_argument);
    EXPECT_THROW(triangulateDelaunay(std::vector<Point> {Point(3, 3), Point(3, 3)}),
                std::invalid_argument);
    EXPECT_THROW(triangulateDelaunay(std::vector<Point> {Point(3, 3), Point(3, 3), Point(3, 3)}),
                std::invalid_argument);
    EXPECT_THROW(triangulateDelaunay(std::vector<Point> (100, Point(3, 3))),
                std::invalid_argument);
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

TEST_P(DelaunayTriangulationFixture, TriangulationOnManyPoints) {
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
                }),
        std::make_pair(
                triangulateDelaunay(std::vector<Point> {
                        Point(1, 1), Point(2, 2), Point(3, 3)
                }),
                std::vector<double> {
                        std::sqrt(2), std::sqrt(2)
                })
        ));

TEST(EMSTInDelaunayTriangulation, DelaunayTriangulationMustContainEMST) {
    std::vector<Point> points = {Point(3, 3), Point(3, 33)};
    points.reserve(3);
    std::vector<std::pair<Point, Point>> delaunayTriangulationFirst = triangulateDelaunay(points);
    std::vector<std::tuple<Point, Point, double>> euclideanMSTFirst = computeEMST(
            std::vector<std::pair<Point, Point>> {
                std::make_pair(Point(3, 3), Point(3, 33))
            });
    EXPECT_EQ(delaunayTriangulationFirst.back(), std::make_pair(
            get<0>(euclideanMSTFirst.back()), get<1>(euclideanMSTFirst.back())
            ));
    points.emplace_back(Point(0, 0));
    std::vector<std::pair<Point, Point>> delaunayTriangulationSecond = triangulateDelaunay(points);
    std::vector<std::tuple<Point, Point, double>> euclideanMSTSecond = computeEMST(
            std::vector<std::pair<Point, Point>> {
                    std::make_pair(Point(3, 3), Point(3, 33)),
                    std::make_pair(Point(0, 0), Point(3, 3)),
                    std::make_pair(Point(0, 0), Point(3, 33))
            });
    std::vector<std::pair<Point, Point>> edgesOfEMST = std::vector<std::pair<Point, Point>> {
            std::make_pair(get<0>(euclideanMSTSecond.front()),
                           get<1>(euclideanMSTSecond.front())),
            std::make_pair(get<0>(euclideanMSTSecond.back()),
                           get<1>(euclideanMSTSecond.back()))
    };
    EXPECT_EQ(delaunayTriangulationSecond.size(), 3);
    EXPECT_EQ(edgesOfEMST.size(), 2);
    EXPECT_EQ(edgesOfEMST.front(), std::make_pair(Point(0, 0), Point(3, 3)));
    EXPECT_EQ(edgesOfEMST.back(), std::make_pair(Point(3, 3), Point(3, 33)));
}

TEST(Circle, DoesNotContainPoint) {
    EXPECT_FALSE(circleContains(Point(0, 3), Point(3, 0), Point(3, 6), Point(6, 2)));
}

TEST(Circle, ContainsPoint) {
    EXPECT_TRUE(circleContains(Point(0, 3), Point(3, 0), Point(3, 6), Point(3, 5)));
}