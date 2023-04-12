#include "functions.h"
#include <iostream>

int main() {
    std::vector<Point> points;
    std::random_device randomDevice;
    std::mt19937 generator(randomDevice());
    std::uniform_int_distribution<int> distribution(-1000000, 1000000);
    int randomSize = 10000;
    points.reserve(randomSize);
    for (int i = 0; i < randomSize; i++) {
        points.emplace_back(Point(distribution(generator), distribution(generator)));
    }
    std::vector<std::pair<Point, Point>> delaunayTriangulation = triangulateDelaunay(points);
    std::vector<std::tuple<Point, Point, double>> euclideanMinimumSpanningTree =
            computeEMST(delaunayTriangulation);
    for (const auto& [firstPoint, secondPoint, weight]: euclideanMinimumSpanningTree) {
        std::cout << "First point: (" << firstPoint.x << ", " << firstPoint.y << "), "
                  << "second point: (" << secondPoint.x << ", " << secondPoint.y << "), "
                  << "weight: " << weight << std::endl;
    }
    return 0;
}