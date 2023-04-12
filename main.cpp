#include "functions.h"
#include <iostream>

int main() {
    std::vector<Point> points;
    std::random_device randomDevice;
    std::mt19937 generator(randomDevice());
    std::uniform_int_distribution<int> distribution(1000, 10000);
    int randomSize = distribution(generator);
    points.reserve(randomSize);
    for (int i = 0; i < randomSize; i++) {
        points.emplace_back(Point(distribution(generator), distribution(generator)));
    }
    std::vector<std::pair<Point, Point>> result = triangulateDelaunay(points);
    for (const auto& [firstPoint, secondPoint]: result) {
        std::cout << "First point: (" << firstPoint.x << ", " << firstPoint.y << "), "
                  << "second point: (" << secondPoint.x << ", " << secondPoint.y << ")" << std::endl;
    }
    return 0;
}