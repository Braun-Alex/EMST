#include "functions.h"

double distance(const Point& a, const Point& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

std::pair<std::vector<Point>, std::vector<Edge>> triangulateDelaunay(const std::vector<Point>& points) {
}