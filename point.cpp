#include "point.h"

Point::Point() noexcept {}

Point::Point(double enteredX, double enteredY) noexcept: x(enteredX), y(enteredY) {}

bool Point::operator==(const Point& otherPoint) const noexcept {
    return x == otherPoint.x && y == otherPoint.y;
}

bool Point::operator<(const Point& otherPoint) const noexcept {
    return x == otherPoint.x ? y < otherPoint.y : x < otherPoint.x;
}

std::size_t std::hash<Point>::operator()(const Point& point) const {
    std::size_t xHash = std::hash<double>()(point.x),
                yHash = std::hash<double>()(point.y);
    return xHash & (yHash << 1);
}