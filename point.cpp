#include "point.h"

Point::Point() noexcept {}

Point::Point(double enteredX, double enteredY) noexcept: x(enteredX), y(enteredY) {}

bool Point::operator==(const Point& otherPoint) const noexcept {
    return x == otherPoint.x && y == otherPoint.y;
}