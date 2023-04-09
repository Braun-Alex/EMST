#include "point.h"

Point::Point() noexcept: x(-1), y(-1), group(-1) {}

void Point::setPoint(int enteredX, int enteredY, int index) noexcept {
    x = enteredX;
    y = enteredY;
    group = index;
}

void Point::setGroup(int index) noexcept {
    group = index;
}

int Point::getX() const noexcept {
    return x;
}

int Point::getY() const noexcept {
    return y;
}

int Point::getGroup() const noexcept {
    return group;
}