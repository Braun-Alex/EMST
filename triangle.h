#pragma once

#include "point.h"
#include "functions.h"
#define UNDEFINED 0.001

class Triangle {
public:
    Triangle(const Point& enteredA, const Point& enteredB, const Point& enteredC) noexcept;
    bool circumcircleContains(const Point& point) const noexcept;
    bool sharesEdgeWith(const Triangle &otherTriangle) const noexcept;
    double circumcircleRadius = 0;
    const Point a, b, c;
    Point circumcircleCenter;
};