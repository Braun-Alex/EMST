#pragma once

class Point {
public:
    Point(double enteredX, double enteredY) noexcept;
    bool operator==(const Point& otherPoint) const noexcept;
    double x, y;
};