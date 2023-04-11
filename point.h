#pragma once

class Point {
public:
    Point() noexcept;
    Point(double enteredX, double enteredY) noexcept;
    bool operator==(const Point& otherPoint) const noexcept;
    bool operator<(const Point& otherPoint) const noexcept;
    double x, y;
};