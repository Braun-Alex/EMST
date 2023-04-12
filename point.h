#pragma once

#include <unordered_map>

class Point {
public:
    Point() noexcept;
    Point(double enteredX, double enteredY) noexcept;
    bool operator==(const Point& otherPoint) const noexcept;
    bool operator<(const Point& otherPoint) const noexcept;
    double x, y;
};

namespace std {
    template<>
    struct hash<Point> {
        size_t operator()(const Point& point) const;
    };
}