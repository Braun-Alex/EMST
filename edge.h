#pragma once

#include "point.h"

class Edge {
public:
    Edge(const Point& a, const Point& b) noexcept;
    bool contains(const Point& point) const noexcept;
    bool operator==(const Edge& otherEdge) const noexcept;
    const Point a, b;
};