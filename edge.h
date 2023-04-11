#pragma once

#include <memory>
#include "point.h"

class Edge {
public:
    Edge() noexcept;
    Edge(const Point& a, const Point& b) noexcept;
    bool operator==(std::unique_ptr<Edge> otherEdge) const noexcept;
    Point start, end;
    std::unique_ptr<Edge> rev, prev, next;
};