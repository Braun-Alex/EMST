#pragma once

#include <memory>
#include "point.h"

class Edge: public std::enable_shared_from_this<Edge> {
public:
    Edge() noexcept;
    Edge(const Point& a, const Point& b) noexcept;
    bool operator==(const std::shared_ptr<Edge>& otherEdge) const noexcept;
    void connect(const std::shared_ptr<Edge>& otherEdge) noexcept;
    Point start, end;
    std::shared_ptr<Edge> rev, prev, next;
};