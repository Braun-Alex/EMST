#include "edge.h"

Edge::Edge(const Point& enteredA, const Point& enteredB) noexcept: a(enteredA), b(enteredB) {}

bool Edge::contains(const Point& point) const noexcept {
    return a == point || b == point;
}

bool Edge::operator==(const Edge& otherEdge) const noexcept {
    return (a == otherEdge.a && b == otherEdge.b) || (a == otherEdge.b && b == otherEdge.a);
}