#include "edge.h"

Edge::Edge() noexcept {}

Edge::Edge(const Point& startPoint, const Point& endPoint) noexcept: start(startPoint),
end(endPoint), rev(std::make_unique<Edge>()), prev(std::make_unique<Edge>()),
next(std::make_unique<Edge>()) {}

bool Edge::operator==(std::unique_ptr<Edge> otherEdge) const noexcept {
    return start == otherEdge->start && end == otherEdge->end;
}