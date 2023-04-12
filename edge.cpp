#include "edge.h"

Edge::Edge() noexcept {}

Edge::Edge(const Point& startPoint, const Point& endPoint) noexcept: start(startPoint),
end(endPoint) {}

bool Edge::operator==(const std::shared_ptr<Edge>& otherEdge) const noexcept {
    return start == otherEdge->start && end == otherEdge->end;
}

void Edge::connect(const std::shared_ptr<Edge>& otherEdge) noexcept {
    if (start == otherEdge->start && end == otherEdge->end) {
        return;
    }
    otherEdge->next->prev = shared_from_this();
    this->next->prev = otherEdge;
    this->next.swap(otherEdge->next);
}