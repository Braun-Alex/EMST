#include "functions.h"

double locate(const std::shared_ptr<Edge>& edge, const Point &point) {
    Eigen::Matrix<double, 2, 2> matrix;
    matrix << edge->start.x - point.x, edge->start.y - point.y,
            edge->end.x - point.x, edge->end.y - point.y;
    return matrix.determinant();
}

bool isLeft(const std::shared_ptr<Edge>& edge, const Point &point) {
    return locate(edge, point) < 0;
}

bool isRight(const std::shared_ptr<Edge>& edge, const Point& point) {
    return locate(edge, point) > 0;
}

bool circleContains(const Point& a, const Point& b, const Point& c, const Point& point) {
    Eigen::Matrix<double, 3, 3> matrix;
    matrix << a.x - point.x, a.y - point.y, (a.x - point.x) * (a.x - point.x) +
                                            (a.y - point.y) * (a.y - point.y),
              b.x - point.x, b.y - point.y, (b.x - point.x) * (b.x - point.x) +
                                            (b.y - point.y) * (b.y - point.y),
              c.x - point.x, c.y - point.y, (c.x - point.x) * (c.x - point.x) +
                                            (c.y - point.y) * (c.y - point.y);
    return matrix.determinant() >= 0;
}

std::shared_ptr<Edge> addEdge(std::unordered_set<std::shared_ptr<Edge>>& edges, const Point& startPoint,
                              const Point& endPoint) {
    std::shared_ptr<Edge> forwardEdge = std::make_shared<Edge>(Edge(startPoint, endPoint)),
            backEdge = std::make_shared<Edge>(Edge(endPoint, startPoint));
    forwardEdge->rev = backEdge;
    backEdge->rev = forwardEdge;
    forwardEdge->next = forwardEdge;
    forwardEdge->prev = forwardEdge;
    backEdge->next = backEdge;
    backEdge->prev = backEdge;
    std::unordered_set<std::shared_ptr<Edge>> compilerNotice;
    edges.insert(forwardEdge);
    return forwardEdge;
}

std::shared_ptr<Edge> connectEdges(std::unordered_set<std::shared_ptr<Edge>>& edges,
                                 const std::shared_ptr<Edge>& firstEdge,
                                 const std::shared_ptr<Edge>& secondEdge) {
    std::shared_ptr<Edge> mergedEdge = addEdge(edges, firstEdge->end, secondEdge->start);
    mergedEdge->connect(firstEdge->rev->prev);
    mergedEdge->rev->connect(secondEdge);
    return mergedEdge;
}

void removeEdge(std::unordered_set<std::shared_ptr<Edge>>& edges,
                const std::shared_ptr<Edge>& edge) {
    edge->connect(edge->prev);
    edge->rev->connect(edge->rev->prev);
    edges.erase(edge);
    edges.erase(edge->rev);
}

std::pair<std::shared_ptr<Edge>, std::shared_ptr<Edge>> divideAndConquer(
        std::unordered_set<std::shared_ptr<Edge>>& edges,
        const std::vector<Point>& points) {
    size_t size = points.size();
    if (size == 2) {
        std::shared_ptr<Edge> edge = addEdge(edges, points[0], points[1]);
        return {edge, edge->rev};
    }
    if (size == 3) {
        std::shared_ptr<Edge> firstEdge = addEdge(edges, points[0], points[1]),
                              secondEdge = addEdge(edges, points[1], points[2]);
        firstEdge->rev->connect(secondEdge);
        if (isLeft(firstEdge, points[2])) {
            std::shared_ptr<Edge> thirstEdge = connectEdges(edges, secondEdge, firstEdge);
            return {thirstEdge->rev, thirstEdge};
        } else if (isRight(firstEdge, points[2])) {
            connectEdges(edges, secondEdge, firstEdge);
            return {firstEdge, secondEdge->rev};
        } else {
            return {firstEdge, secondEdge->rev};
        }
    }
    size_t middle = size/2;
    std::vector<Point> leftPoints(points.begin(), points.begin() + middle),
                       rightPoints(points.begin() + middle, points.end());
    std::pair<std::shared_ptr<Edge>, std::shared_ptr<Edge>>
    leftEdge = divideAndConquer(edges, leftPoints),
    rightEdge = divideAndConquer(edges, rightPoints);
    while (true) {
        if (isLeft(rightEdge.first, leftEdge.second->start)) {
            rightEdge.first = rightEdge.first->rev->prev;
        } else if (isRight(leftEdge.second, rightEdge.first->start)) {
            leftEdge.second = leftEdge.second->rev->next;
        } else {
            break;
        }
    }
    std::shared_ptr<Edge> baseEdge = connectEdges(edges, rightEdge.first, leftEdge.first->rev);
    if (leftEdge.first->start == leftEdge.second->start) {
        leftEdge.second = baseEdge;
    }
    if (rightEdge.first->start == rightEdge.second->start) {
        rightEdge.second = baseEdge->rev;
    }
    while (true) {
        std::shared_ptr<Edge> leftCheckingEdge = baseEdge->prev,
                              rightCheckingEdge = baseEdge->rev->next;
        bool isLeftEdgeValid = isRight(baseEdge, leftCheckingEdge->end),
             isRightEdgeValid = isRight(baseEdge, rightCheckingEdge->end);
        if (!isLeftEdgeValid && !isRightEdgeValid) {
            break;
        }
        if (isLeftEdgeValid) {
            while (circleContains(baseEdge->start, baseEdge->end,
                                  leftCheckingEdge->end,
                                  leftCheckingEdge->prev->end) &&
                                  isRight(baseEdge, leftCheckingEdge->prev->end)) {
                std::shared_ptr<Edge> temporaryEdge = leftCheckingEdge->prev;
                removeEdge(edges, leftCheckingEdge);
                leftCheckingEdge = temporaryEdge;
            }
        }
        if (isRightEdgeValid) {
            while (circleContains(baseEdge->start, baseEdge->end,
                                 rightCheckingEdge->end,
                                 rightCheckingEdge->next->end) &&
                                 isRight(baseEdge, rightCheckingEdge->next->end)) {
                std::shared_ptr<Edge> temporaryEdge = rightCheckingEdge->next;
                removeEdge(edges, rightCheckingEdge);
                rightCheckingEdge = temporaryEdge;
            }
        }
        if ((circleContains(rightCheckingEdge->start, rightCheckingEdge->end,
                            leftCheckingEdge->start, leftCheckingEdge->end) &&
                            isLeftEdgeValid) || isRightEdgeValid) {
            baseEdge = connectEdges(edges, leftCheckingEdge, baseEdge->rev);
        } else {
            baseEdge = connectEdges(edges, baseEdge->rev, rightCheckingEdge->rev);
        }
    }
    return std::make_pair(leftEdge.first, rightEdge.second);
}

std::vector<std::pair<Point, Point>> triangulateDelaunay(const std::vector<Point>& points) {
    size_t size = points.size();
    if (size < 2) {
        return {};
    }
    std::vector<Point> uniqueAndSortedPoints = points;
    std::sort(uniqueAndSortedPoints.begin(), uniqueAndSortedPoints.end());
    uniqueAndSortedPoints.erase(std::unique(uniqueAndSortedPoints.begin(),
                                            uniqueAndSortedPoints.end()),
                                uniqueAndSortedPoints.end());
    std::unordered_set<std::shared_ptr<Edge>> edges;
    divideAndConquer(edges, points);
    std::vector<std::pair<Point, Point>> triangulatedPoints;
    triangulatedPoints.reserve(edges.size());
    for (const auto &edge: edges) {
        triangulatedPoints.emplace_back(std::make_pair(edge->start, edge->end));
    }
    return triangulatedPoints;
}

std::vector<std::tuple<Point, Point, double>>
computeEMST(const std::vector<std::pair<Point, Point>>& edgesWithoutWeight) {
    std::vector<std::tuple<Point, Point, double>> edgesWithWeight;
    edgesWithWeight.reserve(edgesWithoutWeight.size());
    for (const auto& edgeWithoutWeight: edgesWithoutWeight) {
        edgesWithWeight.emplace_back(std::make_tuple(edgeWithoutWeight.first, edgeWithoutWeight.second,
                                  (edgeWithoutWeight.first.x - edgeWithoutWeight.first.y) *
                                  (edgeWithoutWeight.first.x - edgeWithoutWeight.first.y) +
                                  (edgeWithoutWeight.second.x - edgeWithoutWeight.second.y) *
                                  (edgeWithoutWeight.second.x - edgeWithoutWeight.second.y)));
    }
    std::sort(edgesWithWeight.begin(), edgesWithWeight.end(),
              [](const std::tuple<Point, Point, double>& firstEdgeWithWeight,
                 const std::tuple<Point, Point, double>& secondEdgeWithWeight) {
        return get<2>(firstEdgeWithWeight) < get<2>(secondEdgeWithWeight);
    });
}