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
    Eigen::Vector<double, 2> vectorA(a.x - point.x, a.y - point.y),
            vectorB(b.x - point.x, b.y - point.y),
            vectorC(c.x - point.x, c.y - point.y);
    Eigen::Matrix<double, 3, 3> matrix;
    matrix << vectorA, vectorA.squaredNorm(),
            vectorB, vectorB.squaredNorm(),
            vectorC, vectorC.squaredNorm();
    return matrix.determinant() < 0;
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

std::unordered_set<std::shared_ptr<Edge>> divideAndConquer(
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
    std::unordered_set<std::shared_ptr<Edge>> edges = divideAndConquer(edges, points);
    std::vector<std::pair<Point, Point>> triangulatedPoints;
    triangulatedPoints.reserve(edges.size());
    for (const auto &edge: edges) {
        triangulatedPoints.emplace_back(std::make_pair(edge->start, edge->end));
    }
    return triangulatedPoints;
}