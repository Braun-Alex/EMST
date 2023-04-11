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

std::shared_ptr<Edge> addEdge(std::vector<std::shared_ptr<Edge>>& edges, const Point& startPoint,
             const Point& endPoint) {
    std::shared_ptr<Edge> forwardEdge = std::make_shared<Edge>(Edge(startPoint, endPoint)),
            backEdge = std::make_shared<Edge>(Edge(endPoint, startPoint));
    forwardEdge->rev = backEdge;
    backEdge->rev = forwardEdge;
    forwardEdge->next = forwardEdge;
    forwardEdge->prev = forwardEdge;
    backEdge->next = backEdge;
    backEdge->prev = backEdge;
    edges.push_back(forwardEdge);
    return forwardEdge;
}

std::vector<std::shared_ptr<Edge>> divideAndConquer(std::vector<std::shared_ptr<Edge>>& edges,
                                                    const std::vector<Point>& points) {
    size_t size = points.size();
    if (size == 2) {
        std::shared_ptr<Edge> edge = addEdge(edges, points[0], points[1]);
        return {edge, edge->rev};
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
    std::vector<std::shared_ptr<Edge>> edges;
}