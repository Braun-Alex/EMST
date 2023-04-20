#include "functions.h"

double getWeight(const std::pair<Point, Point>& edge) {
    return std::sqrt((edge.second.x - edge.first.x) * (edge.second.x - edge.first.x) +
           (edge.second.y - edge.first.y) * (edge.second.y - edge.first.y));
}

std::vector<double> getWeights(const std::vector<std::pair<Point, Point>>& edges) {
    std::vector<double> weights;
    weights.reserve(edges.size());
    for (const auto& edge: edges) {
        weights.push_back(getWeight(edge));
    }
    return weights;
}

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
    std::shared_ptr<Edge> forwardEdge = std::make_shared<Edge>(startPoint, endPoint),
            backEdge = std::make_shared<Edge>(endPoint, startPoint);
    forwardEdge->rev = backEdge;
    backEdge->rev = forwardEdge;
    forwardEdge->next = forwardEdge;
    forwardEdge->prev = forwardEdge;
    backEdge->next = backEdge;
    backEdge->prev = backEdge;
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
        return std::make_pair(edge, edge->rev);
    }
    if (size == 3) {
        std::shared_ptr<Edge> firstEdge = addEdge(edges, points[0], points[1]),
                              secondEdge = addEdge(edges, points[1], points[2]);
        firstEdge->rev->connect(secondEdge);
        if (isLeft(firstEdge, points[2])) {
            std::shared_ptr<Edge> thirstEdge = connectEdges(edges, secondEdge, firstEdge);
            return std::make_pair(thirstEdge->rev, thirstEdge);
        } else if (isRight(firstEdge, points[2])) {
            connectEdges(edges, secondEdge, firstEdge);
            return std::make_pair(firstEdge, secondEdge->rev);
        } else {
            return std::make_pair(firstEdge, secondEdge->rev);
        }
    }
    size_t middle = size/2;
    std::vector<Point> leftPoints(points.begin(), points.begin() + middle),
                       rightPoints(points.begin() + middle, points.end());
    auto [leftOutside, leftInside] = divideAndConquer(edges, leftPoints);
    auto [rightInside, rightOutside] = divideAndConquer(edges, rightPoints);
    while (true) {
        if (isLeft(rightInside, leftInside->start)) {
            rightInside = rightInside->rev->prev;
        } else if (isRight(leftInside, rightInside->start)) {
            leftInside = leftInside->rev->next;
        } else {
            break;
        }
    }
    std::shared_ptr<Edge> baseEdge = connectEdges(edges, rightInside, leftInside->rev);
    if (leftInside->start == leftOutside->start) {
        leftOutside = baseEdge;
    }
    if (rightInside->start == rightOutside->start) {
        rightOutside = baseEdge->rev;
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
            while (circleContains(baseEdge->end, baseEdge->start,
                                  leftCheckingEdge->end,
                                  leftCheckingEdge->prev->end) &&
                                  isRight(baseEdge, leftCheckingEdge->prev->end)) {
                std::shared_ptr<Edge> temporaryEdge = leftCheckingEdge->prev;
                removeEdge(edges, leftCheckingEdge);
                leftCheckingEdge = temporaryEdge;
            }
        }
        if (isRightEdgeValid) {
            while (circleContains(baseEdge->end, baseEdge->start,
                                 rightCheckingEdge->end,
                                 rightCheckingEdge->next->end) &&
                                 isRight(baseEdge, rightCheckingEdge->next->end)) {
                std::shared_ptr<Edge> temporaryEdge = rightCheckingEdge->next;
                removeEdge(edges, rightCheckingEdge);
                rightCheckingEdge = temporaryEdge;
            }
        }
        if ((circleContains(rightCheckingEdge->end, rightCheckingEdge->start,
                            leftCheckingEdge->start, leftCheckingEdge->end) &&
                            isLeftEdgeValid) || !isRightEdgeValid) {
            baseEdge = connectEdges(edges, leftCheckingEdge, baseEdge->rev);
        } else {
            baseEdge = connectEdges(edges, baseEdge->rev, rightCheckingEdge->rev);
        }
    }
    return std::make_pair(leftOutside, rightOutside);
}

std::vector<std::pair<Point, Point>> triangulateDelaunay(const std::vector<Point>& points) {
    std::vector<Point> uniqueAndSortedPoints = points;
    std::sort(uniqueAndSortedPoints.begin(), uniqueAndSortedPoints.end());
    uniqueAndSortedPoints.erase(std::unique(uniqueAndSortedPoints.begin(),
                                            uniqueAndSortedPoints.end()),
                                uniqueAndSortedPoints.end());
    size_t size = uniqueAndSortedPoints.size();
    if (size < 2) {
        throw std::invalid_argument("Count of points must be greater or equal than two points");
    }
    std::unordered_set<std::shared_ptr<Edge>> edges;
    divideAndConquer(edges, points);
    std::vector<std::pair<Point, Point>> delaunayTriangulation;
    delaunayTriangulation.reserve(edges.size());
    for (const auto &edge: edges) {
        delaunayTriangulation.emplace_back(std::make_pair(edge->start, edge->end));
    }
    return delaunayTriangulation;
}

Point findRoot(std::unordered_map<Point, Point>& treeRoots, const Point& vertex) {
    if (!treeRoots.contains(vertex)) {
        return vertex;
    }
    return treeRoots[vertex] = findRoot(treeRoots, treeRoots[vertex]);
}

void spliceRoots(std::unordered_map<Point, Point>& treeRoots, Point& u, Point& v) {
    u = findRoot(treeRoots, u);
    v = findRoot(treeRoots, v);
    if (u != v) {
        treeRoots[u] = v;
    }
}

bool isAcyclic(std::unordered_map<Point, Point>& treeRoots, const Point& u, const Point& v) {
    return findRoot(treeRoots, u) != findRoot(treeRoots, v);
}

std::vector<std::tuple<Point, Point, double>>
computeEMST(const std::vector<std::pair<Point, Point>>& edgesWithoutWeight) {
    std::vector<std::tuple<Point, Point, double>> edgesWithWeight;
    edgesWithWeight.reserve(edgesWithoutWeight.size());
    for (const auto& edgeWithoutWeight: edgesWithoutWeight) {
        edgesWithWeight.emplace_back(std::make_tuple(edgeWithoutWeight.first, edgeWithoutWeight.second,
                                  getWeight(edgeWithoutWeight)));
    }
    std::sort(edgesWithWeight.begin(), edgesWithWeight.end(),
              [](const std::tuple<Point, Point, double>& firstEdgeWithWeight,
                 const std::tuple<Point, Point, double>& secondEdgeWithWeight) {
        return get<2>(firstEdgeWithWeight) < get<2>(secondEdgeWithWeight);
    });
    std::vector<std::tuple<Point, Point, double>> euclideanMinimumSpanningTree(edgesWithWeight.size());
    std::unordered_map<Point, Point> roots;
    for (const auto& edgeWithWeight: edgesWithWeight) {
        Point u = get<0>(edgeWithWeight), v = get<1>(edgeWithWeight);
        if (isAcyclic(roots, u, v)) {
            euclideanMinimumSpanningTree.push_back(edgeWithWeight);
            spliceRoots(roots, u, v);
        }
    }
    euclideanMinimumSpanningTree.erase(std::remove_if(euclideanMinimumSpanningTree.begin(),
                                                      euclideanMinimumSpanningTree.end(),
                                                      [](std::tuple<Point, Point, double>& edge) {
        return get<2>(edge) == 0;
    }), euclideanMinimumSpanningTree.end());
    return euclideanMinimumSpanningTree;
}