#pragma once

#include "point.h"
#include "edge.h"
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <Eigen/Dense>

double locate(const std::shared_ptr<Edge>& edge, const Point& point);
bool isLeft(const std::shared_ptr<Edge>& edge, const Point& point);
bool isRight(const std::shared_ptr<Edge>& edge, const Point& point);
bool circleContains(const Point& a, const Point& b, const Point& c, const Point& point);
std::shared_ptr<Edge> addEdge(std::unordered_set<std::shared_ptr<Edge>>& edges,
                              const Point& startPoint,
                              const Point& endPoint);
std::shared_ptr<Edge> connectEdges(std::unordered_set<std::shared_ptr<Edge>>& edges,
                                 const std::shared_ptr<Edge>& firstEdge,
                                 const std::shared_ptr<Edge>& secondEdge);
void removeEdge(std::unordered_set<std::shared_ptr<Edge>>& edges,
                const std::shared_ptr<Edge>& edge);
std::unordered_set<std::shared_ptr<Edge>> divideAndConquer(
        std::unordered_set<std::shared_ptr<Edge>>& edges,
        const std::vector<Point>& points);
std::vector<std::pair<Point, Point>> triangulateDelaunay(const std::vector<Point>& points);
std::pair<std::vector<Point>, std::vector<Edge>> computeEMST(const std::pair<std::vector<Point>,
        const std::vector<Edge>>& pointsAndEdges);