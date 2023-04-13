#pragma once

#include "point.h"
#include "edge.h"
#include <iostream>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <random>
#include <Eigen/Dense>

double getWeight(const std::pair<Point, Point>& edge);
std::vector<double> getWeights(const std::vector<std::pair<Point, Point>>& edges);
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
std::pair<std::shared_ptr<Edge>, std::shared_ptr<Edge>> divideAndConquer(
        std::unordered_set<std::shared_ptr<Edge>>& edges,
        const std::vector<Point>& points);
std::vector<std::pair<Point, Point>> triangulateDelaunay(const std::vector<Point>& points);
Point findRoot(std::unordered_map<Point, Point>& treeRoots, const Point& vertex);
void spliceRoots(std::unordered_map<Point, Point>& treeRoots, Point& u, Point& v);
bool isAcyclic(std::unordered_map<Point, Point>& treeRoots, const Point& u, const Point& v);
std::vector<std::tuple<Point, Point, double>>
computeEMST(const std::vector<std::pair<Point, Point>>& edgesWithoutWeight);