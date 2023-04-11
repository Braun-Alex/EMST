#pragma once

#include "point.h"
#include "edge.h"
#include "triangle.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

double distance(const Point& a, const Point& b);
double locate(const std::shared_ptr<Edge>& edge, const Point& point);
bool isLeft(const std::shared_ptr<Edge>& edge, const Point& point);
bool isRight(const std::shared_ptr<Edge>& edge, const Point& point);
std::pair<std::vector<Point>, std::vector<Edge>> triangulateDelaunay(const std::vector<Point>& points);
std::pair<std::vector<Point>, std::vector<Edge>> computeEMST(const std::pair<std::vector<Point>,
        const std::vector<Edge>>& pointsAndEdges);