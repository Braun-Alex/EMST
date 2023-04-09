#pragma once

#include "point.h"
#include "edge.h"
#include "triangle.h"
#include <vector>
#include <cmath>
#include <algorithm>

double distance(const Point& a, const Point& b);
std::pair<std::vector<Point>, std::vector<Edge>> triangulateDelaunay(const std::vector<Point>& points);
std::pair<std::vector<Point>, std::vector<Edge>> computeEMST(const std::pair<std::vector<Point>,
        const std::vector<Edge>>& pointsAndEdges);