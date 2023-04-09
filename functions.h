#pragma once

#include "point.h"
#include "edge.h"
#include <vector>

std::pair<std::vector<Point>, std::vector<Edge>> triangulateDelaunay(const std::vector<Point>& points);
std::pair<std::vector<Point>, std::vector<Edge>> computeEMST(const std::pair<std::vector<Point>,
        const std::vector<Edge>>& pointsAndEdges);