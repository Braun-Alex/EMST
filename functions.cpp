#include "functions.h"

double distance(const Point& a, const Point& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
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
    Eigen::Vector<double, 2> vectorA(a.x - point.x, a.y - point.y),
                             vectorB(b.x - point.x, b.y - point.y),
                             vectorC(c.x - point.x, c.y - point.y);
    Eigen::Matrix<double, 3, 3> matrix;
    matrix << vectorA, vectorA.squaredNorm(),
              vectorB, vectorB.squaredNorm(),
              vectorC, vectorC.squaredNorm();
    return matrix.determinant() < 0;
}