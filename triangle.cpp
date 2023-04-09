#include "triangle.h"

Triangle::Triangle(const Point& enteredA, const Point& enteredB, const Point& enteredC)
noexcept: a(enteredA), b(enteredB), c(enteredC),
circumcircleCenter(Point(UNDEFINED, UNDEFINED)) {
    double A = b.x - a.x;
    double B = b.y - a.y;
    double C = c.x - a.x;
    double D = c.y - a.y;
    double E = A * (a.x + b.x) + B * (a.y + b.y);
    double F = C * (a.x + c.x) + D * (a.y + c.y);
    double G = 2 * (A * (c.y - b.y) - B * (c.x - b.x));
    if (G != 0) {
        circumcircleCenter = Point(
                (D * E - B * F) / G,
                (A * F - C * E) / G);
        circumcircleRadius = distance(circumcircleCenter, a);
    }
}

bool Triangle::circumcircleContains(const Point& point) const noexcept {
    if (circumcircleCenter.x == UNDEFINED && circumcircleCenter.y == UNDEFINED) {
        return false;
    }
    return distance(circumcircleCenter, point) <= circumcircleRadius;
}

bool Triangle::sharesEdgeWith(const Triangle& otherTriangle) const noexcept {
    int sharedVertices = 0;
    if (otherTriangle.a == a || otherTriangle.a == b || otherTriangle.a == c) {
        sharedVertices++;
    }
    if (otherTriangle.b == a || otherTriangle.b == b || otherTriangle.b == c) {
        sharedVertices++;
    }
    if (otherTriangle.c == a || otherTriangle.c == b || otherTriangle.c == c) {
        sharedVertices++;
    }
    return sharedVertices == 2;
}