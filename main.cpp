#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

int main() {
    Eigen::Matrix<double, 3, 3> matrix;
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;
    Eigen::Matrix<double, 3, 3> anotherMatrix;
    anotherMatrix << matrix;
    std::cout << matrix * anotherMatrix;
    return 0;
}