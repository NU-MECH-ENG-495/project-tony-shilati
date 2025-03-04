#include <iostream>
#include <Eigen/Dense>

int main() {
    // Create a 3x3 matrix of doubles
    Eigen::Matrix3d matrix;

    // Initialize the matrix
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;

    // Print the matrix
    std::cout << "Matrix:\n" << matrix << std::endl;

    return 0;
}
