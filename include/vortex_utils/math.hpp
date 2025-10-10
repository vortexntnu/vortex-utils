#ifndef VORTEX_UTILS_MATH_HPP
#define VORTEX_UTILS_MATH_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>

namespace Eigen {
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 3, 3> Matrix3d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

namespace vortex::utils::math {

// @brief Function to calculate the smallest signed angle between two angles.
// Maps the angle to the interval [-pi, pi].
double ssa(const double angle);

// @brief Calculates the skew-symmetric matrix from a 3D vector.
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& vector);

// @brief Converts a quaternion to Euler angles.
Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q);

}  // namespace vortex::utils::math

#endif  // VORTEX_UTILS_MATH_HPP
