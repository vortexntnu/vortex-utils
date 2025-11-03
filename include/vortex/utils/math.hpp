#ifndef VORTEX_UTILS_MATH_HPP
#define VORTEX_UTILS_MATH_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace vortex::utils::math {

// @brief Function to calculate the smallest signed angle between two angles.
// Maps the angle to the interval [-pi, pi].
double ssa(const double angle);

// @brief Calculates the skew-symmetric matrix from a 3D vector.
Eigen::Matrix3d get_skew_symmetric_matrix(const Eigen::Vector3d& vector);

// @brief Rotation matrix from euler angles
Eigen::Matrix3d get_rotation_matrix(const double roll,
                                    const double pitch,
                                    const double yaw);

// @brief Fossen, 2021 eq. 2.41
Eigen::Matrix3d get_transformation_matrix_attitude(const double roll,
                                                   const double pitch);

// @brief Fossen, 2021 eq. 2.78
Eigen::Matrix<double, 4, 3> get_transformation_matrix_attitude_quat(
    const Eigen::Quaterniond& quat);

// @brief Converts an Eigen::Vector3d with euler angles to Eigen::Quaterniond
Eigen::Quaterniond eigen_vector3d_to_quaternion(const Eigen::Vector3d& vector);

// @brief Converts a quaternion to Euler angles.
Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q);

// @brief Converts Euler angles to quaternion
Eigen::Quaterniond euler_to_quat(const double roll,
                                 const double pitch,
                                 const double yaw);

// @brief Converts Eigen::Vector3d with euler angles to quaternion
Eigen::Quaterniond euler_to_quat(const Eigen::Vector3d& euler);

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd& matrix);

Eigen::VectorXd clamp_values(const Eigen::VectorXd& values,
                             const double min_val,
                             const double max_val);

Eigen::VectorXd anti_windup(const double dt,
                            const Eigen::VectorXd& error,
                            const Eigen::VectorXd& integral,
                            const double min_val,
                            const double max_val);
}  // namespace vortex::utils::math

#endif  // VORTEX_UTILS_MATH_HPP
