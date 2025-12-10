#ifndef VORTEX_UTILS_MATH_HPP
#define VORTEX_UTILS_MATH_HPP

#include <cmath>
#include <concepts>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace vortex::utils::math {

/**
 * @brief Function to calculate the smallest signed angle.
 * Maps the angle to the interval (-pi, pi].
 * @param angle The input angle in radians.
 * @return The smallest signed angle in radians.
 */
double ssa(const double angle);

/**
 * @brief Function to calculate the smallest signed angles in a container.
 * Maps the angles to the interval [-pi, pi]. Works with containers like
 * std::vector, std::array, Eigen::VectorXd etc.
 * @param angles The input angles in radians.
 * @return The smallest signed angles in radians.
 */
template <typename T>
concept VectorLike = !std::is_arithmetic_v<std::decay_t<T>> && requires(T t) {
    { *std::begin(t) } -> std::convertible_to<double>;
};
template <VectorLike Container>
auto ssa(Container&& angles_in) {
    using ContainerT = std::decay_t<Container>;
    using std::begin;

    ContainerT angles = std::forward<Container>(angles_in);

    using value_type = std::decay_t<decltype(*begin(angles))>;
    static_assert(std::is_floating_point_v<value_type>);

    for (auto& a : angles) {
        a = static_cast<value_type>(ssa(static_cast<double>(a)));
    }

    return angles;
}

/**
 * @brief Calculates the skew-symmetric matrix from a 3D vector.
 * @param vector Eigen::Vector3d
 * @return Eigen::Matrix3d skew-symmetric matrix
 */
Eigen::Matrix3d get_skew_symmetric_matrix(const Eigen::Vector3d& vector);

/**
 * @brief Rotation matrix from euler angles
 * @param roll Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw Yaw angle in radians.
 * @return Eigen::Matrix3d rotation matrix
 */
Eigen::Matrix3d get_rotation_matrix(const double roll,
                                    const double pitch,
                                    const double yaw);

/**
 * @brief Fossen, 2021 eq. 2.41
 * @param roll Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @return Eigen::Matrix3d transformation matrix
 */
Eigen::Matrix3d get_transformation_matrix_attitude(const double roll,
                                                   const double pitch);

/**
 * @brief Fossen, 2021 eq. 2.78
 * @param quat Eigen::Quaterniond
 * @return Eigen::Matrix<double, 4, 3> transformation matrix
 */
Eigen::Matrix<double, 4, 3> get_transformation_matrix_attitude_quat(
    const Eigen::Quaterniond& quat);

/**
 * @brief Converts an Eigen::Vector3d with euler angles to Eigen::Quaterniond
 * using the axis-angle representation.
 * @param vector Eigen::Vector3d with roll, pitch, yaw
 * @return Eigen::Quaterniond
 */
Eigen::Quaterniond eigen_vector3d_to_quaternion(const Eigen::Vector3d& vector);

/**
 * @brief Converts a quaternion to Euler angles.
 * @param q Eigen::Quaterniond
 * @return Eigen::Vector3d with roll, pitch, yaw
 */
Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q);

/**
 * @brief Converts Euler angles to quaternion.
 * @param roll Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw Yaw angle in radians.
 */
Eigen::Quaterniond euler_to_quat(const double roll,
                                 const double pitch,
                                 const double yaw);

/**
 * @brief Converts Eigen::Vector3d with euler angles to quaternion.
 * @param euler Eigen::Vector3d with roll, pitch, yaw
 * @return Eigen::Quaterniond
 */
Eigen::Quaterniond euler_to_quat(const Eigen::Vector3d& euler);

/**
 * @brief Computes the Moore-Penrose pseudo-inverse of a matrix.
 * @param matrix The input matrix to be inverted.
 * @return The pseudo-inverse of the input matrix.
 */
Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd& matrix);

/**
 * @brief Clamps the values of a vector between min_val and max_val.
 * @param values The input vector.
 * @param min_val The minimum value.
 * @param max_val The maximum value.
 * @return The clamped vector.
 */

Eigen::VectorXd clamp_values(const Eigen::VectorXd& values,
                             const double min_val,
                             const double max_val);

/**
 * @brief Anti-windup for integral term in PID controller.
 * @param dt Time step.
 * @param error The current error vector.
 * @param integral The current integral vector.
 * @param min_val Minimum value for clamping.
 * @param max_val Maximum value for clamping.
 * @return The updated integral vector after applying anti-windup.
 */
Eigen::VectorXd anti_windup(const double dt,
                            const Eigen::VectorXd& error,
                            const Eigen::VectorXd& integral,
                            const double min_val,
                            const double max_val);
}  // namespace vortex::utils::math

#endif  // VORTEX_UTILS_MATH_HPP
