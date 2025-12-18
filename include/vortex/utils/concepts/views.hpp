#ifndef VORTEX_UTILS__VIEWS_HPP_
#define VORTEX_UTILS__VIEWS_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "accessors.hpp"
#include "concepts.hpp"

namespace vortex::utils::views {

using vortex::utils::concepts::EulerLike;
using vortex::utils::concepts::EulerPoseLike;
using vortex::utils::concepts::PositionLike;
using vortex::utils::concepts::QuaternionLike;
using vortex::utils::concepts::QuatPoseLike;

/**
 * @brief Extracts the position components of an object as an Eigen vector.
 *
 *
 * @tparam T Type satisfying PositionLike
 * @param t Object providing positional components
 * @return Eigen::Vector3d containing (x, y, z)
 */
template <PositionLike T>
inline Eigen::Vector3d pos_vector(const T& t) {
    return Eigen::Vector3d{x_of(t), y_of(t), z_of(t)};
}

/**
 * @brief Extracts Euler orientation components as an Eigen vector.
 *
 *
 * @tparam T Type satisfying EulerLike
 * @param t Object providing Euler orientation components
 * @return Eigen::Vector3d containing (roll, pitch, yaw)
 */
template <EulerLike T>
inline Eigen::Vector3d ori_vector(const T& t) {
    return Eigen::Vector3d{roll_of(t), pitch_of(t), yaw_of(t)};
}

/**
 * @brief Extracts a quaternion orientation as an Eigen quaternion.
 *
 * @tparam T Type satisfying QuaternionLike
 * @param t Object providing quaternion components
 * @return Eigen::Quaterniond representing the orientation
 */
template <QuaternionLike T>
inline Eigen::Quaterniond ori_quaternion(const T& t) {
    return Eigen::Quaterniond{qw_of(t), qx_of(t), qy_of(t), qz_of(t)};
}

/**
 * @brief Converts a pose with Euler orientation to a 6-D vector.
 *
 * The resulting vector layout is:
 * `[x, y, z, roll, pitch, yaw]`.
 *
 * @tparam T Type satisfying EulerPoseLike
 * @param t Object providing position and Euler orientation
 * @return Eigen::Vector<double, 6> representing the pose
 */
template <EulerPoseLike T>
inline Eigen::Vector<double, 6> to_vector(const T& t) {
    return Eigen::Vector<double, 6>{x_of(t),    y_of(t),     z_of(t),
                                    roll_of(t), pitch_of(t), yaw_of(t)};
}

/**
 * @brief Converts a pose with quaternion orientation to a 7-D vector.
 *
 * The resulting vector layout is:
 * `[x, y, z, qw, qx, qy, qz]`.
 *
 * @tparam T Type satisfying QuatPoseLike
 * @param t Object providing position and quaternion orientation
 * @return Eigen::Vector<double, 7> representing the pose
 */
template <QuatPoseLike T>
inline Eigen::Vector<double, 7> to_vector(const T& t) {
    return Eigen::Vector<double, 7>{x_of(t),  y_of(t),  z_of(t), qw_of(t),
                                    qx_of(t), qy_of(t), qz_of(t)};
}

}  // namespace vortex::utils::views

#endif  // VORTEX_UTILS__VIEWS_HPP_
