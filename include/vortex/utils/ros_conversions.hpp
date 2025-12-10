#ifndef VORTEX_UTILS__ROS_CONVERSIONS_HPP_
#define VORTEX_UTILS__ROS_CONVERSIONS_HPP_

#include <concepts>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>

#include "math.hpp"

namespace vortex::utils::ros_conversions {

/**
 * @brief Concept describing an Euler pose type expressed in XYZ + RPY form.
 *
 * A type satisfies this concept if it exposes the following fields,
 * all convertible to double:
 * - x, y, z  (position components)
 * - roll, pitch, yaw  (orientation expressed as Euler/RPY angles)
 *
 *
 * @tparam T The candidate type to check.
 */
template <typename T>
concept EulerPoseLike = requires(const T& t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
    { t.z } -> std::convertible_to<double>;

    { t.roll } -> std::convertible_to<double>;
    { t.pitch } -> std::convertible_to<double>;
    { t.yaw } -> std::convertible_to<double>;
};

/**
 * @brief Concept describing a pose type expressed in XYZ + quaternion form.
 *
 * A type satisfies this concept if it exposes the following fields:
 *  - x, y, z              (position components)
 *  - qw, qx, qy, qz       (quaternion orientation)
 *
 *
 * @tparam T The candidate type to check.
 */
template <typename T>
concept QuatPoseLike = requires(const T& t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
    { t.z } -> std::convertible_to<double>;

    { t.qw } -> std::convertible_to<double>;
    { t.qx } -> std::convertible_to<double>;
    { t.qy } -> std::convertible_to<double>;
    { t.qz } -> std::convertible_to<double>;
};

/**
 * @brief Concept for Eigen-based 6-vector Euler poses:
 *        [x, y, z, roll, pitch, yaw].
 *
 * Accepts:
 *   - Eigen::Matrix<double, 6, 1>
 *
 *
 * @tparam T The candidate type.
 */
template <typename T>
concept Eigen6dEuler = std::same_as<T, Eigen::Matrix<double, 6, 1>>;

/**
 * @brief Master concept representing any supported pose-like structure.
 *
 * A type satisfies PoseLike if it matches *any* of the following:
 *   - EulerPoseLike  (XYZ + RPY)
 *   - QuatPoseLike   (XYZ + quaternion)
 *   - Eigen6dEuler   (Matrix<6,1>)
 *
 *
 * @tparam T The candidate pose type.
 */
template <typename T>
concept PoseLike = EulerPoseLike<T> || QuatPoseLike<T> || Eigen6dEuler<T>;

/**
 * @brief Convert a Euler pose-like structure into a ROS Pose message.
 *
 * The function reads position (x, y, z) and orientation (roll, pitch, yaw)
 * from the input object and constructs a corresponding
 * `geometry_msgs::msg::Pose`.
 *
 * Orientation is internally converted from Euler angles (roll, pitch, yaw)
 * into a quaternion via `vortex::utils::math::euler_to_quat()`.
 *
 * @tparam T A type satisfying the `EulerPoseLike` concept.
 * @param ref The input `EulerPoseLike` object.
 * @return A `geometry_msgs::msg::Pose` containing the converted pose.
 */
template <EulerPoseLike T>
geometry_msgs::msg::Pose pose_like_to_pose_msg(const T& ref) {
    geometry_msgs::msg::Pose pose;

    pose.position.x = ref.x;
    pose.position.y = ref.y;
    pose.position.z = ref.z;

    Eigen::Quaterniond quat =
        vortex::utils::math::euler_to_quat(ref.roll, ref.pitch, ref.yaw);

    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    return pose;
}

/**
 * @brief Convert a quaternion pose-like structure into a ROS Pose message.
 *
 * The function reads position (x, y, z) and orientation (qw, qx, qy, qz)
 * from the input object and constructs a corresponding
 * `geometry_msgs::msg::Pose`.
 *
 * @tparam T A type satisfying the `QuatPoseLike` concept.
 * @param ref The input `QuatPoseLike` object.
 * @return A `geometry_msgs::msg::Pose` containing the converted pose.
 */
template <QuatPoseLike T>
geometry_msgs::msg::Pose pose_like_to_pose_msg(const T& ref) {
    geometry_msgs::msg::Pose pose;

    pose.position.x = ref.x;
    pose.position.y = ref.y;
    pose.position.z = ref.z;

    pose.orientation.x = ref.qx;
    pose.orientation.y = ref.qy;
    pose.orientation.z = ref.qz;
    pose.orientation.w = ref.qw;

    return pose;
}

/**
 * @brief Convert an Eigen 6d-vector [x, y, z, roll, pitch, yaw]
 *        into a ROS Pose message.
 *
 * Orientation is internally converted from v(3), v(4), v(5)
 * into a quaternion via `vortex::utils::math::euler_to_quat()`.
 *
 * @tparam T A type satisfying the `Eigen6dEuler` concept.
 * @param v The 6d-vector containing position and Euler angles.
 * @return A `geometry_msgs::msg::Pose` containing the converted pose.
 */
template <Eigen6dEuler T>
geometry_msgs::msg::Pose pose_like_to_pose_msg(const T& v) {
    geometry_msgs::msg::Pose pose;

    pose.position.x = v(0);
    pose.position.y = v(1);
    pose.position.z = v(2);

    Eigen::Quaterniond q = vortex::utils::math::euler_to_quat(v(3), v(4), v(5));

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

}  // namespace vortex::utils::ros_conversions

#endif  // VORTEX_UTILS__ROS_CONVERSIONS_HPP_
