#ifndef VORTEX_UTILS__ROS_CONVERSIONS_HPP_
#define VORTEX_UTILS__ROS_CONVERSIONS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include "math.hpp"

namespace vortex::utils::ros_conversions {

/**
 * @brief Concept describing a generic pose-like type.
 *
 * A type satisfies this concept if it exposes the following fields,
 * all convertible to double:
 * - x, y, z  (position components)
 * - roll, pitch, yaw  (orientation expressed as Euler angles)
 *
 *
 * @tparam T The candidate type to check.
 */
template <typename T>
concept PoseLike = requires(const T& t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
    { t.z } -> std::convertible_to<double>;
    { t.roll } -> std::convertible_to<double>;
    { t.pitch } -> std::convertible_to<double>;
    { t.yaw } -> std::convertible_to<double>;
};

/**
 * @brief Convert a pose-like reference structure into a ROS Pose message.
 *
 * The function reads position (x, y, z) and orientation (roll, pitch, yaw)
 * from the input object and constructs a corresponding
 * `geometry_msgs::msg::Pose`.
 *
 * Orientation is internally converted from Euler angles (roll, pitch, yaw)
 * into a quaternion via `vortex::utils::math::euler_to_quat()`.
 *
 * @tparam T A type satisfying the PoseLike concept.
 * @param ref The input pose-like object.
 * @return A `geometry_msgs::msg::Pose` containing the converted pose.
 */
template <PoseLike T>
geometry_msgs::msg::Pose reference_to_pose(const T& ref) {
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

}  // namespace vortex::utils::ros_conversions

#endif  // VORTEX_UTILS__ROS_CONVERSIONS_HPP_
