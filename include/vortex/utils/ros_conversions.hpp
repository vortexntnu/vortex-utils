#ifndef VORTEX_UTILS__ROS_CONVERSIONS_HPP_
#define VORTEX_UTILS__ROS_CONVERSIONS_HPP_

#include <concepts>
#include <ranges>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "concepts.hpp"
#include "math.hpp"
#include "types.hpp"

namespace vortex::utils::ros_conversions {

/**
 * @brief Helper concept to check if two types are the same
 *        after removing cv-ref qualifiers.
 *
 * @tparam T First type.
 * @tparam U Second type.
 */
template <typename T, typename U>
concept same_bare_as = std::same_as<std::remove_cvref_t<T>, U>;

/**
 * @brief Concept describing ROS pose message types supported by
 * ros_to_eigen6d().
 *
 * Supported types are:
 *
 *  - `geometry_msgs::msg::Pose`
 *
 *  - `geometry_msgs::msg::PoseStamped`
 *
 *  - `geometry_msgs::msg::PoseWithCovariance`
 *
 *  - `geometry_msgs::msg::PoseWithCovarianceStamped`
 *
 *  - `geometry_msgs::msg::PoseArray`
 *
 * @tparam T  The candidate type to test.
 */
template <typename T>
concept ROSPoseLike =
    same_bare_as<T, geometry_msgs::msg::Pose> ||
    same_bare_as<T, geometry_msgs::msg::PoseStamped> ||
    same_bare_as<T, geometry_msgs::msg::PoseWithCovariance> ||
    same_bare_as<T, geometry_msgs::msg::PoseWithCovarianceStamped> ||
    same_bare_as<T, geometry_msgs::msg::PoseArray>;

/**
 * @brief Converts a PoseLike object to a ROS geometry_msgs::msg::Pose.
 *
 * The input type must satisfy PoseLike, i.e. provide position components
 * and exactly one orientation representation (Euler or quaternion).
 *
 * @tparam T Type satisfying PoseLike
 * @param pose_like Input pose object
 * @return geometry_msgs::msg::Pose ROS pose message
 */
template <vortex::utils::concepts::PoseLike T>
geometry_msgs::msg::Pose to_pose_msg(const T& pose_like) {
    geometry_msgs::msg::Pose pose;

    pose.position.x = pose_like.x;
    pose.position.y = pose_like.y;
    pose.position.z = pose_like.z;

    if constexpr (vortex::utils::concepts::QuatPoseLike<T>) {
        pose.orientation.w = pose_like.qw;
        pose.orientation.x = pose_like.qx;
        pose.orientation.y = pose_like.qy;
        pose.orientation.z = pose_like.qz;
    } else if constexpr (vortex::utils::concepts::EulerPoseLike<T>) {
        const auto q = vortex::utils::math::euler_to_quat(
            pose_like.roll, pose_like.pitch, pose_like.yaw);
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
    }

    return pose;
}

/**
 * @brief Converts a range of PoseLike objects to a vector of ROS pose messages.
 *
 * The input range may be any std::ranges::input_range whose value type
 * satisfies PoseLike (e.g. std::vector, std::array, std::span, or views).
 *
 * @tparam R Range type whose value_type satisfies PoseLike
 * @param poses Range of pose-like objects
 * @return std::vector<geometry_msgs::msg::Pose> Converted ROS poses
 */
template <std::ranges::input_range R>
    requires vortex::utils::concepts::PoseLike<std::ranges::range_value_t<R>>
std::vector<geometry_msgs::msg::Pose> to_pose_msgs(const R& poses) {
    std::vector<geometry_msgs::msg::Pose> out;

    if constexpr (std::ranges::sized_range<R>) {
        out.reserve(std::ranges::size(poses));
    }

    for (const auto& p : poses) {
        out.push_back(to_pose_msg(p));
    }
    return out;
}

/**
 * @brief Converts a ROS geometry_msgs::msg::Pose to an internal Pose type.
 * @param pose ROS pose message
 * @return vortex::utils::types::Pose Internal pose representation
 */
inline vortex::utils::types::Pose ros_pose_to_pose(
    const geometry_msgs::msg::Pose& pose) {
    vortex::utils::types::Pose p;
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = pose.position.z;

    p.qw = pose.orientation.w;
    p.qx = pose.orientation.x;
    p.qy = pose.orientation.y;
    p.qz = pose.orientation.z;
    return p;
}

/**
 * @brief Extracts one or more internal Pose objects from a ROS pose message.
 *
 * Supported input types are:
 *
 *  - `geometry_msgs::msg::Pose`
 *
 *  - `geometry_msgs::msg::PoseStamped`
 *
 *  - `geometry_msgs::msg::PoseWithCovariance`
 *
 *  - `geometry_msgs::msg::PoseWithCovarianceStamped`
 *
 *  - `geometry_msgs::msg::PoseArray`
 *
 * Messages containing a single pose produce a vector with one element.
 * PoseArray messages produce a vector with one element per pose.
 *
 * @tparam T ROS message type satisfying ROSPoseLike
 * @param msg ROS pose message
 * @return std::vector<vortex::utils::types::Pose> Extracted internal poses
 */
template <ROSPoseLike T>
std::vector<vortex::utils::types::Pose> ros_to_pose_vec(const T& msg) {
    if constexpr (same_bare_as<T, geometry_msgs::msg::Pose>) {
        return {ros_pose_to_pose(msg)};
    } else if constexpr (same_bare_as<T, geometry_msgs::msg::PoseStamped>) {
        return {ros_pose_to_pose(msg.pose)};
    } else if constexpr (same_bare_as<T,
                                      geometry_msgs::msg::PoseWithCovariance>) {
        return {ros_pose_to_pose(msg.pose)};
    } else if constexpr (same_bare_as<
                             T,
                             geometry_msgs::msg::PoseWithCovarianceStamped>) {
        return {ros_pose_to_pose(msg.pose.pose)};
    } else if constexpr (same_bare_as<T, geometry_msgs::msg::PoseArray>) {
        std::vector<vortex::utils::types::Pose> poses;
        poses.reserve(msg.poses.size());

        for (const auto& pose : msg.poses) {
            poses.push_back(ros_pose_to_pose(pose));
        }
        return poses;
    }
}

}  // namespace vortex::utils::ros_conversions

#endif  // VORTEX_UTILS__ROS_CONVERSIONS_HPP_
