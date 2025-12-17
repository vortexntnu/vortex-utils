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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "accessors.hpp"
#include "concepts.hpp"
#include "math.hpp"
#include "types.hpp"

namespace vortex::utils::ros_conversions {

using vortex::utils::concepts::EulerPoseLike;
using vortex::utils::concepts::PoseLike;
using vortex::utils::concepts::QuatPoseLike;

template <typename T, typename U>
concept same_bare_as = std::same_as<std::remove_cvref_t<T>, U>;

template <typename T>
concept ROSPoseLike =
    same_bare_as<T, geometry_msgs::msg::Pose> ||
    same_bare_as<T, geometry_msgs::msg::PoseStamped> ||
    same_bare_as<T, geometry_msgs::msg::PoseWithCovarianceStamped> ||
    same_bare_as<T, geometry_msgs::msg::PoseArray>;

template <PoseLike T>
geometry_msgs::msg::Pose to_pose_msg(const T& pose_like) {
    geometry_msgs::msg::Pose pose;

    pose.position.x = x_of(pose_like);
    pose.position.y = y_of(pose_like);
    pose.position.z = z_of(pose_like);

    if constexpr (QuatPoseLike<T>) {
        pose.orientation.w = qw_of(pose_like);
        pose.orientation.x = qx_of(pose_like);
        pose.orientation.y = qy_of(pose_like);
        pose.orientation.z = qz_of(pose_like);
    } else if constexpr (EulerPoseLike<T>) {
        const auto q = vortex::utils::math::euler_to_quat(
            roll_of(pose_like), pitch_of(pose_like), yaw_of(pose_like));
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
    }

    return pose;
}

template <std::ranges::input_range R>
    requires PoseLike<std::ranges::range_value_t<R>>
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

template <ROSPoseLike T>
std::vector<vortex::utils::types::Pose> ros_to_pose_vec(const T& msg) {
    if constexpr (same_bare_as<T, geometry_msgs::msg::Pose>) {
        return {ros_pose_to_pose(msg)};
    } else if constexpr (same_bare_as<T, geometry_msgs::msg::PoseStamped>) {
        return ros_pose_to_pose(msg.pose);
    } else if constexpr (same_bare_as<
                             T,
                             geometry_msgs::msg::PoseWithCovarianceStamped>) {
        return ros_pose_to_pose(msg.pose.pose);
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
