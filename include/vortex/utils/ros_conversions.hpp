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
using vortex::utils::types::x_of;
using vortex::utils::types::y_of;
using vortex::utils::types::z_of;

using vortex::utils::types::qw_of;
using vortex::utils::types::qx_of;
using vortex::utils::types::qy_of;
using vortex::utils::types::qz_of;

using vortex::utils::types::pitch_of;
using vortex::utils::types::roll_of;
using vortex::utils::types::yaw_of;

template <PoseLike T>
geometry_msgs::msg::Pose pose_like_to_pose_msg(const T& pose_like) {
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
std::vector<geometry_msgs::msg::Pose> pose_like_to_pose_msgs(const R& poses) {
    std::vector<geometry_msgs::msg::Pose> out;

    if constexpr (std::ranges::sized_range<R>) {
        out.reserve(std::ranges::size(poses));
    }

    for (const auto& p : poses) {
        out.push_back(pose_like_to_pose_msg(p));
    }
    return out;
}

template <typename T, typename U>
concept same_bare_as = std::same_as<std::remove_cvref_t<T>, U>;

template <typename T>
concept ROSPoseLike =
    same_bare_as<T, geometry_msgs::msg::Pose> ||
    same_bare_as<T, geometry_msgs::msg::PoseStamped> ||
    same_bare_as<T, geometry_msgs::msg::PoseWithCovarianceStamped> ||
    same_bare_as<T, geometry_msgs::msg::PoseArray>;

inline Eigen::Matrix<double, 6, 1> ros_pose_to_eigen6d(
    const geometry_msgs::msg::Pose& msg) {
    Eigen::Matrix<double, 6, 1> v;

    v(0) = msg.position.x;
    v(1) = msg.position.y;
    v(2) = msg.position.z;

    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x,
                         msg.orientation.y, msg.orientation.z);

    v.tail<3>() = vortex::utils::math::quat_to_euler(q);
    return v;
}

template <ROSPoseLike T>
Eigen::Matrix<double, 6, Eigen::Dynamic> ros_to_eigen6d(const T& msg) {
    if constexpr (same_bare_as<T, geometry_msgs::msg::Pose>) {
        return ros_pose_to_eigen6d(msg);
    } else if constexpr (same_bare_as<T, geometry_msgs::msg::PoseStamped>) {
        return ros_pose_to_eigen6d(msg.pose);
    } else if constexpr (same_bare_as<
                             T,
                             geometry_msgs::msg::PoseWithCovarianceStamped>) {
        return ros_pose_to_eigen6d(msg.pose.pose);
    } else if constexpr (same_bare_as<T, geometry_msgs::msg::PoseArray>) {
        const size_t n = msg.poses.size();
        Eigen::Matrix<double, 6, Eigen::Dynamic> X(6, n);

        std::ranges::for_each(std::views::iota(size_t{0}, n), [&](size_t i) {
            const auto& pose = msg.poses[i];
            X.col(i) = ros_pose_to_eigen6d(pose);
        });

        return X;
    }
}

using PoseQuatEigen = vortex::utils::types::PoseQuatEigen;

inline PoseQuatEigen ros_pose_to_pose_quat(
    const geometry_msgs::msg::Pose& pose) {
    PoseQuatEigen p;
    p.position = {pose.position.x, pose.position.y, pose.position.z};

    p.orientation = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                                       pose.orientation.y, pose.orientation.z)
                        .normalized();

    return p;
}

template <ROSPoseLike T>
std::vector<PoseQuatEigen> ros_to_pose_quat(const T& msg) {
    if constexpr (same_bare_as<T, geometry_msgs::msg::Pose>) {
        return {ros_pose_to_pose_quat(msg)};
    } else if constexpr (same_bare_as<T, geometry_msgs::msg::PoseStamped>) {
        return ros_to_pose_quat(msg.pose);
    } else if constexpr (same_bare_as<
                             T,
                             geometry_msgs::msg::PoseWithCovarianceStamped>) {
        return ros_to_pose_quat(msg.pose.pose);
    } else if constexpr (same_bare_as<T, geometry_msgs::msg::PoseArray>) {
        std::vector<PoseQuatEigen> poses;
        poses.reserve(msg.poses.size());

        for (const auto& pose : msg.poses) {
            poses.push_back(ros_pose_to_pose_quat(pose));
        }
        return poses;
    }
}

}  // namespace vortex::utils::ros_conversions

#endif  // VORTEX_UTILS__ROS_CONVERSIONS_HPP_
