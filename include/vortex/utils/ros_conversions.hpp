#ifndef ROS_CONVERSIONS_HPP
#define ROS_CONVERSIONS_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "vortex/utils/types.hpp"
#include "vortex/utils/math.hpp"
#include <ranges>
#include <concepts>
#include <vector>

namespace vortex::utils::ros_conversions {

template <typename T>
concept PoseLike =
    std::same_as<T, geometry_msgs::msg::Pose> ||
    std::same_as<T, geometry_msgs::msg::PoseArray> ||
    std::same_as<T, geometry_msgs::msg::PoseStamped> ||
    std::same_as<T, geometry_msgs::msg::PoseWithCovarianceStamped>;

// @brief Convert various ROS pose messages to Eigen 6D vectors/matrices
//        Each column is [x, y, z, roll, pitch, yaw]^T
inline Eigen::Matrix<double, 6, Eigen::Dynamic> ros_to_eigen6d(const PoseLike auto& msg)
{
    using T = std::decay_t<decltype(msg)>;

    if constexpr (std::same_as<T, geometry_msgs::msg::Pose>) {
        Eigen::Matrix<double, 6, 1> v;
        v(0) = msg.position.x;
        v(1) = msg.position.y;
        v(2) = msg.position.z;

        // ROS/tf2 uses (x, y, z, w)
        // while Eigen stores quaternions as (w, x, y, z)
        Eigen::Quaterniond q(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z);

        const Eigen::Vector3d euler = vortex::utils::math::quat_to_euler(q);
        v.tail<3>() = euler;

        return v;
    }

    else if constexpr (std::same_as<T, geometry_msgs::msg::PoseStamped>) {
        return ros_to_eigen6d(msg.pose);
    }

    else if constexpr (std::same_as<T, geometry_msgs::msg::PoseWithCovarianceStamped>) {
        return ros_to_eigen6d(msg.pose.pose);
    }

    else if constexpr (std::same_as<T, geometry_msgs::msg::PoseArray>) {
        const size_t n = msg.poses.size();
        Eigen::Matrix<double, 6, Eigen::Dynamic> X(6, n);

        std::ranges::for_each(
            std::views::iota(size_t{0}, n),
            [&](size_t i) {
                const auto& pose = msg.poses[i];
                X.col(i) = ros_to_eigen6d(pose);
            });

        return X;
    }
}

// @brief Convert an Eigen::Vector6d [x, y, z, roll, pitch, yaw] to a ROS Pose
inline geometry_msgs::msg::Pose eigen6d_to_pose(const Eigen::Vector6d& v)
{
    geometry_msgs::msg::Pose pose;

    pose.position.x = v(0);
    pose.position.y = v(1);
    pose.position.z = v(2);

    const double roll  = v(3);
    const double pitch = v(4);
    const double yaw   = v(5);

    Eigen::Quaterniond q = vortex::utils::math::euler_to_quat(roll, pitch, yaw);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

}  // namespace vortex::utils::ros_conversions

#endif // ROS_CONVERSIONS_HPP