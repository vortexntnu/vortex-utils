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

// @brief Convert a vortex::utils::types::Pose to a ROS PoseStamped message
inline geometry_msgs::msg::PoseStamped pose_to_ros(const vortex::utils::types::Pose& pose) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = pose.position(0);
    pose_stamped.pose.position.y = pose.position(1);
    pose_stamped.pose.position.z = pose.position(2);
    // Eigen stores quaternions as (w, x, y, z)
    // while ROS/tf2 uses (x, y, z, w)
    pose_stamped.pose.orientation.x = pose.orientation.x();
    pose_stamped.pose.orientation.y = pose.orientation.y();
    pose_stamped.pose.orientation.z = pose.orientation.z();
    pose_stamped.pose.orientation.w = pose.orientation.w();
    return pose_stamped;
}

// @brief Convert a ROS PoseStamped message to a vortex::utils::Pose
inline vortex::utils::types::Pose ros_to_pose(const geometry_msgs::msg::PoseStamped& pose_stamped) {
    Eigen::Vector3d position;
    position(0) = pose_stamped.pose.position.x;
    position(1) = pose_stamped.pose.position.y;
    position(2) = pose_stamped.pose.position.z;

    Eigen::Quaterniond orientation;
    // ROS/tf2 uses (x, y, z, w)
    // while Eigen stores quaternions as (w, x, y, z)
    orientation.x() = pose_stamped.pose.orientation.x;
    orientation.y() = pose_stamped.pose.orientation.y;
    orientation.z() = pose_stamped.pose.orientation.z;
    orientation.w() = pose_stamped.pose.orientation.w;

    return vortex::utils::types::Pose(position, orientation);
}

// @brief Convert a vector of vortex::utils::types::Pose to a ROS PoseArray
inline geometry_msgs::msg::PoseArray poses_to_ros(const std::vector<vortex::utils::types::Pose>& poses) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.poses.reserve(poses.size());

    std::ranges::transform(poses, std::back_inserter(pose_array.poses),
        [](const auto& pose) {
            return pose_to_ros(pose).pose;
        });

    return pose_array;
}

// @brief Convert a ROS PoseArray to  vector<vortex::utils::types::Pose>
inline std::vector<vortex::utils::types::Pose> ros_to_poses(const geometry_msgs::msg::PoseArray& pose_array) {
    std::vector<vortex::utils::types::Pose> poses;
    poses.reserve(pose_array.poses.size());

    std::ranges::transform(pose_array.poses, std::back_inserter(poses),
        [](const geometry_msgs::msg::Pose& ros_pose) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose = ros_pose;
            return ros_to_pose(pose_stamped);
        });

    return poses;
}

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

}  // namespace vortex::utils::ros_conversions

#endif // ROS_CONVERSIONS_HPP